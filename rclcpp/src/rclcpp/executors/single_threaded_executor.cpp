// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/tcl_node_interfaces/node_release_timer.hpp"

static
void
take_and_do_error_handling(
  const char * action_description,
  const char * topic_or_service_name,
  std::function<bool()> take_action,
  std::function<void()> handle_action)
{
  bool taken = false;
  try {
    taken = take_action();
  } catch (const rclcpp::exceptions::RCLError & rcl_error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "executor %s '%s' unexpectedly failed: %s",
      action_description,
      topic_or_service_name,
      rcl_error.what());
  }
  if (taken) {
    handle_action();
  } else {
    // Message or Service was not taken for some reason.
    // Note that this can be normal, if the underlying middleware needs to
    // interrupt wait spuriously it is allowed.
    // So in that case the executor cannot tell the difference in a
    // spurious wake up and an entity actually having data until trying
    // to take the data.
    RCLCPP_DEBUG(
      rclcpp::get_logger("rclcpp"),
      "executor %s '%s' failed to take anything",
      action_description,
      topic_or_service_name);
  }
}

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable_for_tcl(any_executable);
      if(use_blocking_io_ && is_condition_satisfied_)
      {
        reset_blocking_condition_map();
        is_condition_satisfied_ = false;
        this->sleep();
      }
    }
    else
    {
      if(!use_blocking_io_)
        this->sleep();
    }
  }
}

void
SingleThreadedExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  execution_start_time_ = get_now();
  return this->spin_some_impl(max_duration, false);
}

void
SingleThreadedExecutor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error(
            std::string("Node '") + node_ptr->get_fully_qualified_name() +
            "' has already been added to an executor.");
  }
  std::lock_guard<std::mutex> guard{mutex_};
  node_ptr->for_each_callback_group(
    [this, node_ptr, notify](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      if (!group_ptr->get_associated_with_executor_atomic().load() &&
      group_ptr->automatically_add_to_executor_with_node())
      {
        this->add_callback_group_to_map(
          group_ptr,
          node_ptr,
          weak_groups_to_nodes_associated_with_executor_,
          notify);
      }
    });

  weak_nodes_.push_back(node_ptr);
}

void
SingleThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  add_node(node_ptr->get_node_base_interface(), notify);
  add_node_timing_interface(node_ptr->get_node_timing_interface());
}

void
SingleThreadedExecutor::sleep()
{
  if(profile_enabled_)
    publish_profile_data();
  if(tcl_timer_)
    tcl_timer_->sleep();
}

void
SingleThreadedExecutor::add_node_timing_interface(rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing_ptr)
{
  use_tcl_ = node_timing_ptr->use_tcl();
  use_blocking_io_ = node_timing_ptr->use_blocking_io();
  profile_enabled_ = node_timing_ptr->profile_enabled();

  if(use_tcl_)
  {
    
    if(use_blocking_io_)
    {
      auto blocking_topics = node_timing_ptr->get_blocking_topics();

      std::for_each(blocking_topics.begin(), blocking_topics.end(), [&](auto& topic)
      {
        blocking_condition_map_[topic] = false;
      });
    }

    if(profile_enabled_)
    {
      tcl_profiler_ = node_timing_ptr->get_profiler();
      tcl_timing_propagator_ = node_timing_ptr->get_timing_propagator();
    }
        
    tcl_timer_ = node_timing_ptr->get_timer();
  }
}

void
SingleThreadedExecutor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive)
{
  auto start = std::chrono::steady_clock::now();
  auto max_duration_not_elapsed = [max_duration, start]() {
      if (std::chrono::nanoseconds(0) == max_duration) {
        // told to spin forever if need be
        return true;
      } else if (std::chrono::steady_clock::now() - start < max_duration) {
        // told to spin only for some maximum amount of time
        return true;
      }
      // spun too long
      return false;
    };

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  bool work_available = false;
  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (!work_available) {
      wait_for_work(std::chrono::milliseconds::zero());
    }
    if (get_next_ready_executable(any_exec)) {
      execute_any_executable_for_tcl(any_exec);
      work_available = true;
    } else {
      if (!work_available)
      {
        if(!use_blocking_io_)
        {
          if(!exhaustive)
          {
            break;
          }
        }
        else
        {
          if(is_condition_satisfied_)
          {
            reset_blocking_condition_map();
            is_condition_satisfied_ = false;
            break;
          }
        }
      }
      work_available = false;
    }
  }
}

void
SingleThreadedExecutor::execute_any_executable_for_tcl(AnyExecutable & any_exec)
{
  if (!spinning.load()) {
    return;
  }

  if (any_exec.subscription) {
    TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.subscription->get_subscription_handle().get()));
    if(use_tcl_ && use_blocking_io_)
    {
      add_subscription_to_buffer(any_exec.subscription);
      update_blocking_condition_map(any_exec.subscription->get_topic_name());
    }
    else
    {
      execute_subscription(any_exec.subscription);
    }
  }
  if (any_exec.timer) {
    TRACEPOINT(
      rclcpp_executor_execute,
      static_cast<const void *>(any_exec.timer->get_timer_handle().get()));
    if(use_tcl_ && use_blocking_io_)
      add_timer_to_buffer(any_exec.timer);
    else
    {
      execution_start_time_ = get_now();
      execute_timer(any_exec.timer);
    }
  }
  if (any_exec.service) {
    execute_service(any_exec.service);
  }
  if (any_exec.client) {
    execute_client(any_exec.client);
  }
  if (any_exec.waitable) {
    any_exec.waitable->execute(any_exec.data);
  }

  if(use_blocking_io_ && check_blocking_condition())
  {
    handle_buffer();
    is_condition_satisfied_ = true;
  }
  // Reset the callback_group, regardless of type
  any_exec.callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  try {
    interrupt_guard_condition_.trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string(
              "Failed to trigger guard condition from execute_any_executable: ") + ex.what());
  }
}

void
SingleThreadedExecutor::add_subscription_to_buffer(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;

  if (subscription->is_serialized()) {
    // This is the case where a copy of the serialized message is taken from
    // the middleware via inter-process communication.
    std::shared_ptr<SerializedMessage> serialized_msg = subscription->create_serialized_message();
    take_and_do_error_handling(
      "taking a serialized message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_serialized(*serialized_msg.get(), message_info);},
      [&]()
      {
        subscription->handle_serialized_message(serialized_msg, message_info);
      });
    subscription->return_serialized_message(serialized_msg);
  } else if (subscription->can_loan_messages()) {
    // This is the case where a loaned message is taken from the middleware via
    // inter-process communication, given to the user for their callback,
    // and then returned.
    void * loaned_msg = nullptr;
    // TODO(wjwwood): refactor this into methods on subscription when LoanedMessage
    //   is extened to support subscriptions as well.
    take_and_do_error_handling(
      "taking a loaned message from topic",
      subscription->get_topic_name(),
      [&]()
      {
        rcl_ret_t ret = rcl_take_loaned_message(
          subscription->get_subscription_handle().get(),
          &loaned_msg,
          &message_info.get_rmw_message_info(),
          nullptr);
        if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
          return false;
        } else if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
        return true;
      },
      [&]() {subscription->handle_loaned_message(loaned_msg, message_info);});
    if (nullptr != loaned_msg) {
      rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
        subscription->get_subscription_handle().get(),
        loaned_msg);
      if (RCL_RET_OK != ret) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "rcl_return_loaned_message_from_subscription() failed for subscription on topic '%s': %s",
          subscription->get_topic_name(), rcl_get_error_string().str);
      }
      loaned_msg = nullptr;
    }
  } else {
    // This case is taking a copy of the message data from the middleware via
    // inter-process communication.
    std::shared_ptr<void> message = subscription->create_message();
    std::string topic_name_str = std::string(subscription->get_topic_name());

    if(topic_name_str == "/parameter_events" || topic_name_str == "/tf" || topic_name_str == "/tf_static")
    {
      take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
    }
    else
    {
      take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() 
      { 
        topic_subscription_objects_[topic_name_str] = std::make_shared<subscription_object_t>(
          subscription, message, message_info);
        topic_subscription_order_.push_back(topic_name_str);
      });
    }
    subscription->return_message(message);
  }
}

void
SingleThreadedExecutor::add_timer_to_buffer(rclcpp::TimerBase::SharedPtr timer)
{
  timer_objects_.push_back(timer);
}

void
SingleThreadedExecutor::handle_buffer()
{
  execution_start_time_ = get_now();
  
  handle_subscription_buffer();
  handle_timer_buffer();
}

void
SingleThreadedExecutor::handle_subscription_buffer()
{
  std::for_each(topic_subscription_order_.begin(), topic_subscription_order_.end(), [&](auto& name)
  {
    topic_subscription_objects_[name]->subscription->handle_message(topic_subscription_objects_[name]->message, topic_subscription_objects_[name]->message_info);
  });

  topic_subscription_order_.clear();
  topic_subscription_objects_.clear();
}

void
SingleThreadedExecutor::handle_timer_buffer()
{
  std::for_each(timer_objects_.begin(), timer_objects_.end(), [&](auto& timer)
  {
    execute_timer(timer);
  });
  timer_objects_.clear();
}

bool
SingleThreadedExecutor::check_blocking_condition()
{
  for(auto& it : blocking_condition_map_)
  {
    if(it.second == false)
      return false;
  }

  return true;
}

void
SingleThreadedExecutor::update_blocking_condition_map(const char * topic_name)
{
  std::string topic_name_str = std::string(topic_name);

  std::for_each(blocking_condition_map_.begin(), blocking_condition_map_.end(), [&](auto& it)
  {
    if(it.first == topic_name_str)
      it.second = true;
  });
}

void
SingleThreadedExecutor::reset_blocking_condition_map()
{
  std::for_each(blocking_condition_map_.begin(), blocking_condition_map_.end(), [&](auto& it)
  {
    it.second = false;
  });
}

void
SingleThreadedExecutor::publish_profile_data()
{
  if(tcl_profiler_)
  {
    auto release_start = tcl_timer_->get_release_start_time_point();
    auto execution_start = execution_start_time_.nanoseconds();

    auto execution_end = get_now().nanoseconds();
    auto release_end = execution_end;
    
    // std::cout<<"elapse : "<<(execution_end - execution_start) * 1.0e-6<<std::endl;
    tcl_profiler_->publish(
      tcl_timing_propagator_->get_timing_header(),
      release_start,
      release_end,
      execution_start,
      execution_end);
  }
}