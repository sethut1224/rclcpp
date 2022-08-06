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

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options), timeout_(std::chrono::nanoseconds(-1)) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable, timeout_)) {
      this->execute_any_executable(any_executable);
      if(this->check_condition())
      {
        this->reset_cond_map();
        this->sleep();
      }
    }
  }
}

void
SingleThreadedExecutor::sleep()
{
  if(timer_)
  {
    if(timer_->timer_ready())
      handle_profile_data();
    this->timer_->sleep();
  }
}


void SingleThreadedExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  // if(max_duration.count() != this->timer_->get_period().count())
  // {
  //   throw std::runtime_error("spin some period must be bigger to timer period [spin_some : "
  //   +std::to_string(max_duration.count()) +" timer : " + std::to_string(this->timer_->get_period().count()) + " ]");
  // }
  return this->spin_some_impl(max_duration, true);
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
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  bool work_available = false;
  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    AnyExecutable any_exec;
    if (!work_available) {
      wait_for_work(std::chrono::milliseconds::zero());
    }
    if (get_next_ready_executable(any_exec)) {
      execute_any_executable(any_exec);
      work_available = true;
    } else {
      if(!work_available)
      {
        if(!exhaustive)
          break;
        else
          if(check_condition())
            break;
      }
      work_available = false;
    }
  }
}

void
SingleThreadedExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  AnyExecutable any_exec;
  if (get_next_executable(any_exec, timeout)) {
    execute_any_executable(any_exec);
    if(this->check_condition())
    {
      this->reset_cond_map();
    }
  }
}

void
SingleThreadedExecutor::spin_once(std::chrono::nanoseconds timeout)
{
  start_time_ = get_now();
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  spin_once_impl(timeout);
}

void
SingleThreadedExecutor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  std::lock_guard<std::mutex> guard{mutex_};
  rclcpp::node_interfaces::global_for_each_callback_group(
    node_ptr.get(),
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
  add_node_timing_coordination(node_ptr->get_node_timing_coordination_interface());
}

bool
SingleThreadedExecutor::verify_use_tcl()
{
  bool val = node_communication_types_.begin()->second.first;

  auto ret = std::find_if(node_communication_types_.begin(), node_communication_types_.end(), [&](auto& type)
  {
    return type.second.first == !val;
  });

  if(ret != node_communication_types_.end())
    throw std::runtime_error("cannot mix tcl node and non-tcl node");
  
  return val;
}

bool
SingleThreadedExecutor::is_use_intra_process(bool start_parameter_services, bool use_intra_process)
{
  bool is_container_node = !start_parameter_services;

  if(!is_container_node)
  {
    node_intra_process_default_.push_back(use_intra_process);

    if(std::find(node_intra_process_default_.begin(), node_intra_process_default_.end(), !use_intra_process) == node_intra_process_default_.end())
    {
      return use_intra_process;
    }
    else
    {
      throw std::runtime_error("cannot mix intra-process communication and inter-process communication");
    }
  }

  return false;
}

void
SingleThreadedExecutor::add_node_timing_coordination(rclcpp::tcl_node_interfaces::NodeTimingCoordinationInterface::SharedPtr node_tcl_ptr)
{ 
  std::string node_name_str = std::string(node_tcl_ptr->get_node_base_interface()->get_name());

  use_intra_process_ = is_use_intra_process(
    node_tcl_ptr->get_node_options().start_parameter_services(),
    node_tcl_ptr->get_node_options().use_intra_process_comms());

  auto communication_type = std::pair<bool, bool>(
    node_tcl_ptr->use_tcl(),
    node_tcl_ptr->blocking_io());

  init_timing_coordination_dependency_map<NodeCommunicationTypes, std::string, std::pair<bool, bool>>(
    node_communication_types_,
    node_name_str,
    communication_type,
    "NodeCommunicationTypes");

  use_tcl_ = verify_use_tcl();

  if(use_tcl_)
  {
    init_timing_coordination_dependency_map<NodeBlockingTopics, std::string, std::vector<std::string>>(
      node_blocking_topics_, 
      node_name_str, 
      node_tcl_ptr->get_blocking_topics(),
      "NodeBlockingTopics");

    init_timing_coordination_dependency_map<NodeTimingProfiles, std::string, TimingProfileSharedPtr>(
      node_timing_profiles_,
      node_name_str,
      node_tcl_ptr->get_timing_profile(),
      "NodeTimingProfiles");

    init_timing_coordination_dependency_map<NodeTimingMessagePropagates, std::string, TimingMessagePropagateSharedPtr>(
      node_timing_message_propagates_,
      node_name_str,
      node_tcl_ptr->get_timing_message_propagate(),
      "NodeTimingMessagePropagate");
    
    init_timing_coordination_dependency_map<NodeTimerObjects, std::string, TimerObjects>(
      node_timer_objects_,
      node_name_str,
      TimerObjects(),
      "NodeTimerObjects");

    if(use_intra_process_)
      init_timing_coordination_dependency_map<NodeTopicWaitableObjects, std::string, TopicWaitableObjects>(
        node_topic_waitable_objects_,
        node_name_str,
        TopicWaitableObjects(),
        "NodeTopicWaitableObjects");
    else
      init_timing_coordination_dependency_map<NodeTopicSubscriptionObjects, std::string, TopicSubscriptionObjects>(
        node_topic_subscription_objects_,
        node_name_str,
        TopicSubscriptionObjects(),
        "NodeTopicSubscriptionObjects");

    if(node_communication_types_[node_name_str].second)
      init_timing_coordination_dependency_map<std::unordered_map<std::string, bool>, std::string, bool>(
        node_cond_map_,
        node_name_str,
        false,
        "ConditionMap");

    if(!timer_)
    {
      timer_ = node_tcl_ptr->get_timer();
    }
    else
    {
      bool same = timer_->is_same_timer(node_tcl_ptr->get_rate(), node_tcl_ptr->get_phase());
      if(!same)
        throw std::runtime_error("component nodes must be used same timer");
    }
    timeout_ = timer_->get_period();
  }
}

void
SingleThreadedExecutor::handle_profile_data()
{
  end_time_ = get_now();

  std::for_each(node_timing_profiles_.begin(), node_timing_profiles_.end(), [this](auto& t)
  {
    auto timing_header = node_timing_message_propagates_[t.first]->get_timing_header_ptr();
    t.second->publish(timing_header, this->timer_->get_release_start_time(), end_time_, start_time_, end_time_);
  });
}

void
SingleThreadedExecutor::update_cond_map(const char * node_name)
{
  std::string node_name_str = std::string(node_name);

  if(node_cond_map_.find(node_name_str) != node_cond_map_.end())
    node_cond_map_[node_name_str] = true;
}

void
SingleThreadedExecutor::reset_cond_map()
{
  std::for_each(node_cond_map_.begin(), node_cond_map_.end(), [&](auto& pair)
  {
    pair.second = false;
  });
}

bool
SingleThreadedExecutor::check_condition()
{
  auto ret = std::find_if(node_cond_map_.begin(), node_cond_map_.end(), [&](auto& pair)
  {
    return pair.second == false;
  });

  return ret == node_cond_map_.end();
}

void
SingleThreadedExecutor::handle_buffer(const char * node_name)
{
  start_time_ = get_now();
  handle_subscription_buffer(node_name);
  handle_waitable_buffer(node_name);
  handle_timer_buffer(node_name);
}

void
SingleThreadedExecutor::handle_subscription_buffer(const char * node_name)
{
  std::string node_name_str = std::string(node_name);

  std::for_each(node_topic_subscription_objects_[node_name_str].begin(), node_topic_subscription_objects_[node_name_str].end(), [&](auto& v)
  {
    auto timing_header = v.second->subscription->read_timing_header(v.second->message);
    node_timing_message_propagates_[node_name_str]->receive_timing_header(timing_header);

    v.second->subscription->handle_message(
      v.second->message,
      v.second->message_info
    );
  });

  node_topic_subscription_objects_[node_name_str].clear();
}

void
SingleThreadedExecutor::handle_waitable_buffer(const char * node_name)
{
  std::string node_name_str = std::string(node_name);

  std::for_each(node_topic_waitable_objects_[node_name_str].begin(), node_topic_waitable_objects_[node_name_str].end(), [&](auto& v)
  {
    auto timing_header = v.second->waitable->read_timing_header(v.second->data);
    node_timing_message_propagates_[node_name_str]->receive_timing_header(timing_header);

    v.second->waitable->execute(v.second->data);
  });
  
  node_topic_waitable_objects_[node_name_str].clear();
}

void
SingleThreadedExecutor::handle_timer_buffer(const char * node_name)
{
  std::string node_name_str = std::string(node_name);

  std::for_each(node_timer_objects_[node_name_str].begin(), node_timer_objects_[node_name_str].end(), [&](auto timer_object)
  {
    execute_timer(timer_object);
  });

  node_timer_objects_[node_name_str].clear();
}

bool
SingleThreadedExecutor::execute_any_executable(AnyExecutable & any_exec)
{
  std::string node_name_str = std::string(any_exec.node_base->get_name());
  bool cond = !node_communication_types_[node_name_str].second;

  if (!spinning.load()) {
    return false;
  }
  if (any_exec.subscription) {
    add_subscription_to_buffer(any_exec.subscription, any_exec.node_base->get_name());
    if(!use_intra_process_)
      cond = blocking_condition<NodeTopicSubscriptionObjects>(
        any_exec.node_base->get_name(), 
        node_topic_subscription_objects_);
    
  }
  if (any_exec.timer) {
    add_timer_to_buffer(any_exec.timer, any_exec.node_base->get_name());
    // execute_timer(any_exec.timer);
  }
  if (any_exec.service) {
    //add_service_to_buffer();
    execute_service(any_exec.service);
  }
  if (any_exec.client) {
    //add_client_to_buffer();
    execute_client(any_exec.client);
  }
  if (any_exec.waitable) {
    add_waitable_to_buffer(any_exec.waitable, any_exec.data, any_exec.node_base->get_name());
    if(use_intra_process_)
      cond = blocking_condition<NodeTopicWaitableObjects>(
        any_exec.node_base->get_name(), 
        node_topic_waitable_objects_);
  }

  if(cond)
  {
    handle_buffer(any_exec.node_base->get_name());
    update_cond_map(any_exec.node_base->get_name());
  }

  // Reset the callback_group, regardless of type
  any_exec.callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  rcl_ret_t ret = rcl_trigger_guard_condition(&interrupt_guard_condition_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to trigger guard condition from execute_any_executable");
  }

  return cond;
}

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

void
SingleThreadedExecutor::add_topic_subscription_object(std::string node_name, std::string topic_name, SubscriptionObjectSharedPtr object)
{
  for(size_t i = 0; i < node_topic_subscription_objects_[node_name].size(); ++i)
  {
    if(node_topic_subscription_objects_[node_name][i].first == topic_name)
    {
      node_topic_subscription_objects_[node_name][i].second = object;
      return;
    }
  }
  node_topic_subscription_objects_[node_name].emplace_back(topic_name, object);
}

void
SingleThreadedExecutor::add_service_object(std::string node_name, ServiceObjectSharedPtr service)
{
  (void)node_name;
  (void)service;
}

void
SingleThreadedExecutor::add_client_object(std::string node_name, ClientObjectSharedPtr client)
{
  (void)node_name;
  (void)client;
}

void
SingleThreadedExecutor::add_waitable_object(std::string node_name, std::string topic_name, WaitableObjectSharedPtr object)
{
  for(size_t i = 0; i < node_topic_waitable_objects_[node_name].size(); ++i)
  {
    if(node_topic_waitable_objects_[node_name][i].first == topic_name)
    {
      node_topic_waitable_objects_[node_name][i].second = object;
      return;
    }
  }
  node_topic_waitable_objects_[node_name].emplace_back(topic_name, object);
}

void
SingleThreadedExecutor::add_subscription_to_buffer(rclcpp::SubscriptionBase::SharedPtr subscription, const char * node_name)
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
        auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
        subscription->handle_message(void_serialized_msg, message_info);
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
    std::string node_name_str = std::string(node_name);
    std::string topic_name_str = std::string(subscription->get_topic_name());

    if(topic_name_str == "/parameter_events")
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
        SubscriptionObjectSharedPtr unhandled_subscription_object = std::make_shared<subscription_object_t>();
        unhandled_subscription_object->subscription = subscription;
        unhandled_subscription_object->message = message;
        unhandled_subscription_object->message_info = message_info;
        
        add_topic_subscription_object(node_name_str, topic_name_str, unhandled_subscription_object);
      });
    }
    subscription->return_message(message);
  }
}

void
SingleThreadedExecutor::add_timer_to_buffer(rclcpp::TimerBase::SharedPtr timer, const char * node_name)
{
  std::string node_name_str = std::string(node_name);
  node_timer_objects_[node_name_str].push_back(timer);
}

void
SingleThreadedExecutor::add_service_to_buffer(rclcpp::ServiceBase::SharedPtr service, const char * node_name)
{
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  std::string node_name_str = std::string(node_name);

  take_and_do_error_handling(
    "taking a service server request from service",
    service->get_service_name(),
    [&]() {return service->take_type_erased_request(request.get(), *request_header);},
    [&]() 
    {
      ServiceObjectSharedPtr unhandled_service_object = std::make_shared<service_object_t>();
      unhandled_service_object->service = service;
      unhandled_service_object->request_header = request_header;
      unhandled_service_object->request = request;

      add_service_object(node_name_str, unhandled_service_object);
    });
}

void 
SingleThreadedExecutor::add_client_to_buffer(rclcpp::ClientBase::SharedPtr client, const char * node_name)
{
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  std::string node_name_str = std::string(node_name);

  take_and_do_error_handling(
    "taking a service client response from service",
    client->get_service_name(),
    [&]() {return client->take_type_erased_response(response.get(), *request_header);},
    [&]() 
    {
      ClientObjectSharedPtr unhandled_client_object = std::make_shared<client_object_t>();
      unhandled_client_object->client = client;
      unhandled_client_object->request_header = request_header;
      unhandled_client_object->response = response;

      add_client_object(node_name_str, unhandled_client_object);
    });
}

void
SingleThreadedExecutor::add_waitable_to_buffer(rclcpp::Waitable::SharedPtr waitable, std::shared_ptr<void> data, const char * node_name)
{
  auto waitable_cast = std::dynamic_pointer_cast<rclcpp::experimental::SubscriptionIntraProcessBase>(waitable);
  if(waitable_cast)
  {
    const char * topic_name = waitable_cast->get_topic_name();
    std::string node_name_str = std::string(node_name);
    std::string topic_name_str = std::string(topic_name);

    if(topic_name_str == "/parameter_events")
    {
      waitable_cast->execute(data);
    }
    else
    {
      WaitableObjectSharedPtr unhandled_waitable_object = std::make_shared<waitable_object_t>();
      unhandled_waitable_object->waitable = waitable;
      unhandled_waitable_object->data = data;

      add_waitable_object(node_name_str, topic_name_str, unhandled_waitable_object);
    }
  }
}
