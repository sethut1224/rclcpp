// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/tcl_node_interfaces/node_timing_interface.hpp"

namespace rclcpp
{
namespace executors
{
//
// Timing Coordination Library
//
typedef struct subscription_object_t
{
  rclcpp::SubscriptionBase::SharedPtr subscription;
  std::shared_ptr<void> message;
  rclcpp::MessageInfo message_info;
  
  subscription_object_t(
    rclcpp::SubscriptionBase::SharedPtr & _subscription,
    std::shared_ptr<void> _message,
    rclcpp::MessageInfo _message_info
  ) : subscription(_subscription), message(_message), message_info(_message_info) {}

} subscription_object_t;

/// Single-threaded executor implementation.
/**
 * This is the default executor created by rclcpp::spin.
 */
class SingleThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  explicit SingleThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~SingleThreadedExecutor();

  /// Single-threaded implementation of spin.
  /**
   * This function will block until work comes in, execute it, and then repeat
   * the process until canceled.
   * It may be interrupt by a call to rclcpp::Executor::cancel() or by ctrl-c
   * if the associated context is configured to shutdown on SIGINT.
   * \throws std::runtime_error when spin() called while already spinning
   */

  //
  //Timing Coordination Library
  //

  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  RCLCPP_PUBLIC
  void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  sleep();

  RCLCPP_PUBLIC
  void
  add_node_timing_interface(rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing) override;

  RCLCPP_PUBLIC
  void
  spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

  RCLCPP_PUBLIC
  void
  execute_any_executable_for_tcl(AnyExecutable & any_exec);

  RCLCPP_PUBLIC
  void
  add_subscription_to_buffer(rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  void
  add_timer_to_buffer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  void
  handle_buffer();

  RCLCPP_PUBLIC
  void
  handle_subscription_buffer();

  RCLCPP_PUBLIC
  void
  handle_timer_buffer();

  RCLCPP_PUBLIC
  bool
  check_blocking_condition();

  RCLCPP_PUBLIC
  void
  update_blocking_condition_map(const char * topic_name);

  RCLCPP_PUBLIC
  void
  reset_blocking_condition_map();

  RCLCPP_PUBLIC
  void
  publish_profile_data();

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)

  rclcpp::tcl_node_interfaces::NodeReleaseTimerInterface::SharedPtr tcl_timer_{nullptr};
  rclcpp::tcl_node_interfaces::NodeProfileInterface::SharedPtr tcl_profiler_{nullptr};
  rclcpp::tcl_node_interfaces::NodeTimingPropagateInterface::SharedPtr tcl_timing_propagator_{nullptr};

  std::unordered_map<std::string, std::shared_ptr<subscription_object_t>> topic_subscription_objects_;
  std::vector<std::string> topic_subscription_order_;
  std::unordered_map<std::string, bool> blocking_condition_map_;

  std::vector<rclcpp::TimerBase::SharedPtr> timer_objects_;

  bool use_tcl_{false};
  bool use_blocking_io_{false};
  bool is_condition_satisfied_{false};
  bool profile_enabled_{false};

  rclcpp::Time execution_start_time_;

  std::function<rclcpp::Time()> get_now = [&]()
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec), RCL_STEADY_TIME);
  };
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
