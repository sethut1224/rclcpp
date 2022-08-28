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
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/tcl_timer/release_wall_timer.hpp"

namespace rclcpp
{
namespace executors
{

/// Single-threaded executor implementation.
/**
 * This is the default executor created by rclcpp::spin.
 */

typedef struct subscription_object_t
{
  rclcpp::SubscriptionBase::SharedPtr subscription;
  std::shared_ptr<void> message;
  rclcpp::MessageInfo message_info;
} subscription_object_t;

typedef struct service_object_t
{
  rclcpp::ServiceBase::SharedPtr service;
  std::shared_ptr<rmw_request_id_t> request_header;
  std::shared_ptr<void> request;
} service_object_t;

typedef struct client_object_t
{
  rclcpp::ClientBase::SharedPtr client;
  std::shared_ptr<rmw_request_id_t> request_header;
  std::shared_ptr<void> response;
} client_object_t;

typedef struct waitable_object_t
{
  rclcpp::Waitable::SharedPtr waitable;
  std::shared_ptr<void> data;
} waitable_object_t;


typedef typename std::shared_ptr<subscription_object_t> SubscriptionObjectSharedPtr;
typedef typename std::pair<std::string, SubscriptionObjectSharedPtr> TopicSubscriptionObject;
typedef typename std::vector<TopicSubscriptionObject> TopicSubscriptionObjects;
typedef typename std::unordered_map<std::string, TopicSubscriptionObjects> NodeTopicSubscriptionObjects;

typedef typename std::shared_ptr<service_object_t> ServiceObjectSharedPtr;
typedef typename std::vector<ServiceObjectSharedPtr> ServiceObjects;
typedef typename std::unordered_map<std::string, ServiceObjects> NodeServiceObjects;

typedef typename std::shared_ptr<client_object_t> ClientObjectSharedPtr;
typedef typename std::vector<ClientObjectSharedPtr> ClientObjects;
typedef typename std::unordered_map<std::string, ClientObjects> NodeClientObjects;

typedef typename std::shared_ptr<waitable_object_t> WaitableObjectSharedPtr;
typedef typename std::pair<std::string, WaitableObjectSharedPtr> TopicWaitableObject;
typedef typename std::vector<TopicWaitableObject> TopicWaitableObjects;
typedef typename std::unordered_map<std::string, TopicWaitableObjects> NodeTopicWaitableObjects;

typedef typename std::vector<rclcpp::TimerBase::SharedPtr> TimerObjects;
typedef typename std::unordered_map<std::string, TimerObjects> NodeTimerObjects;

typedef typename rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr TimingProfileSharedPtr;
typedef typename std::unordered_map<std::string, TimingProfileSharedPtr> NodeTimingProfiles;

typedef typename rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr TimingMessagePropagateSharedPtr;
typedef typename std::unordered_map<std::string, TimingMessagePropagateSharedPtr> NodeTimingMessagePropagates;

typedef typename std::unordered_map<std::string, std::vector<std::string>> NodeTopics;
typedef typename std::unordered_map<std::string, std::pair<bool, bool>> NodeCommunicationTypes;

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
  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  virtual void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node_timing_coordination(rclcpp::tcl_node_interfaces::NodeTimingCoordinationInterface::SharedPtr node_tcl_ptr) override;

  RCLCPP_PUBLIC
  void
  add_subscription_to_buffer(rclcpp::SubscriptionBase::SharedPtr subscription, const char * node_name);

  RCLCPP_PUBLIC
  void
  add_timer_to_buffer(rclcpp::TimerBase::SharedPtr timer, const char * node_name);

  RCLCPP_PUBLIC
  void
  add_service_to_buffer(rclcpp::ServiceBase::SharedPtr service, const char * node_name);

  RCLCPP_PUBLIC
  void
  add_client_to_buffer(rclcpp::ClientBase::SharedPtr client, const char * node_name);

  RCLCPP_PUBLIC
  void
  add_waitable_to_buffer(rclcpp::Waitable::SharedPtr waitable, std::shared_ptr<void> data, const char * node_name);

  RCLCPP_PUBLIC
  void
  add_topic_subscription_object(std::string node_name, std::string topic_name, SubscriptionObjectSharedPtr object);

  RCLCPP_PUBLIC
  void
  add_service_object(std::string node_name, ServiceObjectSharedPtr service);

  RCLCPP_PUBLIC
  void
  add_client_object(std::string node_name, ClientObjectSharedPtr client);

  RCLCPP_PUBLIC
  void
  add_waitable_object(std::string node_name, std::string topic_name, WaitableObjectSharedPtr waitable);

  RCLCPP_PUBLIC
  bool 
  execute_any_executable(AnyExecutable & any_exec);

  template <typename Map, typename Key, typename Value>
  void 
  init_timing_coordination_dependency_map(Map& map, Key k, Value v, std::string type)
  {
    if(map.find(k) == map.end())
      map.insert(
        std::pair<Key, Value>(k, v));
    else
      throw std::runtime_error("Faile to set " + type + "[Duplicate Nodes]");
  }

  template <typename ObjectsMap>
  bool
  blocking_condition(const char * node_name, ObjectsMap& map)
  {
    bool ret = true;
    std::string node_name_str = std::string(node_name);

    if(!node_communication_types_[node_name_str].second)
      return ret;

    std::for_each(node_blocking_topics_[node_name_str].begin(), node_blocking_topics_[node_name_str].end(), [&](auto& topic)
    {
      auto it = std::find_if( map[node_name_str].begin(),  map[node_name_str].end(), [&](auto& v)
      {
        return v.first == topic;
      });

      if(it == map[node_name_str].end())
        ret = false;
    });

    return ret;
  }

  RCLCPP_PUBLIC
  void
  handle_buffer(const char * node_name);

  RCLCPP_PUBLIC
  void
  handle_subscription_buffer(const char * node_name);

  RCLCPP_PUBLIC
  void
  handle_waitable_buffer(const char * node_name);

  RCLCPP_PUBLIC
  void
  handle_timer_buffer(const char * node_name);

  RCLCPP_PUBLIC
  void update_cond_map(const char * node_name);

  RCLCPP_PUBLIC
  void reset_cond_map();

  RCLCPP_PUBLIC
  bool check_condition();

  RCLCPP_PUBLIC
  bool
  verify_use_tcl();

  RCLCPP_PUBLIC
  bool
  is_use_intra_process(bool is_container_node, bool use_intra_process);

  RCLCPP_PUBLIC
  void
  sleep();

  RCLCPP_PUBLIC
  void
  handle_profile_data();
  
  RCLCPP_PUBLIC
  void 
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  RCLCPP_PUBLIC
  void
  spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

private:
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)

  bool use_intra_process_ {false};
  bool use_tcl_{false};
  bool need_sleep_{false};

  std::unordered_map<std::string, bool> node_cond_map_;
  std::vector<bool> node_intra_process_default_;

  NodeCommunicationTypes node_communication_types_;
  NodeTopicSubscriptionObjects node_topic_subscription_objects_;
  NodeServiceObjects node_service_objects_;
  NodeClientObjects node_client_objects_;
  NodeTopicWaitableObjects node_topic_waitable_objects_;
  NodeTimerObjects node_timer_objects_;
  NodeTopics node_blocking_topics_;
  NodeTimingProfiles node_timing_profiles_;
  NodeTimingMessagePropagates node_timing_message_propagates_;
  
  rclcpp::tcl_timer::ReleaseWallTimer::SharedPtr timer_;
  rclcpp::Time start_time_;
  rclcpp::Time end_time_;
  std::chrono::nanoseconds timeout_;
  
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
