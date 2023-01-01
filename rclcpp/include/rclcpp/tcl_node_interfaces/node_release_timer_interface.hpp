#ifndef RCLCPP_TCL_NODE_INTERFACES__NODE_RELEASE_TIMER_INTERFACE_HPP_
#define RCLCPP_TCL_NODE_INTERFACES__NODE_RELEASE_TIMER_INTERFACE_HPP_

#include "rclcpp/clock.hpp"
#include <unistd.h>
#include <iostream>
#include <sys/timerfd.h>

using namespace std::chrono_literals;

namespace rclcpp
{
namespace tcl_node_interfaces
{

class NodeReleaseTimerInterface
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(NodeReleaseTimerInterface)

    RCLCPP_PUBLIC
    virtual
    ~NodeReleaseTimerInterface() = default;

    RCLCPP_PUBLIC
    virtual void init_timer() = 0;

    RCLCPP_PUBLIC
    virtual void sleep() = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_release_start_time_point() const = 0;

    RCLCPP_PUBLIC
    virtual std::chrono::nanoseconds get_period_ns() const = 0;
};

}
}

#endif

