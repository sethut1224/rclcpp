#ifndef RCLCPP_TCL_NODE_INTERFACES__NODE_RELEASE_TIMER_HPP_
#define RCLCPP_TCL_NODE_INTERFACES__NODE_RELEASE_TIMER_HPP_

#include "rclcpp/tcl_node_interfaces/node_release_timer_interface.hpp"

using namespace std::chrono_literals;

namespace rclcpp
{
namespace tcl_node_interfaces
{

class NodeReleaseTimer : public NodeReleaseTimerInterface
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(NodeReleaseTimer)

    RCLCPP_PUBLIC
    explicit NodeReleaseTimer(
        int64_t rate,
        int64_t phase,
        int64_t ref_time);

    RCLCPP_PUBLIC
    virtual
    ~NodeReleaseTimer() = default;

    RCLCPP_PUBLIC
    void init_timer() override;

    RCLCPP_PUBLIC
    void sleep() override;

    RCLCPP_PUBLIC
    int64_t get_release_start_time_point() const override;

    RCLCPP_PUBLIC
    std::chrono::nanoseconds get_period_ns() const override;

private:
    int64_t rate_;
    int64_t phase_;

    rclcpp::Time global_ref_time_;
    rclcpp::Time local_ref_time_;

    int64_t release_start_time_;

    std::chrono::nanoseconds period_ns_;

    int timerfd_;
};

}
}

#endif

