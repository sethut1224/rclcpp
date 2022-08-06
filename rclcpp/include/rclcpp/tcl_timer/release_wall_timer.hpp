#ifndef RCLCPP__TCL_TIMER__TCL_TIMER_HPP_
#define RCLCPP__TCL_TIMER__TCL_TIMER_HPP_

#include "rclcpp/clock.hpp"
#include <unistd.h>
#include <iostream>
#include <sys/timerfd.h>

using namespace std::chrono_literals;

namespace rclcpp
{
namespace tcl_timer
{

class ReleaseWallTimer
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(ReleaseWallTimer)

    explicit ReleaseWallTimer(int64_t rate, int64_t phase, int64_t ref_time);

    ~ReleaseWallTimer()=default;

    RCLCPP_PUBLIC
    void init_timer();

    RCLCPP_PUBLIC
    void sleep();

    RCLCPP_PUBLIC
    bool is_same_timer(int64_t rate, int64_t phase);

    RCLCPP_PUBLIC
    rclcpp::Time get_release_start_time();

    RCLCPP_PUBLIC
    rclcpp::Time get_global_ref_time();

    RCLCPP_PUBLIC
    rclcpp::Time get_local_ref_time();
    
    RCLCPP_PUBLIC
    bool timer_ready();

    RCLCPP_PUBLIC
    std::chrono::nanoseconds get_period();

private:
    int64_t rate_;
    int64_t phase_;

    rclcpp::Time global_ref_time_;
    rclcpp::Time local_ref_time_;
    std::chrono::nanoseconds period_;

    int timerfd_;
    int64_t timer_expired_time_;
    bool timer_ready_;
};

}
}

#endif

