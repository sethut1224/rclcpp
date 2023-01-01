#include "rclcpp/tcl_node_interfaces/node_release_timer.hpp"

using rclcpp::tcl_node_interfaces::NodeReleaseTimer;

NodeReleaseTimer::NodeReleaseTimer(
    int64_t rate,
    int64_t phase,
    int64_t ref_time
) : rate_(rate), 
    phase_(phase), 
    global_ref_time_(rclcpp::Time(ref_time)), 
    timerfd_(-1)
{
    auto period_ms = std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / static_cast<double>(rate_)));
    period_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(period_ms);

    local_ref_time_= global_ref_time_ + rclcpp::Duration(std::chrono::nanoseconds(phase_));
    release_start_time_ = local_ref_time_.nanoseconds();

    std::cout<<std::setprecision(15);

    init_timer();

    sleep();
}

void
NodeReleaseTimer::init_timer()
{   
    if(timerfd_ != -1)
        close(timerfd_);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME,&ts);

    auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec));
    
    int64_t timer_expired_time = local_ref_time_.nanoseconds();

    if(now >= local_ref_time_)
    {
        int64_t curr_offset = (now - global_ref_time_).nanoseconds() % period_ns_.count();
        int64_t offset_diff = phase_ - curr_offset + period_ns_.count();
        timer_expired_time = now.nanoseconds() + offset_diff;
    }

    std::cout<<"now : "<<now.nanoseconds()<<std::endl;
    std::cout<<"ref : "<<local_ref_time_.nanoseconds()<<std::endl;
    std::cout<<"exp : "<<timer_expired_time<<std::endl;

    struct itimerspec timeout;

    timeout.it_value.tv_sec = timer_expired_time / (int64_t)1e9;
    timeout.it_value.tv_nsec = timer_expired_time % (int64_t)1e9;
    timeout.it_interval.tv_sec = period_ns_.count() / 1000000000;
    timeout.it_interval.tv_nsec = period_ns_.count() % 1000000000;

    if ((timerfd_ = timerfd_create(CLOCK_REALTIME, 0)) <= 0)
    {
        std::cout << "timerfd_create failed!" << std::endl;
    }

    if (timerfd_settime(timerfd_, TFD_TIMER_ABSTIME, &timeout, NULL) != 0)
    {
        std::cout << "timerfd_settime fail" << std::endl;
    }
}

void
NodeReleaseTimer::sleep()
{   
    unsigned long long missed;
    struct timespec ts;

    if (read(timerfd_, &missed, sizeof(missed)) < 0)
        std::cout << "timer read error" << std::endl;

    clock_gettime(CLOCK_REALTIME,&ts);

    auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec));

    int64_t curr_offset = (now - local_ref_time_).nanoseconds() % period_ns_.count();
    release_start_time_ = now.nanoseconds() - curr_offset;

    // std::cout<<"predict : "<<release_start_time_<<std::endl;
    // std::cout<<"current : "<<now.nanoseconds()<<std::endl;
    // std::cout<<std::endl;
}

int64_t
NodeReleaseTimer::get_release_start_time_point() const
{
    return release_start_time_;
}

std::chrono::nanoseconds
NodeReleaseTimer::get_period_ns() const
{
    return this->period_ns_;
}