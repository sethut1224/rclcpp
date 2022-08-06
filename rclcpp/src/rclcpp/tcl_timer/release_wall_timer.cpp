#include "rclcpp/tcl_timer/release_wall_timer.hpp"

using rclcpp::tcl_timer::ReleaseWallTimer;

ReleaseWallTimer::ReleaseWallTimer(
    int64_t rate,
    int64_t phase,
    int64_t ref_time
) : rate_(rate), phase_(phase), global_ref_time_(rclcpp::Time(ref_time, RCL_STEADY_TIME)), timerfd_(-1), timer_ready_(false)
{
    auto period_milli = std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / static_cast<double>(rate_)));
    period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(period_milli);

    local_ref_time_= global_ref_time_ + rclcpp::Duration(std::chrono::nanoseconds(phase_));
    std::cout<<std::setprecision(15);
    init_timer();
}

bool
ReleaseWallTimer::is_same_timer(int64_t rate, int64_t phase)
{
    if(this->rate_ == rate && this->phase_ == phase)
        return true;

    return false;
}

void
ReleaseWallTimer::init_timer()
{   
    if(timerfd_ != -1)
        close(timerfd_);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);

    auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec), RCL_STEADY_TIME);
    
    int64_t curr_offset = period_.count() - (global_ref_time_ - now).nanoseconds() % period_.count();
    int64_t release_offset = phase_;
    int64_t diff = 0;

    diff = (period_.count() - curr_offset) + release_offset + period_.count() * 5;

    timer_expired_time_ = now.nanoseconds() + diff;

    struct itimerspec timeout;
    int64_t sec = timer_expired_time_ / (int64_t)1e9;
    int64_t nsec = timer_expired_time_ % (int64_t)1e9;

    timeout.it_value.tv_sec = sec;
    timeout.it_value.tv_nsec = nsec;
    timeout.it_interval.tv_sec = period_.count() / 1000000000;
    timeout.it_interval.tv_nsec = period_.count() % 1000000000;

    if ((timerfd_ = timerfd_create(CLOCK_MONOTONIC, 0)) <= 0)
    {
        std::cout << "timerfd_create failed!" << std::endl;
    }

    if (timerfd_settime(timerfd_, TFD_TIMER_ABSTIME, &timeout, NULL) != 0)
    {
        std::cout << "timerfd_settime fail" << std::endl;
    }
}

void
ReleaseWallTimer::sleep()
{   
    unsigned long long missed;
    struct timespec ts;

    if (read(timerfd_, &missed, sizeof(missed)) < 0)
        std::cout << "timer read error" << std::endl;

    clock_gettime(CLOCK_MONOTONIC,&ts);

    std::cout<<"[current | intended] : "<<rclcpp::Time(
        static_cast<int32_t>(ts.tv_sec), 
        static_cast<uint32_t>(ts.tv_nsec), 
        RCL_STEADY_TIME).seconds()<< " | " <<rclcpp::Time(timer_expired_time_, RCL_STEADY_TIME).seconds()<<std::endl;

    timer_expired_time_ = timer_expired_time_ + period_.count();
}

rclcpp::Time
ReleaseWallTimer::get_release_start_time()
{
    return rclcpp::Time(timer_expired_time_ - period_.count(), RCL_STEADY_TIME);
}

rclcpp::Time
ReleaseWallTimer::get_global_ref_time()
{
    return this->global_ref_time_;
}

rclcpp::Time
ReleaseWallTimer::get_local_ref_time()
{
    return this->local_ref_time_;
}

std::chrono::nanoseconds
ReleaseWallTimer::get_period()
{
    return this->period_;
}

bool
ReleaseWallTimer::timer_ready()
{   
    if(timer_ready_)
        return true;
    
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec), RCL_STEADY_TIME);

    if(now > local_ref_time_)
        timer_ready_ = true; 

    return timer_ready_;
}
