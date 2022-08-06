#include "rclcpp/tcl_timing_interfaces/tcl_timing_profile.hpp"

using rclcpp::tcl_timing_interfaces::TimingProfile;

TimingProfile::TimingProfile(std::shared_ptr<Publisher<Profile>> pub)
:timing_profile_publisher_(pub)
{
}

void TimingProfile::publish(
    TimingCoordinationHeader::SharedPtr timing_header,
    rclcpp::Time release_start,
    rclcpp::Time release_end,
    rclcpp::Time execution_start,
    rclcpp::Time execution_end)
{
    Profile msg = Profile();
    msg.timing_header = *timing_header.get();
    msg.release_time.start = release_start.nanoseconds();
    msg.release_time.end = release_end.nanoseconds();
    msg.execution_time.start = execution_start.nanoseconds();
    msg.execution_time.end = execution_end.nanoseconds();

    timing_profile_publisher_->publish(msg);
}

void 
TimingProfile::set_profile_publisher(std::shared_ptr<Publisher<Profile>> publisher)
{
    timing_profile_publisher_ = publisher;
}