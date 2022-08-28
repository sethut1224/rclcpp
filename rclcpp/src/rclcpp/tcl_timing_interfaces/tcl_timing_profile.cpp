#include "rclcpp/tcl_timing_interfaces/tcl_timing_profile.hpp"

using rclcpp::tcl_timing_interfaces::TimingProfile;

TimingProfile::TimingProfile(std::string node_name)
:node_name_(node_name)
{
}

void TimingProfile::create_publisher(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
    std::string topic_name)
{
    auto pub = rclcpp::detail::create_publisher<tcl_msgs::msg::Profile>(
        node_parameters,
        node_topics,
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(1)));

    profile_publisher_ = pub;
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

    profile_publisher_->publish(msg);
}