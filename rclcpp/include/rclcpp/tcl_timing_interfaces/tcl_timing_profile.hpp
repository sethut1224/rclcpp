#ifndef RCLCPP__TCL_TIMING_TIMING_INTERFACES__TCL_TIMING_PROFILE_HPP_
#define RCLCPP__TCL_TIMING_TIMING_INTERFACES__TCL_TIMING_PROFILE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tcl_msgs/msg/profile.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"


using tcl_msgs::msg::Profile;
using tcl_msgs::msg::TimingCoordinationHeader;

namespace rclcpp
{
namespace tcl_timing_interfaces
{   
    class TimingProfile
    {
    private:
        std::shared_ptr<Publisher<Profile>> profile_publisher_;
        std::string node_name_;

    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(TimingProfile)
        TimingProfile() = default;
        ~TimingProfile() = default;
        TimingProfile(std::string node_name);

        RCLCPP_PUBLIC
        void publish(
            TimingCoordinationHeader::SharedPtr timing_header,
            rclcpp::Time release_start, 
            rclcpp::Time release_end, 
            rclcpp::Time execution_start, 
            rclcpp::Time execution_end);
        
        RCLCPP_PUBLIC
        void create_publisher(
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
            std::string topic_name);
    };

}
}
#endif