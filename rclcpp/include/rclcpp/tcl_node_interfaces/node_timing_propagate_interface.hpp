#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_PROPAGATE_INTERFACE_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_PROPAGATE_INTERFACE_HPP_

#include <unistd.h>
#include <time.h>
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tcl_std_msgs/msg/timing_header.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/create_subscription.hpp"


namespace rclcpp
{
namespace tcl_node_interfaces
{
class NodeTimingPropagateInterface
{
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimingPropagateInterface)

        RCLCPP_PUBLIC
        virtual
        ~NodeTimingPropagateInterface() = default;

        RCLCPP_PUBLIC
        virtual 
        tcl_std_msgs::msg::TimingHeader::SharedPtr get_timing_header() const = 0;

        RCLCPP_PUBLIC
        virtual void receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg) = 0;

        RCLCPP_PUBLIC
        virtual void receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg) = 0;

        RCLCPP_PUBLIC
        virtual void receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg) = 0;

        RCLCPP_PUBLIC
        virtual void create_timing_header() = 0;

        RCLCPP_PUBLIC
        virtual void publish_timing_message() = 0;
};
}
}
#endif