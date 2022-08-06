#ifndef RCLCPP__TCL_TIMING_INTERFACES__TCL_TIMING_MESSAGE_PROPAGATE_HPP_
#define RCLCPP__TCL_TIMING_INTERFACES__TCL_TIMING_MESSAGE_PROPAGATE_HPP_

#include <unistd.h>
#include <time.h>
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tcl_msgs/msg/timing_coordination_header.hpp"
#include "std_msgs/msg/header.hpp"
// #include "rclcpp/tcl_timing_header_traits/tcl_timing_header_traits.hpp"

using tcl_msgs::msg::TimingCoordinationHeader;
namespace rclcpp
{
namespace tcl_timing_interfaces
{
    class TimingMessagePropagate
    {
        private:
            TimingCoordinationHeader::SharedPtr timing_header_msg_;
            std::string task_name_;
        public:
            RCLCPP_SMART_PTR_ALIASES_ONLY(TimingMessagePropagate)

            RCLCPP_PUBLIC
            TimingMessagePropagate() = default;

            RCLCPP_PUBLIC
            TimingMessagePropagate(std::string task_name);

            RCLCPP_PUBLIC
            TimingMessagePropagate(const char * task_name);

            RCLCPP_PUBLIC
            ~TimingMessagePropagate() = default;

            RCLCPP_PUBLIC
            void receive_timing_header(TimingCoordinationHeader& msg);

            RCLCPP_PUBLIC 
            TimingCoordinationHeader propagate_timing_header();

            RCLCPP_PUBLIC
            TimingCoordinationHeader create_timing_header();

            RCLCPP_PUBLIC
            TimingCoordinationHeader::SharedPtr get_timing_header_ptr() const;
    };
}
}
#endif