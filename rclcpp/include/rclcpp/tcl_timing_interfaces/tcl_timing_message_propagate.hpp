#ifndef RCLCPP__TCL_TIMING_INTERFACES__TCL_TIMING_MESSAGE_PROPAGATE_HPP_
#define RCLCPP__TCL_TIMING_INTERFACES__TCL_TIMING_MESSAGE_PROPAGATE_HPP_

#include <unistd.h>
#include <time.h>
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "tcl_msgs/msg/timing_coordination_header.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/create_subscription.hpp"


using tcl_msgs::msg::TimingCoordinationHeader;

namespace rclcpp
{
namespace tcl_timing_interfaces
{
    typedef typename std::shared_ptr<Subscription<tcl_msgs::msg::TimingCoordinationHeader>> TimingSubscriber;
    typedef typename std::shared_ptr<Publisher<tcl_msgs::msg::TimingCoordinationHeader>> TimingPublisher;

    class TimingMessagePropagate
    {
        private:
            std::string task_name_;

            TimingPublisher timing_publisher_;
            std::unordered_map<std::string, TimingSubscriber> timing_subscriber_map_;

            TimingCoordinationHeader::SharedPtr timing_header_msg_;
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
            void timing_message_callback(const TimingCoordinationHeader::SharedPtr msg);

            RCLCPP_PUBLIC
            TimingCoordinationHeader create_timing_header();

            RCLCPP_PUBLIC
            TimingCoordinationHeader::SharedPtr get_timing_header_ptr() const;

            RCLCPP_PUBLIC
            void publish_timing_message();

            RCLCPP_PUBLIC
            void create_subscription(
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
                std::string topic_name);
            
            RCLCPP_PUBLIC
            void create_publisher(
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
                std::string topic_name);
    };
}
}
#endif