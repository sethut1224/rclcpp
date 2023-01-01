#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_PROPAGATE_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_PROPAGATE_HPP_

#include "rclcpp/tcl_node_interfaces/node_timing_propagate_interface.hpp"

namespace rclcpp
{
namespace tcl_node_interfaces
{
    class NodeTimingPropagate : public NodeTimingPropagateInterface
    {
        public:
            RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimingPropagate)

            RCLCPP_PUBLIC
            explicit NodeTimingPropagate(
            const char * node_name,
            std::vector<std::string>& sub_timing_topics,
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics);

            RCLCPP_PUBLIC
            virtual
            ~NodeTimingPropagate() = default;

            RCLCPP_PUBLIC
            tcl_std_msgs::msg::TimingHeader::SharedPtr
            get_timing_header() const override;

            RCLCPP_PUBLIC
            void publish_timing_message() override;

            RCLCPP_PUBLIC
            void create_timing_header() override;

            RCLCPP_PUBLIC
            void receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg) override;

            RCLCPP_PUBLIC
            void receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg) override;

            RCLCPP_PUBLIC
            void receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg) override;

            RCLCPP_PUBLIC
            void create_subscription(
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
                std::string topic_name);
            
            RCLCPP_PUBLIC
            void create_publisher(
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
                std::string topic_name);

        private:
            std::string node_name_;

            std::shared_ptr<Publisher<tcl_std_msgs::msg::TimingHeader>> timing_publisher_;
            std::unordered_map<std::string, std::shared_ptr<Subscription<tcl_std_msgs::msg::TimingHeader>>> timing_subscriber_map_;

            tcl_std_msgs::msg::TimingHeader::SharedPtr timing_header_msg_;
    };
}
}
#endif