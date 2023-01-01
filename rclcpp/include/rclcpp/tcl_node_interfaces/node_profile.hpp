#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_PROFILE_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_PROFILE_HPP_

#include "rclcpp/tcl_node_interfaces/node_profile_interface.hpp"

namespace rclcpp
{
namespace tcl_node_interfaces
{   
    class NodeProfile : public NodeProfileInterface
    {
    public:
        RCLCPP_SMART_PTR_ALIASES_ONLY(NodeProfile)

        RCLCPP_PUBLIC
        explicit NodeProfile(
            const char * node_name, 
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics);

        RCLCPP_PUBLIC
        virtual
        ~NodeProfile() = default;

        RCLCPP_PUBLIC
        void 
        publish(
            tcl_std_msgs::msg::TimingHeader::SharedPtr timing_header,
            int64_t release_start, 
            int64_t release_end, 
            int64_t execution_start, 
            int64_t execution_end) override;
        
        // RCLCPP_PUBLIC
        // void 
        // create_publisher(
        //     rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
        //     rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
        //     std::string topic_name) override;

    private:
        std::shared_ptr<Publisher<tcl_std_msgs::msg::ProfileData>> profile_data_publisher_;
        std::string node_name_;
    };

}
}
#endif