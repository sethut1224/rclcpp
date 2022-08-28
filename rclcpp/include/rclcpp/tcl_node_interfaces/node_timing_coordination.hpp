#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_COORDINATION_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_COORDINATION_HPP_

#include "rclcpp/tcl_node_interfaces/node_timing_coordination_interface.hpp"

namespace rclcpp
{
namespace tcl_node_interfaces
{
    
class NodeTimingCoordination : public NodeTimingCoordinationInterface
{
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimingCoordination)

    RCLCPP_PUBLIC
    NodeTimingCoordination(
        node_interfaces::NodeBaseInterface::SharedPtr node_base,
        node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
        node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
        const rclcpp::NodeOptions options
    );

    RCLCPP_PUBLIC
    virtual
    ~NodeTimingCoordination() = default;

    RCLCPP_PUBLIC
    bool sched_set_cpu_affinity(int64_t cpu_affinity) override;

    RCLCPP_PUBLIC
    bool sched_set_scheduler(int64_t priority, int scheduling_algorithm = SCHED_FIFO) override;

    RCLCPP_PUBLIC
    std::vector<std::string> get_blocking_topics() override;

    RCLCPP_PUBLIC
    std::vector<std::string> get_sub_timing_observation_topics() override;

    RCLCPP_PUBLIC
    std::vector<std::string> get_pub_timing_observation_topics() override;

    RCLCPP_PUBLIC
    int64_t get_cpu_affinity() override;

    RCLCPP_PUBLIC
    int64_t get_rate() override;

    RCLCPP_PUBLIC
    int64_t get_phase() override;

    RCLCPP_PUBLIC
    int64_t get_priority() override;
    
    RCLCPP_PUBLIC
    int64_t get_message_communication_type() override;

    RCLCPP_PUBLIC
    bool get_enable_profile() override;

    RCLCPP_PUBLIC
    bool use_tcl() override;

    RCLCPP_PUBLIC
    bool blocking_io() override;
    
    RCLCPP_PUBLIC
    void print_node_sched_info() override;
    
    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override;

    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() override;

    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() override;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr get_timing_profile() override;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr get_timing_message_propagate() override;

    RCLCPP_PUBLIC
    virtual rclcpp::Time get_local_ref_time() override;

    RCLCPP_PUBLIC
    virtual rclcpp::Time get_global_ref_time() override;

    RCLCPP_PUBLIC
    const rclcpp::NodeOptions& get_node_options() const override;

    RCLCPP_PUBLIC
    rclcpp::tcl_timer::ReleaseWallTimer::SharedPtr get_timer() override;
    
    RCLCPP_PUBLIC
    tcl_msgs::msg::TimingCoordinationHeader::SharedPtr get_timing_header_ptr() const override;

    RCLCPP_PUBLIC
    tcl_msgs::msg::TimingCoordinationHeader create_timing_header() override;
    
    RCLCPP_PUBLIC
    std::chrono::nanoseconds get_period_ns() const override;

    RCLCPP_PUBLIC
    void propagate_timing_message() override;

    void create_profile_publisher(const std::string& node_name)
    {
        const std::string suffix = std::string("/tcl_profile");
        const std::string topic_name = node_name + suffix;

        timing_profile_ = std::make_shared<tcl_timing_interfaces::TimingProfile>(node_name);
        timing_profile_->create_publisher(node_parameters_, node_topics_, topic_name);
    }

    void create_timing_publisher(const std::string& node_name)
    {
        const std::string suffix = std::string("/tcl_timing");
        const std::string topic_name = node_name + suffix;

        timing_message_propagate_ = std::make_shared<tcl_timing_interfaces::TimingMessagePropagate>(node_name);
        timing_message_propagate_->create_publisher(node_parameters_, node_topics_, topic_name);
    }

    RCLCPP_PUBLIC
    void create_timing_subscriber(const std::string& topic)
    {   
        timing_message_propagate_->create_subscription(node_parameters_, node_topics_, topic);
    }


private:
    node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
    const rclcpp::NodeOptions options_;

    rclcpp::tcl_timer::ReleaseWallTimer::SharedPtr timer_;
    
    int64_t cpu_affinity_;
    int64_t rate_;
    int64_t phase_;
    int64_t priority_;
    int64_t message_communication_type_;

    bool use_tcl_;

    std::vector<std::string> blocking_topics_;
    std::vector<std::string> sub_timing_observation_topics_;
    std::vector<std::string> pub_timing_observation_topics_;

    int64_t global_ref_time_point_;
    
    bool enable_profile_;
    int timerfd_;
    bool timer_ready_ {false};

    rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr timing_profile_;
    rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr timing_message_propagate_;
};
}
}

#endif