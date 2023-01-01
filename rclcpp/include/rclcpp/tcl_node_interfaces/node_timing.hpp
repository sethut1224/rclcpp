#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_HPP_

#include "rclcpp/tcl_node_interfaces/node_timing_interface.hpp"

namespace rclcpp
{
namespace tcl_node_interfaces
{
class NodeTiming : public NodeTimingInterface
{
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTiming)

    RCLCPP_PUBLIC
    NodeTiming(
        node_interfaces::NodeBaseInterface::SharedPtr node_base,
        node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
        node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
        const rclcpp::NodeOptions options
    );

    RCLCPP_PUBLIC
    ~NodeTiming() = default;
    
    RCLCPP_PUBLIC
    bool sched_set_cpu_affinity(int64_t cpu_affinity) override;

    RCLCPP_PUBLIC
    bool sched_set_scheduler(int64_t priority, int scheduling_algorithm = SCHED_FIFO) override;

    RCLCPP_PUBLIC
    std::vector<std::string> get_blocking_topics() const override;

    RCLCPP_PUBLIC
    int64_t get_cpu_affinity() const override;

    RCLCPP_PUBLIC
    int64_t get_rate() const override;
    
    RCLCPP_PUBLIC
    int64_t get_phase() const override;

    RCLCPP_PUBLIC
    int64_t get_priority() const override;

    RCLCPP_PUBLIC
    int64_t get_message_communication_type() const override;

    RCLCPP_PUBLIC
    bool profile_enabled() const override;

    RCLCPP_PUBLIC
    std::chrono::nanoseconds get_node_period_ns() const override;

    RCLCPP_PUBLIC
    bool use_tcl() const override;

    RCLCPP_PUBLIC
    bool use_blocking_io() const override;

    RCLCPP_PUBLIC
    void print_node_sched_info() override;
    
    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const override;

    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() const override;

    RCLCPP_PUBLIC
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() const override;

    RCLCPP_PUBLIC
    rclcpp::tcl_node_interfaces::NodeReleaseTimerInterface::SharedPtr get_timer() const override;

    RCLCPP_PUBLIC
    rclcpp::tcl_node_interfaces::NodeProfileInterface::SharedPtr get_profiler() const override;

    RCLCPP_PUBLIC
    rclcpp::tcl_node_interfaces::NodeTimingPropagateInterface::SharedPtr get_timing_propagator() const override;

    RCLCPP_PUBLIC
    void create_timing_header() override;
    
    RCLCPP_PUBLIC
    void propagate_timing_info() override;

    RCLCPP_PUBLIC
    void receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg) override;

    RCLCPP_PUBLIC
    void receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg) override;

    RCLCPP_PUBLIC
    void receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg) override;

    RCLCPP_PUBLIC
    tcl_std_msgs::msg::TimingHeader::SharedPtr get_timing_header() const override;

private:
    node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
    const rclcpp::NodeOptions options_;

    rclcpp::tcl_node_interfaces::NodeReleaseTimerInterface::SharedPtr timer_;
    rclcpp::tcl_node_interfaces::NodeProfileInterface::SharedPtr profiler_;
    rclcpp::tcl_node_interfaces::NodeTimingPropagateInterface::SharedPtr propagator_;
    
    int64_t cpu_affinity_;
    int64_t rate_;
    int64_t phase_;
    int64_t priority_;
    int64_t message_communication_type_;

    bool use_tcl_;

    std::vector<std::string> blocking_topics_;
    std::vector<std::string> sub_timing_topics_;

    int64_t global_ref_time_point_;
    
    bool enable_profile_;
    int timerfd_;
};

}
}

#endif