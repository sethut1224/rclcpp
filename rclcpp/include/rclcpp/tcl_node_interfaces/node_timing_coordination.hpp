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
        node_interfaces::NodeTopicsInterface::SharedPtr node_topics_,
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
    std::vector<std::string> get_timing_observation_topics() override;

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
    virtual bool topic_propagate_status(std::string topic_name) override;

    RCLCPP_PUBLIC
    void create_profile_publisher(const std::string& node_name) override;

    RCLCPP_PUBLIC
    tcl_msgs::msg::TimingCoordinationHeader create_timing_header() override;
    
    RCLCPP_PUBLIC
    virtual std::chrono::nanoseconds get_period_ns() const override;

    template<
    typename MessageT = tcl_msgs::msg::Profile,
    typename AllocatorT =  std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
    void create_publisher_impl(
        std::string topic_name,
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)),
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
        rclcpp::PublisherOptionsWithAllocator<AllocatorT>()))
    {
        auto pub = node_topics_->create_publisher(
          topic_name,
          rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(options),
          qos);
        node_topics_->add_publisher(pub, options.callback_group);

        // auto pub_cast = );
        // timing_profile_->set_profile_publisher(std::dynamic_pointer_cast<PublisherT>(pub));

        timing_profile_ = std::make_shared<
        rclcpp::tcl_timing_interfaces::TimingProfile>(
            std::dynamic_pointer_cast<PublisherT>(pub));
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
    std::vector<std::string> timing_observation_topics_;

    int64_t global_ref_time_point_;

    int timerfd_;
    bool timer_ready_ {false};

    rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr timing_profile_;
    rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr timing_message_propagate_;
};
}
}

#endif