#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_INTERFACE_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_INTERFACE_HPP_

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/tcl_node_interfaces/node_release_timer_interface.hpp"
#include "rclcpp/tcl_node_interfaces/node_profile_interface.hpp"
#include "rclcpp/tcl_node_interfaces/node_timing_propagate_interface.hpp"

#include <unistd.h>

namespace rclcpp
{
namespace tcl_node_interfaces
{
class NodeTimingInterface
{
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimingInterface)

    RCLCPP_PUBLIC
    virtual ~NodeTimingInterface() = default;
    
    RCLCPP_PUBLIC
    virtual bool sched_set_cpu_affinity(int64_t cpu_affinity) = 0;

    RCLCPP_PUBLIC
    virtual bool sched_set_scheduler(int64_t priority, int scheduling_algorithm = SCHED_FIFO) = 0;

    RCLCPP_PUBLIC
    virtual std::vector<std::string> get_blocking_topics() const = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_cpu_affinity() const = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_rate() const = 0;
    
    RCLCPP_PUBLIC
    virtual int64_t get_phase() const = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_priority() const = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_message_communication_type() const = 0;

    RCLCPP_PUBLIC
    virtual bool profile_enabled() const = 0;

    RCLCPP_PUBLIC
    virtual std::chrono::nanoseconds get_node_period_ns() const = 0;

    RCLCPP_PUBLIC
    virtual bool use_tcl() const = 0;
    
    RCLCPP_PUBLIC
    virtual bool use_blocking_io() const = 0;

    RCLCPP_PUBLIC
    virtual void print_node_sched_info() = 0;
    
    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_node_interfaces::NodeReleaseTimerInterface::SharedPtr get_timer() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_node_interfaces::NodeProfileInterface::SharedPtr get_profiler() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_node_interfaces::NodeTimingPropagateInterface::SharedPtr get_timing_propagator() const = 0;

    RCLCPP_PUBLIC
    virtual void create_timing_header() = 0;

    RCLCPP_PUBLIC
    virtual void propagate_timing_info() = 0;
    
    RCLCPP_PUBLIC
    virtual void receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg) = 0;

    RCLCPP_PUBLIC
    virtual void receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg) = 0;

    RCLCPP_PUBLIC
    virtual void receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg) = 0;

    RCLCPP_PUBLIC
    virtual tcl_std_msgs::msg::TimingHeader::SharedPtr get_timing_header() const = 0;
};

}
}

#endif