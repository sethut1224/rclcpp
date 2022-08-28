#ifndef RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_COORDINATION_INTERFACE_HPP_
#define RCLCPP__TCL_NODE_INTERFACES__NODE_TIMING_COORDINATION_INTERFACE_HPP_

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/tcl_timer/release_wall_timer.hpp"
#include "rclcpp/tcl_timing_interfaces/tcl_timing_profile.hpp"
#include "rclcpp/tcl_timing_interfaces/tcl_timing_message_propagate.hpp"
#include "rclcpp/tcl_timing_header_traits/tcl_timing_header_traits.hpp"
#include <unistd.h>

namespace rclcpp
{
namespace tcl_node_interfaces
{

class NodeTimingCoordinationInterface
{
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimingCoordinationInterface)

    RCLCPP_PUBLIC
    virtual ~NodeTimingCoordinationInterface() = default;
    
    RCLCPP_PUBLIC
    virtual bool sched_set_cpu_affinity(int64_t cpu_affinity) = 0;

    RCLCPP_PUBLIC
    virtual bool sched_set_scheduler(int64_t priority, int scheduling_algorithm = SCHED_FIFO) = 0;

    RCLCPP_PUBLIC
    virtual std::vector<std::string> get_blocking_topics() = 0;

    RCLCPP_PUBLIC
    virtual std::vector<std::string> get_sub_timing_observation_topics() = 0;

    RCLCPP_PUBLIC
    virtual std::vector<std::string> get_pub_timing_observation_topics() = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_cpu_affinity() = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_rate() = 0;
    
    RCLCPP_PUBLIC
    virtual int64_t get_phase() = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_priority() = 0;

    RCLCPP_PUBLIC
    virtual int64_t get_message_communication_type() = 0;

    RCLCPP_PUBLIC
    virtual bool get_enable_profile() = 0;

    RCLCPP_PUBLIC
    virtual std::chrono::nanoseconds get_period_ns() const = 0;

    RCLCPP_PUBLIC
    virtual bool blocking_io() = 0;

    RCLCPP_PUBLIC
    virtual void print_node_sched_info() = 0;
    
    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr get_timing_profile() = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr get_timing_message_propagate() = 0;

    RCLCPP_PUBLIC
    virtual bool use_tcl() = 0;
    
    RCLCPP_PUBLIC
    virtual rclcpp::Time get_local_ref_time() = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::Time get_global_ref_time() = 0;
    
    RCLCPP_PUBLIC
    virtual const rclcpp::NodeOptions & get_node_options() const = 0;

    RCLCPP_PUBLIC
    virtual rclcpp::tcl_timer::ReleaseWallTimer::SharedPtr get_timer() = 0;

    RCLCPP_PUBLIC
    virtual tcl_msgs::msg::TimingCoordinationHeader::SharedPtr get_timing_header_ptr() const = 0;

    RCLCPP_PUBLIC
    virtual void propagate_timing_message()  = 0;

    RCLCPP_PUBLIC
    virtual tcl_msgs::msg::TimingCoordinationHeader create_timing_header() = 0;

};

}
}

#endif