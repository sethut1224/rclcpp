#include "rclcpp/tcl_node_interfaces/node_timing_coordination.hpp"

using namespace std::chrono_literals;

using rclcpp::tcl_node_interfaces::NodeTimingCoordination;

NodeTimingCoordination::NodeTimingCoordination(
    node_interfaces::NodeBaseInterface::SharedPtr node_base,
    node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::NodeOptions options)
:   node_base_(node_base),
    node_parameters_(node_parameters),
    node_topics_(node_topics),
    options_(options),
    timing_message_propagate_(new rclcpp::tcl_timing_interfaces::TimingMessagePropagate(node_base_->get_name()))
{
    cpu_affinity_ = node_parameters_->declare_parameter(
        "tcl_sched_param.cpu",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    rate_ = node_parameters_->declare_parameter(
        "tcl_sched_param.rate",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    phase_ = node_parameters_->declare_parameter(
        "tcl_sched_param.phase",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    priority_ = node_parameters_->declare_parameter(
        "tcl_sched_param.priority", 
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    message_communication_type_ = node_parameters_->declare_parameter(
        "tcl_sched_param.type",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    blocking_topics_ = node_parameters_->declare_parameter(
        "tcl_sched_param.blocking_topics", 
        rclcpp::ParameterValue(std::vector<std::string>())
    ).get<std::vector<std::string>>();

    timing_observation_topics_ = node_parameters_->declare_parameter(
        "tcl_timing_param.timing_observation_topics",
        rclcpp::ParameterValue(std::vector<std::string>())
    ).get<std::vector<std::string>>();

    global_ref_time_point_ = node_parameters_->declare_parameter(
        "tcl_sched_param.ref_time_point",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    use_tcl_ = message_communication_type_ == -1 ? false : true;

    if(blocking_topics_.size() > 0 && blocking_topics_[0].compare("") == 0)
        blocking_topics_.clear();
    
    if(timing_observation_topics_.size() > 0 && timing_observation_topics_[0].compare("") == 0)
        timing_observation_topics_.clear();
    
    std::for_each(blocking_topics_.begin(), blocking_topics_.end(), [&](auto& topic)
    {
        if(topic.front() != '/')
            topic = std::string("/") + topic;
    });

    std::for_each(timing_observation_topics_.begin(), timing_observation_topics_.end(), [&](auto& topic)
    {
        if(topic.front() != '/')
            topic = std::string("/") + topic;
    });

    if(priority_ > 0 && priority_ < 99)
    {
        sched_set_scheduler(priority_);
    }
    
    if(cpu_affinity_ > -1)
    {
        sched_set_cpu_affinity(cpu_affinity_);
    }

    if(use_tcl_)
    {
        timer_ = std::make_shared<rclcpp::tcl_timer::ReleaseWallTimer>(rate_, phase_, global_ref_time_point_);
        create_profile_publisher(std::string(node_base_->get_name()));
    }
    
    // print_node_sched_info();
}

bool 
NodeTimingCoordination::sched_set_cpu_affinity(int64_t cpu_affinity)
{
    unsigned long cpu = static_cast<unsigned long>(cpu_affinity);
    cpu_set_t set;
    
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    
    int ret = sched_setaffinity(getpid(), sizeof(cpu_set_t), &set);

    if(ret == -1) {
        perror("sched_setaffinity");
        return false;
    }

    return true;
}

bool 
NodeTimingCoordination::sched_set_scheduler(int64_t priority, int scheduling_algorithm)
{
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = static_cast<int>(priority);

    int ret = sched_setscheduler(getpid(), scheduling_algorithm, &param);

    if(ret == -1) {
        perror("sched_setscheduler");
        return false;
    }
    
    return true;
}

std::vector<std::string>
NodeTimingCoordination::get_blocking_topics()
{
    return this->blocking_topics_;
}

std::vector<std::string>
NodeTimingCoordination::get_timing_observation_topics()
{
    return this->timing_observation_topics_;
}

int64_t 
NodeTimingCoordination::get_cpu_affinity()
{
    return this->cpu_affinity_;
}

int64_t
NodeTimingCoordination::get_rate()
{
    return this->rate_;
}

int64_t 
NodeTimingCoordination::get_phase()
{
    return this->phase_;
}

int64_t 
NodeTimingCoordination::get_priority()
{
    return this->priority_;
}

int64_t 
NodeTimingCoordination::get_message_communication_type()
{
    return this->message_communication_type_;
}

bool
NodeTimingCoordination::use_tcl()
{
    return this->use_tcl_;
}

bool
NodeTimingCoordination::blocking_io()
{
    return (this->message_communication_type_ == 1 && this->blocking_topics_.size() > 0);
}

void
NodeTimingCoordination::print_node_sched_info()
{
    std::cout<<"message communication type : "<<this->get_message_communication_type()<<std::endl;
    std::cout<<"cpu_affinity : "<<this->get_cpu_affinity()<<std::endl;
    std::cout<<"phase : "<<this->get_phase()<<std::endl;
    std::cout<<"priority : "<<this->get_priority()<<std::endl;

    std::cout<<"blocking topics : ";
    if(blocking_topics_.size() > 0)
    {
        std::for_each(blocking_topics_.begin(), blocking_topics_.end(), [&](auto& topic)
        {
            std::cout<<topic<<" ";
        });
    }
    std::cout<<std::endl;
    std::cout<<"timing observation topics : ";
    if(timing_observation_topics_.size() > 0)
    std::for_each(timing_observation_topics_.begin(), timing_observation_topics_.end(), [&](auto& topic)
    {
        std::cout<<topic<<" ";
    });
    std::cout<<std::endl;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
NodeTimingCoordination::get_node_base_interface()
{
    return this->node_base_;
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
NodeTimingCoordination::get_node_parameters_interface()
{
    return this->node_parameters_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
NodeTimingCoordination::get_node_topics_interface()
{
    return this->node_topics_;
}

rclcpp::Time
NodeTimingCoordination::get_global_ref_time()
{
    return this->timer_->get_global_ref_time();
}

rclcpp::Time
NodeTimingCoordination::get_local_ref_time()
{
    return this->timer_->get_local_ref_time();
}

const rclcpp::NodeOptions&
NodeTimingCoordination::get_node_options() const
{
    return this->options_;
}

rclcpp::tcl_timing_interfaces::TimingProfile::SharedPtr
NodeTimingCoordination::get_timing_profile()
{
    return this->timing_profile_;
}

rclcpp::tcl_timing_interfaces::TimingMessagePropagate::SharedPtr
NodeTimingCoordination::get_timing_message_propagate()
{
    return this->timing_message_propagate_;
}

rclcpp::tcl_timer::ReleaseWallTimer::SharedPtr
NodeTimingCoordination::get_timer()
{
    return this->timer_;
}

tcl_msgs::msg::TimingCoordinationHeader::SharedPtr
NodeTimingCoordination::get_timing_header_ptr() const
{
    return timing_message_propagate_->get_timing_header_ptr();
}
 
bool 
NodeTimingCoordination::topic_propagate_status(std::string topic_name)
{   
    if(topic_name.front() != '/')
        topic_name = std::string("/") + topic_name;

    auto ret = std::find(timing_observation_topics_.begin(), timing_observation_topics_.end(), topic_name);
    
    return !(ret == timing_observation_topics_.end());
}

void
NodeTimingCoordination::create_profile_publisher(const std::string& node_name)
{
    std::string timing_topic_name = node_name + std::string("/timing");
    create_publisher_impl(timing_topic_name);
}

tcl_msgs::msg::TimingCoordinationHeader
NodeTimingCoordination::create_timing_header()
{
    return timing_message_propagate_->create_timing_header();
}

std::chrono::nanoseconds
NodeTimingCoordination::get_period_ns() const 
{
    return this->timer_->get_period();
}
