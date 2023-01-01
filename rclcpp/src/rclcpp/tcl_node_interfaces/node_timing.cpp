#include "rclcpp/tcl_node_interfaces/node_timing.hpp"
#include "rclcpp/tcl_node_interfaces/node_release_timer.hpp"
#include "rclcpp/tcl_node_interfaces/node_profile.hpp"
#include "rclcpp/tcl_node_interfaces/node_timing_propagate.hpp"

using namespace std::chrono_literals;

using rclcpp::tcl_node_interfaces::NodeTiming;

NodeTiming::NodeTiming(
    node_interfaces::NodeBaseInterface::SharedPtr node_base,
    node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::NodeOptions options)
:   node_base_(node_base),
    node_parameters_(node_parameters),
    node_topics_(node_topics),
    options_(options)
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

    sub_timing_topics_ = node_parameters_->declare_parameter(
        "tcl_timing_param.sub_timing_topics",
        rclcpp::ParameterValue(std::vector<std::string>())
    ).get<std::vector<std::string>>();

    global_ref_time_point_ = node_parameters_->declare_parameter(
        "tcl_sched_param.ref_time_point",
        rclcpp::ParameterValue(-1)
    ).get<int64_t>();

    enable_profile_ = node_parameters_->declare_parameter(
        "tcl_timing_param.enable_profile",
        rclcpp::ParameterValue(false)
    ).get<bool>();

    use_tcl_ = message_communication_type_ == -1 ? false : true;

    if(blocking_topics_.size() > 0 && blocking_topics_[0].compare("") == 0)
        blocking_topics_.clear();

    if(sub_timing_topics_.size() > 0 && sub_timing_topics_[0].compare("") == 0)
        sub_timing_topics_.clear();
    
    std::for_each(blocking_topics_.begin(), blocking_topics_.end(), [&](auto& topic)
    {
        if(topic.front() != '/')
            topic = std::string("/") + topic;
    });

    std::for_each(sub_timing_topics_.begin(), sub_timing_topics_.end(), [&](auto& topic)
    {
        if(topic.front() != '/')
            topic = std::string("/") + topic;
    });

    if(priority_ > 0 && priority_ < 99)
        sched_set_scheduler(priority_);

    if(cpu_affinity_ > -1)
        sched_set_cpu_affinity(cpu_affinity_);

    if(use_tcl_)
    {
        print_node_sched_info();

        if(enable_profile_)
        {
            profiler_ = std::make_shared<rclcpp::tcl_node_interfaces::NodeProfile>(
                get_node_base_interface()->get_name(),
                get_node_parameters_interface(),
                get_node_topics_interface());
            
            propagator_ = std::make_shared<rclcpp::tcl_node_interfaces::NodeTimingPropagate>(
                get_node_base_interface()->get_name(),
                sub_timing_topics_,
                get_node_parameters_interface(),
                get_node_topics_interface());
        }

        timer_ = std::make_shared<rclcpp::tcl_node_interfaces::NodeReleaseTimer>(rate_, phase_, global_ref_time_point_);
    }   
}

bool 
NodeTiming::sched_set_cpu_affinity(int64_t cpu_affinity)
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
NodeTiming::sched_set_scheduler(int64_t priority, int scheduling_algorithm)
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
NodeTiming::get_blocking_topics() const
{
    return this->blocking_topics_;
}

// std::vector<std::string>
// NodeTiming::get_sub_timing_observation_topics() const
// {
//     return this->sub_timing_observation_topics_;
// }

int64_t 
NodeTiming::get_message_communication_type() const
{
    return this->message_communication_type_;
}

int64_t 
NodeTiming::get_cpu_affinity() const
{
    return this->cpu_affinity_;
}

int64_t
NodeTiming::get_rate() const
{
    return this->rate_;
}

int64_t 
NodeTiming::get_phase() const
{
    return this->phase_;
}

int64_t 
NodeTiming::get_priority() const
{
    return this->priority_;
}

bool
NodeTiming::use_tcl() const
{
    return this->use_tcl_;
}

bool
NodeTiming::use_blocking_io() const
{
    return (this->message_communication_type_ == 1 && this->blocking_topics_.size() > 0);
}

bool
NodeTiming::profile_enabled() const
{
    return this->enable_profile_;
}



void
NodeTiming::print_node_sched_info()
{
    std::cout<<"message communication type : "<<this->get_message_communication_type()<<std::endl;
    std::cout<<"cpu_affinity : "<<this->get_cpu_affinity()<<std::endl;
    std::cout<<"rate  : "<<this->get_rate()<<std::endl;
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
    // std::cout<<"sub timing observation topics : ";
    // if(sub_timing_topics_.size() > 0)
    // std::for_each(sub_timing_topics_.begin(), sub_timing_topics_.end(), [&](auto& topic)
    // {
    //     std::cout<<topic<<" ";
    // });
    // std::cout<<std::endl;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
NodeTiming::get_node_base_interface() const
{
    return this->node_base_;
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
NodeTiming::get_node_parameters_interface() const
{
    return this->node_parameters_;
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
NodeTiming::get_node_topics_interface() const
{
    return this->node_topics_;
}

rclcpp::tcl_node_interfaces::NodeReleaseTimerInterface::SharedPtr
NodeTiming::get_timer() const
{
    return this->timer_;
}

rclcpp::tcl_node_interfaces::NodeProfileInterface::SharedPtr
NodeTiming::get_profiler() const
{
    return this->profiler_;
}

rclcpp::tcl_node_interfaces::NodeTimingPropagateInterface::SharedPtr
NodeTiming::get_timing_propagator() const
{
    return this->propagator_;
}

std::chrono::nanoseconds
NodeTiming::get_node_period_ns() const 
{
    return this->timer_->get_period_ns();
}

void
NodeTiming::create_timing_header()
{
    if(this->propagator_)
        this->propagator_->create_timing_header();
}

void
NodeTiming::propagate_timing_info()
{
    if(this->propagator_)
        this->propagator_->publish_timing_message();
}

void
NodeTiming::receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg)
{
    if(this->propagator_)
        this->propagator_->receive_timing_header(msg);
}

void
NodeTiming::receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg)
{
    if(this->propagator_)
        this->propagator_->receive_timing_header(msg);
}

void
NodeTiming::receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg)
{
    if(this->propagator_)
        this->propagator_->receive_timing_header(msg);
}

tcl_std_msgs::msg::TimingHeader::SharedPtr
NodeTiming::get_timing_header() const
{
    if(this->propagator_)
        return this->propagator_->get_timing_header();
    else
        return nullptr;
}
