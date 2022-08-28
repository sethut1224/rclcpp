#include "rclcpp/tcl_timing_interfaces/tcl_timing_message_propagate.hpp"
#include  "rclcpp/time.hpp"
#include <iostream>

using rclcpp::tcl_timing_interfaces::TimingMessagePropagate;

TimingMessagePropagate::TimingMessagePropagate(std::string task_name)
:timing_header_msg_(new TimingCoordinationHeader())
{
    task_name_ = task_name;
}

TimingMessagePropagate::TimingMessagePropagate(const char* task_name)
:timing_header_msg_(new TimingCoordinationHeader())
{
    task_name_ = std::string(task_name);
}

void 
TimingMessagePropagate::create_subscription(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
    std::string topic_name)
{
    auto sub = rclcpp::create_subscription<tcl_msgs::msg::TimingCoordinationHeader>(
        node_parameters,
        node_topics,
        topic_name,
        rclcpp::QoS(KeepLast(1)),
        std::bind(&TimingMessagePropagate::timing_message_callback, this, std::placeholders::_1));

    timing_subscriber_map_[topic_name] = sub;   
}

void
TimingMessagePropagate::create_publisher(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
    std::string topic_name)
{
    auto pub = rclcpp::detail::create_publisher<tcl_msgs::msg::TimingCoordinationHeader>(
        node_parameters,
        node_topics,
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(1)));
    
    timing_publisher_ = pub;
}

void TimingMessagePropagate::timing_message_callback(const TimingCoordinationHeader::SharedPtr msg)
{
    TimingCoordinationHeader::SharedPtr timing_header = std::make_shared<TimingCoordinationHeader>(*msg);
    receive_timing_header(*timing_header);
}

void TimingMessagePropagate::publish_timing_message()
{
    timing_publisher_->publish(*timing_header_msg_);
}

void TimingMessagePropagate::receive_timing_header(TimingCoordinationHeader& msg)
{
    std::for_each(msg.msg_infos.begin(), msg.msg_infos.end(), [&](auto& msg_info)
    {
        msg_info.task_history.push_back(task_name_);

        if(timing_header_msg_->msg_infos.size() > 0)
        {
            auto ret = std::find_if(timing_header_msg_->msg_infos.begin(), timing_header_msg_->msg_infos.end(), [&](auto& msg_info_)
            {
                return msg_info_.task_history == msg_info.task_history;
            });

            if(ret == timing_header_msg_->msg_infos.end())
            {
                timing_header_msg_->msg_infos.push_back(msg_info);
            }
            else
            {
                ret->msg_id = msg_info.msg_id;
                ret->create_time = msg_info.create_time;
            }
        }
        else
        {
            timing_header_msg_->msg_infos.push_back(msg_info);
        }
    });

    timing_header_msg_->task_name = task_name_;
}


TimingCoordinationHeader TimingMessagePropagate::create_timing_header()
{   
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec));

    int64_t create_time = now.nanoseconds();
    uint64_t msg_id = create_time / 10000;

    TimingCoordinationHeader timing_header;
    tcl_msgs::msg::MsgInfo msg_info;

    msg_info.msg_id = msg_id;
    msg_info.create_time = create_time;
    msg_info.task_history.push_back(task_name_);

    timing_header.msg_infos.push_back(msg_info);
    timing_header.task_name = task_name_;
    *timing_header_msg_.get() = timing_header;

    return *timing_header_msg_.get();
}

tcl_msgs::msg::TimingCoordinationHeader::SharedPtr
TimingMessagePropagate::get_timing_header_ptr() const
{
    return this->timing_header_msg_;
}


