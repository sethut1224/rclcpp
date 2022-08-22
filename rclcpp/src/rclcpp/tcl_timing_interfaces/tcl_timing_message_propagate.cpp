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

TimingCoordinationHeader TimingMessagePropagate::propagate_timing_header()
{
    TimingCoordinationHeader msg = *timing_header_msg_.get();
    std::for_each(msg.msg_infos.begin(), msg.msg_infos.end(), [&](auto msg_info)
    {
        msg_info.task_history.push_back(task_name_);
    });

    return msg;
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


