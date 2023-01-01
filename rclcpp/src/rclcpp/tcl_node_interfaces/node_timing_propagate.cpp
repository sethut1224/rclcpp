#include "rclcpp/tcl_node_interfaces/node_timing_propagate.hpp"

using rclcpp::tcl_node_interfaces::NodeTimingPropagate;

NodeTimingPropagate::NodeTimingPropagate(
    const char * node_name,
    std::vector<std::string>& sub_timing_topics,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics
)
{
    (void)node_parameters;
    (void)node_topics;
    (void)sub_timing_topics;
    node_name_ = std::string(node_name);
    // const std::string suffix = "/timing_info";
    // const std::string pub_topic_name = node_name_ + suffix;

    timing_header_msg_ = std::make_shared<tcl_std_msgs::msg::TimingHeader>();

    // std::for_each(sub_timing_topics.begin(), sub_timing_topics.end(), [&](auto& topic)
    // {
    //     this->create_subscription(node_parameters, node_topics, topic);
    // });

    // this->create_publisher(node_parameters, node_topics, pub_topic_name);
}

void 
NodeTimingPropagate::create_subscription(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    std::string topic_name)
{
    auto sub = rclcpp::create_subscription<tcl_std_msgs::msg::TimingHeader>(
        node_parameters,
        node_topics,
        topic_name,
        rclcpp::QoS(KeepLast(1)),
        [&](const tcl_std_msgs::msg::TimingHeader & msg)
        {
            tcl_std_msgs::msg::TimingHeader::SharedPtr message(new tcl_std_msgs::msg::TimingHeader(msg));
            receive_timing_header(message);
        });

    timing_subscriber_map_[topic_name] = sub;
}

void
NodeTimingPropagate::create_publisher(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    std::string topic_name)
{
    timing_publisher_ = rclcpp::detail::create_publisher<tcl_std_msgs::msg::TimingHeader>(
        node_parameters,
        node_topics,
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(1)));

}

void 
NodeTimingPropagate::receive_timing_header(tcl_std_msgs::msg::TimingHeader::SharedPtr msg)
{   
    if(timing_header_msg_)
    {
        auto msg_infos = msg->msg_infos;
        std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info){
            msg_info.task_history.push_back(node_name_);
        });
    
        std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info)
        {
            if(timing_header_msg_->msg_infos.size() > 0)
            {
                auto ret = std::find_if(timing_header_msg_->msg_infos.begin(), timing_header_msg_->msg_infos.end(), [&](auto& m_msg_info)
                {
                    return msg_info.task_history == m_msg_info.task_history;
                });
    
                if(ret == timing_header_msg_->msg_infos.end())
                {
                    timing_header_msg_->msg_infos.push_back(msg_info);
                }
                else
                {
                    ret->msg_id = msg_info.msg_id;
                    ret->creation_time = msg_info.creation_time;
                }
            }
            else
            {
                timing_header_msg_->msg_infos.push_back(msg_info);
            }
        });
    
        timing_header_msg_->task_name = node_name_;
    }
}

void 
NodeTimingPropagate::receive_timing_header(tcl_std_msgs::msg::TimingHeader& msg)
{
    auto msg_infos = msg.msg_infos;
    std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info){
        msg_info.task_history.push_back(node_name_);
    });

    std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info)
    {
        if(timing_header_msg_->msg_infos.size() > 0)
        {
            auto ret = std::find_if(timing_header_msg_->msg_infos.begin(), timing_header_msg_->msg_infos.end(), [&](auto& m_msg_info)
            {
                return msg_info.task_history == m_msg_info.task_history;
            });

            if(ret == timing_header_msg_->msg_infos.end())
            {
                timing_header_msg_->msg_infos.push_back(msg_info);
            }
            else
            {
                ret->msg_id = msg_info.msg_id;
                ret->creation_time = msg_info.creation_time;
            }
        }
        else
        {
            timing_header_msg_->msg_infos.push_back(msg_info);
        }
    });

    timing_header_msg_->task_name = node_name_;
}

void 
NodeTimingPropagate::receive_timing_header(const tcl_std_msgs::msg::TimingHeader& msg)
{
    auto msg_infos = msg.msg_infos;
    std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info){
        msg_info.task_history.push_back(node_name_);
    });

    std::for_each(msg_infos.begin(), msg_infos.end(), [&](auto& msg_info)
    {
        if(timing_header_msg_->msg_infos.size() > 0)
        {
            auto ret = std::find_if(timing_header_msg_->msg_infos.begin(), timing_header_msg_->msg_infos.end(), [&](auto& m_msg_info)
            {
                return msg_info.task_history == m_msg_info.task_history;
            });

            if(ret == timing_header_msg_->msg_infos.end())
            {
                timing_header_msg_->msg_infos.push_back(msg_info);
            }
            else
            {
                ret->msg_id = msg_info.msg_id;
                ret->creation_time = msg_info.creation_time;
            }
        }
        else
        {
            timing_header_msg_->msg_infos.push_back(msg_info);
        }
    });

    timing_header_msg_->task_name = node_name_;
}

void
NodeTimingPropagate::create_timing_header()
{
    if(timing_header_msg_)
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        auto now = rclcpp::Time(static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec));

        int64_t create_time = now.nanoseconds();
        uint64_t msg_id = create_time / 10000;

        tcl_std_msgs::msg::TimingHeader timing_header;
        tcl_std_msgs::msg::MessageInfo msg_info;

        msg_info.msg_id = msg_id;
        msg_info.creation_time = create_time;
        msg_info.task_history.push_back(node_name_);

        timing_header.msg_infos.push_back(msg_info);
        timing_header.task_name = node_name_;

        *timing_header_msg_ = timing_header;
    }
}

void
NodeTimingPropagate::publish_timing_message()
{
    if(timing_publisher_)
        timing_publisher_->publish(*timing_header_msg_);
}

tcl_std_msgs::msg::TimingHeader::SharedPtr
NodeTimingPropagate::get_timing_header() const
{
    return this->timing_header_msg_;
}