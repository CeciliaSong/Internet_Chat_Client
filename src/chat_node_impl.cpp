#include "internet_chat_client/chat_node.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

ChatNode::ChatNode(const std::string & username)
    : Node("chat_node"), username_(username), user_callback_(nullptr)
{
    current_group_ = "general";
    join_group(current_group_);

    RCLCPP_INFO(get_logger(), "Welcome %s! You have joined the '%s' group.", username_.c_str(), current_group_.c_str());
}

void ChatNode::join_group(const std::string & group_name) 
{
    if (subscription_)
    {
        subscription_.reset();
    }

    const std::string topic_name = "/chat/" + group_name;

    publisher_ = create_publisher<std_msgs::msg::String>(topic_name, 10);

    subscription_ = create_subscription<std_msgs::msg::String>(
        topic_name, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) 
        {
            message_callback(msg);
        }
    );

    current_group_ = group_name;
    RCLCPP_INFO(get_logger(), "Joined group: %s", current_group_.c_str());
}

void ChatNode::send_message(const std::string & message_text) 
{
    if(!message_text.empty())
    {
        const auto timestamp = current_timestamp();
        auto message = std_msgs::msg::String();
        message.data = username_ + "|" + timestamp + "|" + message_text;
        publisher_->publish(message);
        std::cout << "[" << timestamp << "] " << username_ << ": " << message_text << std::endl;
    }
}

void ChatNode::set_message_callback(MessageCallback callback)
{
    user_callback_ = callback;
}

void ChatNode::message_callback(const std_msgs::msg::String::SharedPtr msg) 
{
    const auto & data = msg->data;

    const auto first_sep = data.find('|');
    const auto second_sep = (first_sep == std::string::npos) ? std::string::npos : data.find('|', first_sep + 1);

    if(first_sep == std::string::npos || second_sep == std::string::npos)
    {
        RCLCPP_INFO(get_logger(), "%s", data.c_str());
        if(user_callback_)
        {
            user_callback_("System", "", data);
        }
        return;
    }

    const auto sender = data.substr(0, first_sep);
    if(sender == username_)
    {
        return;
    }

    const auto timestamp = data.substr(first_sep + 1, second_sep - first_sep - 1);
    const auto text = data.substr(second_sep + 1);

    std::cout << "[" << timestamp << "] " << sender << ": " << text << std::endl;
    
    if(user_callback_)
    {
        user_callback_(sender, timestamp, text);
    }
}

std::string ChatNode::current_timestamp() const
{
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}
