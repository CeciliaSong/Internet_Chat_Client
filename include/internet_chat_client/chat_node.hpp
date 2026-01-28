#ifndef INTERNET_CHAT_CLIENT_CHAT_NODE_HPP
#define INTERNET_CHAT_CLIENT_CHAT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <functional>
#include <memory>

class ChatNode : public rclcpp::Node 
{
public:
    using MessageCallback = std::function<void(const std::string& sender, const std::string& timestamp, const std::string& text)>;
    
    ChatNode(const std::string & username);
    
    void join_group(const std::string & group_name);
    
    void send_message(const std::string & message_text);
    
    void set_message_callback(MessageCallback callback);
    
    const std::string & get_current_group() const { return current_group_; }
    const std::string & get_username() const { return username_; }

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg);
    
    std::string current_timestamp() const;
    
    std::string username_;
    std::string current_group_;
    MessageCallback user_callback_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif // INTERNET_CHAT_CLIENT_CHAT_NODE_HPP
