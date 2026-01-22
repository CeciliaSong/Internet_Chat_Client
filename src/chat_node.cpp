#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<iostream>
#include<string>
#include<chrono>
#include<thread>
#include<sstream>
#include<cstdlib>
#include<cstdio>
#include<ctime>
#include<iomanip>

class ChatNode : public rclcpp::Node {
public:
    ChatNode(const std::string & username)
    : Node("chat_node"), username_(username) 
    {
        current_group_ = "general";
        join_group(current_group_);

        RCLCPP_INFO(get_logger(), "Welcome %s! You have joined the '%s' group.", username_.c_str(), current_group_.c_str());
    }

    void join_group(const std::string & group_name) 
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

    void send_message(const std::string & message_text) 
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

    const std::string & get_current_group() const {return current_group_;}
    const std::string & get_username() const {return username_;}

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg) 
    {
        const auto & data = msg->data;

        const auto first_sep = data.find('|');
        const auto second_sep = (first_sep == std::string::npos) ? std::string::npos : data.find('|', first_sep + 1);

        if(first_sep == std::string::npos || second_sep == std::string::npos)
        {
            RCLCPP_INFO(get_logger(), "%s", data.c_str());
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
    }

    std::string current_timestamp() const
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
    std::string username_;
    std::string current_group_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

static volatile bool running = true;

void signal_handler(int) 
{
    running = false;
}

int main(int argc, char ** argv) 
{
    rclcpp::init(argc, argv);

    std::string username;
    std::cout << "Please enter your username: ";
    std::getline(std::cin, username);
    if(username.empty()) 
    {
        username = "Anonymous";
    }

    auto chat_node = std::make_shared<ChatNode>(username);

    std::signal(SIGINT, signal_handler);

    std::thread input_thread([&chat_node]() 
    {
        rclcpp::spin(chat_node);
    });

    std::string input_line;

    try 
    {
        while (running && rclcpp::ok()) 
        {
            std::cout<<"["<<chat_node->get_current_group()<<"] "<<chat_node->get_username()<<": "<<std::flush;
            if(!std::getline(std::cin, input_line)) 
            {
                break;
            }
            if(input_line.substr(0, 6) == "/join ") 
            {
                std::string new_group = input_line.substr(6);
                if(!new_group.empty())
                {
                    chat_node->join_group(new_group);
                } 
                else 
                {
                    std::cout<<"Usage: /join <group_name>"<<std::endl;
                }
            } 
            else 
            {
                if(input_line=="/exit"||input_line=="/quit") 
                {
                    break;
                }
                else 
                if(input_line=="/help")
                {
                    std::cout<<"\nCommands:"<<std::endl;
                    std::cout<<"/join <group_name> - Swtich to chat group"<<std::endl;
                    std::cout<<"/exit or /quit - Exit the chat client"<<std::endl;
                    std::cout<<"/help - Show this help message\n"<<std::endl;
                }
                else
                {
                    if(!input_line.empty())
                    {
                        chat_node->send_message(input_line);
                    }
                }
            }
        }
    } 
    catch (const std::exception & e) 
    {
        RCLCPP_ERROR(chat_node->get_logger(), "Exception in main loop: %s", e.what());
    }
    running = false;
    if(input_thread.joinable())
    {
        input_thread.join();
    }
    std::cout<<"Exiting chat client. Goodbye!"<<std::endl; 
    rclcpp::shutdown();
    return 0;
}