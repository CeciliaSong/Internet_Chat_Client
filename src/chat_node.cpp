#include "internet_chat_client/chat_node.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <csignal>

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
                    std::cout<<"/join <group_name> - Switch to chat group"<<std::endl;
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