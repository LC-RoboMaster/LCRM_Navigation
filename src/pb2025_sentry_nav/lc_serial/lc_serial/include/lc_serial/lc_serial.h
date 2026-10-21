#ifndef LC_SERIAL_H
#define LC_SERIAL_H

//ros2
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
//interface
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "cJSON.h"
#include <iostream>
#include <vector>

class Serial : public rclcpp::Node{
    public:
        //初始化节点，订阅nav2的cmd_vel话题
        Serial();
        ~Serial();
    
    private:

        void sendData(const geometry_msgs::msg::Twist::SharedPtr msg);

        void reopenPort();

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
        // std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::shared_ptr<drivers::common::IoContext> io_context_;
        std::string device_name_;
};


#endif