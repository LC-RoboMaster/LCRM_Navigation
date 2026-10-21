#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

class NavigationBridge : public rclcpp::Node{

public: 
    NavigationBridge();

private:
    void navigationMsgSub(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_msg_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_msg_pub_; 


};

#endif