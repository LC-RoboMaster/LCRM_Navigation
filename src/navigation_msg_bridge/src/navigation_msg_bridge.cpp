#include "../include/navigation_msg_bridge/navigation_msg_bridge.hpp"

NavigationBridge::NavigationBridge() : Node("Navigation_msg_bridge"){

    nav_msg_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&NavigationBridge::navigationMsgSub, this, std::placeholders::_1)
    );
    nav_msg_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_to_cv", 10);

    RCLCPP_INFO(this->get_logger(), "Navigation msg bridge node init!");
}

void NavigationBridge::navigationMsgSub(const geometry_msgs::msg::Twist::SharedPtr msg){
    geometry_msgs::msg::Twist msg_temp = *msg;
    nav_msg_pub_->publish(msg_temp);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto lc_robot_cmd = std::make_shared<NavigationBridge>();
    rclcpp::spin(lc_robot_cmd);
    rclcpp::shutdown();
    return 0;
}