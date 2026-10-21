#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FakeLocalization : public rclcpp::Node
{
public:
    FakeLocalization() : Node("fake_localization")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&FakeLocalization::callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&FakeLocalization::publishTF, this));

        // 初始为0
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        auto &q = msg->pose.pose.orientation;
        yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        RCLCPP_INFO(this->get_logger(), "Updated pose: x=%.2f y=%.2f yaw=%.2f", x_, y_, yaw_);
    }

    void publishTF()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.z = sin(yaw_ / 2.0);
        t.transform.rotation.w = cos(yaw_ / 2.0);

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, yaw_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLocalization>());
    rclcpp::shutdown();
    return 0;
}