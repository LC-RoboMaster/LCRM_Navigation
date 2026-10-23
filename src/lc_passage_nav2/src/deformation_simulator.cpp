// 仅用于仿真/回放。实车必须由实际底盘反馈完成，禁止用此节点代替机械确认。
#include <chrono>
#include <lc_passage_msgs/msg/deformation_command.hpp>
#include <lc_passage_msgs/msg/deformation_feedback.hpp>
#include <rclcpp/rclcpp.hpp>

class DeformationSimulator : public rclcpp::Node {
  lc_passage_msgs::msg::DeformationCommand command_;
  std::chrono::steady_clock::time_point started_{};
  rclcpp::Subscription<lc_passage_msgs::msg::DeformationCommand>::SharedPtr
      sub_;
  rclcpp::Publisher<lc_passage_msgs::msg::DeformationFeedback>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double delay_;

public:
  DeformationSimulator() : Node("deformation_simulator") {
    delay_ = declare_parameter<double>("delay_seconds", .5);
    pub_ = create_publisher<lc_passage_msgs::msg::DeformationFeedback>(
        "passage/deformation_feedback", 10);
    sub_ = create_subscription<lc_passage_msgs::msg::DeformationCommand>(
        "passage/deformation_command",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](lc_passage_msgs::msg::DeformationCommand::ConstSharedPtr m) {
          if (m->request_id != command_.request_id) {
            command_ = *m;
            started_ = std::chrono::steady_clock::now();
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (command_.request_id.empty() ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        started_)
                  .count() < delay_)
        return;
      lc_passage_msgs::msg::DeformationFeedback f;
      f.header.stamp = now();
      f.request_id = command_.request_id;
      f.mode = command_.mode;
      f.completed = true;
      pub_->publish(f);
    });
    RCLCPP_WARN(get_logger(), "SIMULATION deformation feedback enabled");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeformationSimulator>());
  rclcpp::shutdown();
}
