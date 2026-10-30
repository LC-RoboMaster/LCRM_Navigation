#include "pb_omni_pid_pursuit_controller/passage_controller.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
namespace {
class RosPassageTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::executors::SingleThreadedExecutor executor;
  void pump(double seconds = .15) {
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }
  std::string file(const std::string &suffix) {
    return (std::filesystem::temp_directory_path() /
            ("lcrm_passage_" + std::to_string(getpid()) + suffix + ".json"))
        .string();
  }
  void writeConfig(const std::string &filename, double center = 0,
                   double width = 1.2, bool clear_unknown = false) {
    nlohmann::json j = {
        {"schema_version", 1},
        {"frame_id", "map"},
        {"passages", nlohmann::json::array({{{"id", "door"},
                                             {"region",
                                              {{"center", {center, 0.}},
                                               {"length", .5},
                                               {"width", width},
                                               {"yaw", 0.}}},
                                             {"endpoint_a", {center - 1., 0.}},
                                             {"endpoint_b", {center + 1., 0.}},
                                             {"speed", .3},
                                             {"clear_unknown", clear_unknown},
                                             {"prepare_distance", .5},
                                             {"release_distance", .4}}})}};
    std::ofstream(filename) << j.dump();
  }
};

TEST_F(RosPassageTest, ControllerAdoptionAcknowledgementAndRelease) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "controller_adapter_test");
  auto io = std::make_shared<rclcpp::Node>("controller_adapter_io");
  executor.add_node(node->get_node_base_interface());
  executor.add_node(io);
  pb_omni_pid_pursuit_controller::PassageController controller(node, "FollowPath");
  auto plans = io->create_publisher<lc_passage_msgs::msg::PassagePlan>(
      "passage/plan", rclcpp::QoS(32).reliable().transient_local());
  auto feedback =
      io->create_publisher<lc_passage_msgs::msg::DeformationFeedback>(
          "passage/deformation_feedback", 10);
  lc_passage_msgs::msg::PassagePlan p;
  p.plan_id = "execution-test";
  p.header.frame_id = p.path.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * .1;
    pose.pose.orientation.w = 1.;
    p.path.poses.push_back(pose);
    p.arc_lengths.push_back(i * .1);
  }
  lc_passage_msgs::msg::PassageTraversal t;
  t.passage_id = "door";
  t.mode = "folded";
  t.direction = "a_to_b";
  t.prepare_s = 1;
  t.enter_s = 2;
  t.exit_s = 4;
  t.release_s = 5;
  t.speed = .3;
  t.fold_hold_seconds = .1;
  t.restore_hold_seconds = .1;
  p.traversals.push_back(t);
  controller.setPlan(p.path);
  EXPECT_TRUE(controller.step({0, 0}, 0, 0).stop);
  pump();
  plans->publish(p);
  pump();
  auto out = controller.step({0, 0}, 2., 0);
  EXPECT_EQ(out.phase, "APPROACH");
  EXPECT_TRUE(std::isinf(out.speed_limit));
  EXPECT_FALSE(out.stop);
  out = controller.step({.95, 0}, 1., 0);
  EXPECT_EQ(out.phase, "APPROACH");
  EXPECT_TRUE(std::isinf(out.speed_limit));
  EXPECT_FALSE(out.stop);
  EXPECT_DOUBLE_EQ(out.target_limit, t.prepare_s);
  out = controller.step({1, 0}, 1., 0);
  EXPECT_TRUE(out.stop);
  EXPECT_FALSE(controller.locked());
  out = controller.step({1, 0}, 0, 0);
  ASSERT_EQ(out.phase, "WAIT_FOLD");
  ASSERT_TRUE(controller.locked());
  lc_passage_msgs::msg::DeformationFeedback ack;
  ack.request_id = "stale";
  ack.mode = "folded";
  ack.completed = true;
  feedback->publish(ack);
  pump();
  EXPECT_TRUE(controller.step({1, 0}, 0, 0).stop);
  ack.request_id = out.request_id;
  feedback->publish(ack);
  pump();
  out = controller.step({1, 0}, 0, 0);
  EXPECT_EQ(out.phase, "PASSING");
  EXPECT_DOUBLE_EQ(out.speed_limit, t.speed);
  for (int i = 2; i <= 5; ++i)
    out = controller.step({double(i), 0}, 0, 0);
  ASSERT_EQ(out.phase, "WAIT_RESTORE");
  ack.request_id = out.request_id;
  ack.mode = "normal";
  feedback->publish(ack);
  pump();
  out = controller.step({5, 0}, 0, 0);
  EXPECT_EQ(out.phase, "NORMAL");
  EXPECT_FALSE(out.locked);
  EXPECT_FALSE(out.folded);
  EXPECT_TRUE(std::isinf(out.speed_limit));
  executor.remove_node(io);
  executor.remove_node(node->get_node_base_interface());
}
} // namespace
