#include "lc_passage_nav2/tdt_passage_planner.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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

TEST_F(RosPassageTest, LiveCostmapResizeAndAnnotatedNarrowCenterline) {
  auto io = std::make_shared<rclcpp::Node>("planner_diagnostic_io");
  executor.add_node(io);
  std_msgs::msg::String::SharedPtr diagnostic;
  auto diagnostic_sub = io->create_subscription<std_msgs::msg::String>(
      "passage/planner_diagnostics", rclcpp::QoS(1).reliable().transient_local(),
      [&](std_msgs::msg::String::SharedPtr m) { diagnostic = m; });
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("plugins", std::vector<std::string>{}),
           rclcpp::Parameter("robot_radius", .25)}));
  map->configure();
  ASSERT_NE(map->getCostmap(), nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "live_planner_test",
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("GridBased.optimize", false)}));
  lc_passage_nav2::TdtPassagePlanner planner;
  planner.configure(node, "GridBased", map->getTfBuffer(), map);
  planner.activate();
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = map->getGlobalFrameID();
  start.pose.orientation.w = goal.pose.orientation.w = 1.;
  goal.pose.position.x = 4;
  auto *grid = map->getCostmap();
  grid->resizeMap(200, 160, .05, -2, -2);
  std::memset(grid->getCharMap(), 0, 200 * 160);
  auto path = planner.createPlan(start, goal);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().pose.position.x, 4, 1e-6);
  grid->resizeMap(320, 240, .05, -6, -4);
  std::memset(grid->getCharMap(), 0, 320 * 240);
  path = planner.createPlan(start, goal);
  EXPECT_NEAR(path.poses.front().pose.position.x, 0, 1e-6);
  planner.deactivate();
  planner.cleanup();
  const auto filename = file("planner");
  writeConfig(filename, 2., .4, true);
  auto annotated = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "annotated_planner_test",
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("GridBased.passage_file", filename),
           rclcpp::Parameter("GridBased.optimize", false)}));
  planner.configure(annotated, "GridBased", map->getTfBuffer(), map);
  planner.activate();
  // 小于普通机器人膨胀直径的已标注门洞，仍走机械保证的中线。
  for (unsigned y = 0; y < 240; ++y) {
    double wy = -4 + (y + .5) * .05;
    if (std::abs(wy) > .2)
      grid->setCost(160, y, 254);
  }
  path = planner.createPlan(start, goal);
  ASSERT_FALSE(path.poses.empty());
  pump();
  ASSERT_TRUE(diagnostic);
  auto report = nlohmann::json::parse(diagnostic->data);
  EXPECT_EQ(report["selected_count"], 1);
  EXPECT_TRUE(report["passage_checks"][0]["valid"].get<bool>());
  for (const auto &p : path.poses)
    if (p.pose.position.x > 1.75 && p.pose.position.x < 2.25)
      EXPECT_NEAR(p.pose.position.y, 0, 1e-5);
  grid->setCost(160, 80, 254); // 中线出现新的真实障碍，语义不能将它清除。
  EXPECT_THROW(planner.createPlan(start, goal), nav2_core::PlannerException);
  pump();
  report = nlohmann::json::parse(diagnostic->data);
  EXPECT_FALSE(report["success"].get<bool>());
  EXPECT_EQ(report["passage_checks"][0]["reason"].get<std::string>().find("centerline_obstacle"), 0u);
  grid->setCost(160,80,255);
  EXPECT_THROW(planner.createPlan(start,goal),nav2_core::PlannerException);
  pump();
  report=nlohmann::json::parse(diagnostic->data);
  EXPECT_EQ(report["passage_checks"][0]["reason"].get<std::string>().find("centerline_unknown"),0u);
  planner.deactivate();
  planner.cleanup();
  map->cleanup();
  std::filesystem::remove(filename);
  executor.remove_node(io);
}

} // namespace
