#include "lc_passage_nav2/passage_controller.hpp"
#include "lc_passage_nav2/tdt_passage_planner.hpp"
#include <class_loader/class_loader.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_components/node_factory.hpp>
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
                   double width = 1.2) {
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
                                             {"prepare_distance", .5},
                                             {"release_distance", .4}}})}};
    std::ofstream(filename) << j.dump();
  }
};

TEST_F(RosPassageTest, StaticMapAndLiveHeightFilteringAndGrowingMap) {
  const auto filename = file("map");
  writeConfig(filename);
  class_loader::ClassLoader loader(PASSAGE_COMPONENT_LIBRARY);
  auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  ASSERT_EQ(classes.size(), 1u);
  auto factory =
      loader.createInstance<rclcpp_components::NodeFactory>(classes.front());
  auto wrapper =
      factory->create_node_instance(rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("passage_file", filename)}));
  auto io = std::make_shared<rclcpp::Node>("passage_map_test_io");
  executor.add_node(wrapper.get_node_base_interface());
  executor.add_node(io);
  nav_msgs::msg::OccupancyGrid::SharedPtr received;
  auto sub = io->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "passage_map", rclcpp::QoS(1).reliable().transient_local(),
      [&](nav_msgs::msg::OccupancyGrid::SharedPtr m) { received = m; });
  auto pub = io->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).reliable().transient_local());
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.origin.orientation.w = 1;
  map.info.resolution = .05;
  map.info.width = 100;
  map.info.height = 80;
  map.info.origin.position.x = -2;
  map.info.origin.position.y = -1;
  map.data.assign(8000, 0);
  map.data[20 * 100 + 40] = 100;
  map.data[21 * 100 + 40] = -1;
  map.data[60 * 100 + 90] = 100;
  pump();
  pub->publish(map);
  pump();
  ASSERT_TRUE(received);
  EXPECT_EQ(received->data[20 * 100 + 40], 0);
  EXPECT_EQ(received->data[21 * 100 + 40], -1);
  EXPECT_EQ(received->data[60 * 100 + 90], 100);
  // 地图扩展和原点变化后，仍以同一世界坐标清理通道。
  map.info.width = 200;
  map.info.height = 160;
  map.info.origin.position.x = -5;
  map.info.origin.position.y = -3;
  map.data.assign(32000, 0);
  map.data[60 * 200 + 100] = 100;
  pub->publish(map);
  pump();
  ASSERT_EQ(received->info.width, 200u);
  EXPECT_EQ(received->info.origin.position.x, -5);
  EXPECT_EQ(received->data[60 * 200 + 100], 0);
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  auto cloud_sub = io->create_subscription<sensor_msgs::msg::PointCloud2>(
      "terrain_map_nav", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::SharedPtr m) { cloud = m; });
  auto cloud_pub = io->create_publisher<sensor_msgs::msg::PointCloud2>(
      "terrain_map", rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2 input;
  input.header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier modifier(input);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(input, "x"), y(input, "y"),
      z(input, "z"), height(input, "intensity");
  for (int i = 0; i < 3; ++i, ++x, ++y, ++z, ++height) {
    *x = i == 2 ? 2. : 0.;
    *y = 0.;
    *z = .5;
    *height = i == 1 ? .1 : .5;
  }
  pump();
  cloud_pub->publish(input);
  pump();
  ASSERT_TRUE(cloud);
  EXPECT_EQ(cloud->width, 2u);
  executor.remove_node(io);
  executor.remove_node(wrapper.get_node_base_interface());
  std::filesystem::remove(filename);
}

TEST_F(RosPassageTest, LiveCostmapResizeAndAnnotatedNarrowCenterline) {
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
  writeConfig(filename, 2., .4);
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
  for (const auto &p : path.poses)
    if (p.pose.position.x > 1.75 && p.pose.position.x < 2.25)
      EXPECT_NEAR(p.pose.position.y, 0, 1e-5);
  grid->setCost(160, 80, 254); // 中线出现新的真实障碍，语义不能将它清除。
  EXPECT_THROW(planner.createPlan(start, goal), nav2_core::PlannerException);
  planner.deactivate();
  planner.cleanup();
  map->cleanup();
  std::filesystem::remove(filename);
}

TEST_F(RosPassageTest, ControllerAdoptionAcknowledgementAndRelease) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "controller_adapter_test");
  auto io = std::make_shared<rclcpp::Node>("controller_adapter_io");
  executor.add_node(node->get_node_base_interface());
  executor.add_node(io);
  lc_passage_nav2::PassageController controller(node, "FollowPath");
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
  EXPECT_EQ(controller.step({0, 0}, 0, 0).phase, "APPROACH");
  auto out = controller.step({1, 0}, 0, 0);
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
  EXPECT_EQ(controller.step({1, 0}, 0, 0).phase, "PASSING");
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
  executor.remove_node(io);
  executor.remove_node(node->get_node_base_interface());
}
} // namespace
