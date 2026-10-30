#include "lc_passage_map/passage_markers.hpp"
#include <nlohmann/json.hpp>
#include <class_loader/class_loader.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
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
  visualization_msgs::msg::MarkerArray::SharedPtr markers;
  auto marker_sub = io->create_subscription<visualization_msgs::msg::MarkerArray>(
      "passage/markers", rclcpp::QoS(1).reliable().transient_local(),
      [&](visualization_msgs::msg::MarkerArray::SharedPtr m) { markers = m; });
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
  ASSERT_TRUE(markers); // subscriber joined AFTER component construction
  EXPECT_GT(markers->markers.size(), 5u);
  bool has_a = false;
  for (const auto &m : markers->markers)
    if (m.ns == "endpoints" && std::abs(m.pose.position.x+1.) < 1e-6) has_a=true;
  EXPECT_TRUE(has_a);
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

TEST_F(RosPassageTest, ExplicitStaticUnknownOverrideIsLimitedToTheGate) {
  const auto filename=file("static_unknown");
  writeConfig(filename,0.,1.2,true);
  class_loader::ClassLoader loader(PASSAGE_COMPONENT_LIBRARY);
  auto names=loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  auto factory=loader.createInstance<rclcpp_components::NodeFactory>(names.front());
  auto wrapper=factory->create_node_instance(rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("passage_file",filename)}));
  auto io=std::make_shared<rclcpp::Node>("static_unknown_test_io");
  executor.add_node(wrapper.get_node_base_interface()); executor.add_node(io);
  nav_msgs::msg::OccupancyGrid::SharedPtr received;
  auto sub=io->create_subscription<nav_msgs::msg::OccupancyGrid>("passage_map",
      rclcpp::QoS(1).reliable().transient_local(),
      [&](nav_msgs::msg::OccupancyGrid::SharedPtr m){received=m;});
  auto pub=io->create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(1).reliable().transient_local());
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id="map";m.info.resolution=.1;m.info.width=40;m.info.height=20;
  m.info.origin.position.x=-2.;m.info.origin.position.y=-1.;m.info.origin.orientation.w=1.;
  m.data.assign(800,-1);
  pump();pub->publish(m);pump();ASSERT_TRUE(received);
  EXPECT_EQ(received->data[10*40+20],0);
  EXPECT_EQ(received->data[10*40+35],-1);
  executor.remove_node(io);executor.remove_node(wrapper.get_node_base_interface());
  std::filesystem::remove(filename);
}

TEST_F(RosPassageTest, MarkersPreserveWorldGeometryAndActualPreparePoint) {
  lc_passage_common::PassageFile config;
  lc_passage_common::PassageConfig g;
  g.id="rotated"; g.center={7.9,-4.1}; g.yaw=.4;
  g.a={6.9,-4.5}; g.b={8.9,-3.7}; config.passages={g};
  lc_passage_msgs::msg::PassagePlan p;
  p.header.frame_id=p.path.header.frame_id="map";
  p.arc_lengths={0.,2.,5.};
  for (const auto &xy : std::array<lc_passage_common::Point,3>{{{1,1},{1,3},{4,3}}}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x=xy.x; pose.pose.position.y=xy.y;
    p.path.poses.push_back(pose);
  }
  lc_passage_msgs::msg::PassageTraversal t;
  t.passage_id=g.id; t.prepare_s=1.; t.enter_s=2.; t.exit_s=4.; t.release_s=5.;
  p.traversals={t};
  const auto markers=lc_passage_map::passageMarkers(config,p);
  bool geometry=false,prepare=false;
  for (const auto &m : markers.markers) {
    EXPECT_EQ(m.header.frame_id,"map");
    if (m.ns=="regions") {
      geometry=true; EXPECT_DOUBLE_EQ(m.pose.position.x,7.9);
      EXPECT_DOUBLE_EQ(m.pose.position.y,-4.1);
      EXPECT_NEAR(m.pose.orientation.z,std::sin(.2),1e-9);
    }
    if (m.ns=="event_labels" && m.text.find("PREPARE")!=std::string::npos) {
      prepare=true; EXPECT_DOUBLE_EQ(m.pose.position.x,1.);
      EXPECT_DOUBLE_EQ(m.pose.position.y,2.);
    }
  }
  EXPECT_TRUE(geometry); EXPECT_TRUE(prepare);
}

} // namespace
