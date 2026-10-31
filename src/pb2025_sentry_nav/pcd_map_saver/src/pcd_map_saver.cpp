// Copyright 2026 HBUT RM Vision
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pcd_map_saver
{

// 累积一个已经处于 odom/map 系的点云话题（默认 loam_interface 的 registered_scan），
// 退出时写成二进制 PCD。这样先验点云与 map_saver 保存的 pgm 同帧，
// small_gicp 重定位不需要再做任何预旋转/预平移。
class PcdMapSaver : public rclcpp::Node
{
public:
  explicit PcdMapSaver(const rclcpp::NodeOptions & options)
  : Node("pcd_map_saver", options)
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "registered_scan");
    output_file_ = this->declare_parameter<std::string>("output_file", "scans.pcd");
    expected_frame_ = this->declare_parameter<std::string>("expected_frame", "odom");

    if (!output_file_.empty()) {
      output_file_ = std::filesystem::absolute(output_file_).lexically_normal().string();
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&PcdMapSaver::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Accumulating '%s' (expected frame '%s') -> '%s', saved on shutdown.",
      cloud_topic_.c_str(), expected_frame_.c_str(), output_file_.c_str());
  }

  ~PcdMapSaver() override { save(); }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    if (!expected_frame_.empty() && msg->header.frame_id != expected_frame_ && !frame_warned_) {
      frame_warned_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Cloud frame is '%s', expected '%s'. The saved PCD will not share the 2D map frame!",
        msg->header.frame_id.c_str(), expected_frame_.c_str());
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }
    accumulated_ += cloud;
    ++cloud_count_;
    if (cloud_count_ % 100 == 0) {
      RCLCPP_INFO(
        get_logger(), "Accumulated %zu points from %zu clouds.", accumulated_.size(), cloud_count_);
    }
  }

  void save() noexcept
  {
    if (accumulated_.empty()) {
      RCLCPP_WARN(get_logger(), "No points accumulated, nothing saved.");
      return;
    }
    if (output_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "output_file is empty, nothing saved.");
      return;
    }
    try {
      const std::filesystem::path path(output_file_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      if (pcl::io::savePCDFileBinary(output_file_, accumulated_) != 0) {
        RCLCPP_ERROR(get_logger(), "Failed to write '%s'.", output_file_.c_str());
        return;
      }
      RCLCPP_INFO(
        get_logger(), "Saved %zu points to '%s'.", accumulated_.size(), output_file_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to write '%s': %s", output_file_.c_str(), ex.what());
    }
  }

  std::string cloud_topic_;
  std::string output_file_;
  std::string expected_frame_;
  bool frame_warned_{false};
  size_t cloud_count_{0};
  pcl::PointCloud<pcl::PointXYZI> accumulated_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace pcd_map_saver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<pcd_map_saver::PcdMapSaver>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
