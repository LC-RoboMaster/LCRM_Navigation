#include "lc_passage_nav2/passage_config.hpp"
#include "lc_passage_nav2/passage_markers.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lc_passage_nav2
{
    // 保留 /map 给定位、保存地图使用。只有导航的静态层订阅 /passage_map。
    // 静态层与实时观测分开处理，绝不在合并后的 master costmap 上擦除矩形。
    class PassageMapNode : public rclcpp::Node
    {
    public:
        explicit PassageMapNode(const rclcpp::NodeOptions& options)
            : Node("passage_map", options), tf_(get_clock()), listener_(tf_)
        {
            const auto passage_file = declare_parameter<std::string>("passage_file", "");
            config_ = loadPassages(passage_file);
            auto qos = rclcpp::QoS(1).reliable().transient_local();
            marker_pub_ = create_publisher<MarkerArray>("passage/markers", qos);
            active_sub_ = create_subscription<lc_passage_msgs::msg::PassagePlan>(
                "passage/active_plan", qos,
                [this](lc_passage_msgs::msg::PassagePlan::ConstSharedPtr p) {
                    active_plan_ = *p;
                    publishMarkers();
                });
            state_sub_ = create_subscription<lc_passage_msgs::msg::PassageState>(
                "passage/state", qos,
                [this](lc_passage_msgs::msg::PassageState::ConstSharedPtr s) {
                    state_ = *s;
                });
            // Durable snapshot plus periodic refresh supports RViz reconnects.
            marker_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publishMarkers(); });
            publishMarkers();
            map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("passage_map", qos);
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_nav", rclcpp::SensorDataQoS());
            map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr input)
                {
                    auto map = *input;
                    if (map.data.size() != size_t(map.info.width) * map.info.height)
                    {
                        RCLCPP_ERROR(get_logger(), "Malformed occupancy map");
                        return;
                    }
                    if (!config_.passages.empty() &&
                        map.header.frame_id != config_.frame)
                    {
                        RCLCPP_ERROR(get_logger(), "Passage/map frame mismatch");
                        return;
                    }
                    double yaw = tf2::getYaw(map.info.origin.orientation), c = std::cos(yaw), s = std::sin(yaw);
                    for (unsigned y = 0; y < map.info.height; ++y)
                        for (unsigned x = 0; x < map.info.width; ++x)
                        {
                            auto& cell = map.data[size_t(y) * map.info.width + x];
                            if (cell == 0)
                                continue;
                            double lx = (x + .5) * map.info.resolution,
                                   ly = (y + .5) * map.info.resolution;
                            Point p{
                                map.info.origin.position.x + c * lx - s * ly,
                                map.info.origin.position.y + s * lx + c * ly
                            };
                            for (const auto& gate : config_.passages)
                                if (gate.enabled && gate.clear_static && gate.contains(p) &&
                                    (cell > 0 || gate.clear_unknown))
                                {
                                    cell = 0;
                                    break;
                                }
                        }
                    map_pub_->publish(map);
                });
            cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "terrain_map", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
                {
                    filter(*input);
                });
            RCLCPP_INFO(
                get_logger(),
                "Passage map ready: %zu annotations, file='%s', RViz=passage/markers",
                config_.passages.size(), passage_file.c_str());
        }

    private:
        PassageFile config_;
        lc_passage_msgs::msg::PassagePlan active_plan_;
        lc_passage_msgs::msg::PassageState state_;
        rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<lc_passage_msgs::msg::PassagePlan>::SharedPtr active_sub_;
        rclcpp::Subscription<lc_passage_msgs::msg::PassageState>::SharedPtr state_sub_;
        rclcpp::TimerBase::SharedPtr marker_timer_;
        void publishMarkers() { marker_pub_->publish(passageMarkers(config_, active_plan_, state_)); }
        tf2_ros::Buffer tf_;
        tf2_ros::TransformListener listener_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

        void filter(const sensor_msgs::msg::PointCloud2& input)
        {
            if (config_.passages.empty())
            {
                cloud_pub_->publish(input);
                return;
            }
            try
            {
                // terrain_analysis 的 intensity 是相对地面高度(米)
                auto tf = tf_.lookupTransform(config_.frame, input.header.frame_id, rclcpp::Time(input.header.stamp), rclcpp::Duration::from_seconds(.05));
                sensor_msgs::PointCloud2ConstIterator<float> x(input, "x"), y(input, "y"),
                                                             z(input, "z"), height(input, "intensity");
                auto output = input;
                output.data.clear();
                output.data.reserve(input.data.size());
                output.height = 1;
                output.width = 0;
                for (size_t i = 0; i < size_t(input.width) * input.height;
                     ++i, ++x, ++y, ++z, ++height)
                {
                    geometry_msgs::msg::PointStamped p, q;
                    p.point.x = *x;
                    p.point.y = *y;
                    p.point.z = *z;
                    tf2::doTransform(p, q, tf);
                    if (isBeam(config_, {q.point.x, q.point.y}, *height))
                        continue;
                    const size_t offset = (i / input.width) * input.row_step +
                        (i % input.width) * input.point_step;
                    output.data.insert(output.data.end(), input.data.begin() + offset,
                                       input.data.begin() + offset + input.point_step);
                    ++output.width;
                }
                output.row_step = output.width * output.point_step;
                cloud_pub_->publish(output);
            }
            catch (const std::exception& e)
            {
                // TF 暂不可用时保留原始观测，不能因为过滤失败而清空真实障碍。
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                     "Passage cloud filter bypassed: %s", e.what());
                cloud_pub_->publish(input);
            }
        }
    };
} // namespace lc_passage_nav2
RCLCPP_COMPONENTS_REGISTER_NODE(lc_passage_nav2::PassageMapNode)
