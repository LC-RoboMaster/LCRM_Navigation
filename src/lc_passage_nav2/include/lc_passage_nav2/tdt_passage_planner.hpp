#pragma once
#include "lc_passage_common/passage_config.hpp"
#include <lc_passage_msgs/msg/passage_plan.hpp>
#include <lc_passage_msgs/msg/passage_state.hpp>
#include <mutex>
#include <nav2_core/global_planner.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <string>
#include <tdt_passage/semantic_planner.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lc_passage_nav2
{
    using lc_passage_common::PassageFile;
    class TdtPassagePlanner : public nav2_core::GlobalPlanner
    {
    public:
        TdtPassagePlanner() = default;
        ~TdtPassagePlanner() override = default;
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;
        nav_msgs::msg::Path
        createPlan(const geometry_msgs::msg::PoseStamped& start,
                   const geometry_msgs::msg::PoseStamped& goal) override;

    private:
        std::mutex mutex_;
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp_lifecycle::LifecyclePublisher<
            lc_passage_msgs::msg::PassagePlan>::SharedPtr plan_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<lc_passage_msgs::msg::PassageState>::SharedPtr
        state_sub_;
        rclcpp::Subscription<lc_passage_msgs::msg::PassagePlan>::SharedPtr
        active_sub_;
        std::mutex state_mutex_;
        bool locked_ = false;
        lc_passage_msgs::msg::PassagePlan committed_;
        PassageFile config_;
        std::string name_, global_frame_, project_file_;
        std::string last_diagnostic_state_;
        tdt_passage::Project project_;
        bool configured_ = false, active_ = false, unknown_is_obstacle_ = true;
    };
} // namespace lc_passage_nav2
