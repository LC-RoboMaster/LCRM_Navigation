#pragma once
#include "pb_omni_pid_pursuit_controller/execution.hpp"
#include "lc_passage_common/passage_config.hpp"
#include <chrono>
#include <deque>
#include <lc_passage_msgs/msg/deformation_command.hpp>
#include <lc_passage_msgs/msg/deformation_feedback.hpp>
#include <lc_passage_msgs/msg/passage_plan.hpp>
#include <lc_passage_msgs/msg/passage_state.hpp>
#include <mutex>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace pb_omni_pid_pursuit_controller
{
    using lc_passage_common::PassageFile;
    // 此类由 PID 控制器持有。没有独立的“变形管理节点”，也不在标量 PID 中做状态判断。
    class PassageController
    {
    public:
        explicit PassageController(
            const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
            const std::string& prefix);
        bool locked() const;
        bool insideSpecial(Point p) const;
        void setPlan(const nav_msgs::msg::Path& path);
        ExecutionOutput step(Point robot, double speed, double angular_speed);
        Point target(double lookahead) const;
        std::string frame() const;
        void suspend();
        void hold();

    private:
        mutable std::mutex mutex_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        Execution execution_;
        PassageFile config_;
        std::deque<lc_passage_msgs::msg::PassagePlan> candidates_;
        nav_msgs::msg::Path pending_;
        lc_passage_msgs::msg::PassagePlan active_;
        bool pending_valid_ = false, suspended_ = true;
        double pending_since_ = 0, last_step_ = 0;
        rclcpp::Subscription<lc_passage_msgs::msg::PassagePlan>::SharedPtr plan_sub_;
        rclcpp::Subscription<lc_passage_msgs::msg::DeformationFeedback>::SharedPtr
        feedback_sub_;
        rclcpp_lifecycle::LifecyclePublisher<
            lc_passage_msgs::msg::PassageState>::SharedPtr state_pub_;
        rclcpp_lifecycle::LifecyclePublisher<
            lc_passage_msgs::msg::PassagePlan>::SharedPtr active_pub_;
        rclcpp_lifecycle::LifecyclePublisher<
            lc_passage_msgs::msg::DeformationCommand>::SharedPtr command_pub_;
        rclcpp::TimerBase::SharedPtr watchdog_;
        static double steadyNow();
        bool adopt();
        void publish(const ExecutionOutput& output);
    };
} // namespace pb_omni_pid_pursuit_controller
