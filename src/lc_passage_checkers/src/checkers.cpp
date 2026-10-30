#include <chrono>
#include <cmath>
#include <lc_passage_msgs/msg/passage_state.hpp>
#include <mutex>
#include <nav2_core/goal_checker.hpp>
#include <nav2_core/progress_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lc_passage_checkers
{
    class StateReader
    {
    protected:
        std::mutex mutex_;
        lc_passage_msgs::msg::PassageState state_;
        std::chrono::steady_clock::time_point received_{};
        rclcpp::Subscription<lc_passage_msgs::msg::PassageState>::SharedPtr sub_;

        void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
        {
            sub_ = node->create_subscription<lc_passage_msgs::msg::PassageState>(
                "passage/state", rclcpp::QoS(1).reliable().transient_local(),
                [this](lc_passage_msgs::msg::PassageState::ConstSharedPtr s)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_ = *s;
                    received_ = std::chrono::steady_clock::now();
                });
        }
    };

    // 变形等待豁免进度超时；实际障碍阻挡和变形故障不享受豁免。
    class PassageProgressChecker : public nav2_core::ProgressChecker,
                                   private StateReader
    {
        double radius_ = .1, allowance_ = 10.;
        bool baseline_ = false;
        geometry_msgs::msg::Pose base_;
        std::chrono::steady_clock::time_point moved_;

    public:
        void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                        const std::string& name) override
        {
            auto node = parent.lock();
            subscribe(node);
            nav2_util::declare_parameter_if_not_declared(
                node, name + ".required_movement_radius", rclcpp::ParameterValue(.1));
            nav2_util::declare_parameter_if_not_declared(
                node, name + ".movement_time_allowance", rclcpp::ParameterValue(10.));
            node->get_parameter(name + ".required_movement_radius", radius_);
            node->get_parameter(name + ".movement_time_allowance", allowance_);
        }

        void reset() override { baseline_ = false; }

        bool check(geometry_msgs::msg::PoseStamped& pose) override
        {
            auto now = std::chrono::steady_clock::now();
            bool wait;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                wait = state_.intentional_wait && !state_.fault &&
                    std::chrono::duration<double>(now - received_).count() < .5;
            }
            if (wait || !baseline_ ||
                std::hypot(pose.pose.position.x - base_.position.x,
                           pose.pose.position.y - base_.position.y) > radius_)
            {
                baseline_ = true;
                base_ = pose.pose;
                moved_ = now;
                return true;
            }
            return std::chrono::duration<double>(now - moved_).count() <= allowance_;
        }
    };

    // 到达最终坐标也必须等待出口释放和恢复确认，不能在洞中提前结束 FollowPath。
    class PassageGoalChecker : public nav2_core::GoalChecker, private StateReader
    {
        double xy_ = .15, yaw_ = 6.28;

    public:
        void
        initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   const std::string& name,
                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
        {
            auto node = parent.lock();
            subscribe(node);
            nav2_util::declare_parameter_if_not_declared(
                node, name + ".xy_goal_tolerance", rclcpp::ParameterValue(.15));
            nav2_util::declare_parameter_if_not_declared(
                node, name + ".yaw_goal_tolerance", rclcpp::ParameterValue(6.28));
            node->get_parameter(name + ".xy_goal_tolerance", xy_);
            node->get_parameter(name + ".yaw_goal_tolerance", yaw_);
        }

        void reset() override
        {
        }

        bool isGoalReached(const geometry_msgs::msg::Pose& query,
                           const geometry_msgs::msg::Pose& goal,
                           const geometry_msgs::msg::Twist&) override
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_.locked || state_.folded || state_.fault ||
                    state_.phase == "WAIT_METADATA")
                    return false;
            }
            return std::hypot(query.position.x - goal.position.x,
                              query.position.y - goal.position.y) <= xy_ &&
                std::abs(std::remainder(tf2::getYaw(query.orientation) -
                                        tf2::getYaw(goal.orientation),
                                        2 * M_PI)) <= yaw_;
        }

        bool getTolerances(geometry_msgs::msg::Pose& p,
                           geometry_msgs::msg::Twist& v) override
        {
            p = geometry_msgs::msg::Pose{};
            v = geometry_msgs::msg::Twist{};
            p.position.x = p.position.y = xy_;
            p.orientation.z = std::sin(yaw_ / 2);
            p.orientation.w = std::cos(yaw_ / 2);
            return true;
        }
    };
} // namespace lc_passage_checkers
PLUGINLIB_EXPORT_CLASS(lc_passage_checkers::PassageProgressChecker,
                       nav2_core::ProgressChecker)

PLUGINLIB_EXPORT_CLASS(lc_passage_checkers::PassageGoalChecker,
                       nav2_core::GoalChecker)
