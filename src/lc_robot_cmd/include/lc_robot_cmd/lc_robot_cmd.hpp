#ifndef LC_ROBOT_CMD_HPP_
#define LC_ROBOT_CMD_HPP_

#include <memory>
#include "lc_passage_msgs/msg/passage_state.hpp"
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

class LcRobotCmdNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit LcRobotCmdNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool passage_locked_ = false;
  rclcpp::Subscription<lc_passage_msgs::msg::PassageState>::SharedPtr passage_sub_;
  /* ========== FSM ========== */
    enum class RobotState
    {
      IDLE,       //待机状态
      PATROL,     //巡逻状态
      CHASE,      //追击状态
      RETURN_HOME //回家状态
    };

    enum class NavState{
        IDLE,       //服务待机
        SENDING,    //导航点发送中
        ACTIVE,     //导航活动中
        CANCELED,   //导航取消
        ABORTED,    //放弃
        FAILED,     //导航失败
    };

    void stateUpdate();                       // FSM 主循环
    void switchState(RobotState new_state);   //行为逻辑

    /* ========== Nav2 Action ========== */
    bool sendNavGoal(const geometry_msgs::msg::PoseStamped & pose);
    void cancelNav();
    void navFeedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /* ========== Callbacks ========== */
    void hpCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void GameStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void enemyCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    //导航动作结果回调函数
    void navResultCallback(const GoalHandleNav::WrappedResult & result);

    //巡回逻辑/如果有多点导航参照这个模板在导航结束回调函数中调用
    void UpdatePatrol(GoalHandleNav::WrappedResult result);
    void UpdateReturingHome(GoalHandleNav::WrappedResult result);


private:
    void LogPrint();
private:

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hp_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr game_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nav_state_pub_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    //watch dog
    rclcpp::Time nav_start_time_;

private:

    RobotState current_state_;
    NavState nav_status_;
    RobotState nav_owner_state_;
    std_msgs::msg::Int32 nav_state_msg;

    int hp_;
    int game_state_;
    bool enemy_detected_;

    geometry_msgs::msg::PoseStamped enemy_pose_;
    geometry_msgs::msg::PoseStamped home_pose_;

    std::vector<geometry_msgs::msg::PoseStamped> patrol_points_;
    std::vector<geometry_msgs::msg::PoseStamped> go_patrol_points_;
    std::vector<geometry_msgs::msg::PoseStamped> go_home_points_;

    int go_home_idx = 0;
    int go_patrol_idx = 0;
    int patrol_index_ = 0;
    bool nav_active_ = false;

private:
    int home_waiting_count = 0;
    int patrol_count = 0;
    
};

#endif
