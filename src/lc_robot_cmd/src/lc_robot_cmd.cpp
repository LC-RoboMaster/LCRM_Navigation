#include "../include/lc_robot_cmd/lc_robot_cmd.hpp"

using namespace std::chrono_literals;

LcRobotCmdNode::LcRobotCmdNode(const rclcpp::NodeOptions & options)
: Node("LcRobotCmdNode", options),
  current_state_(RobotState::IDLE),
  enemy_detected_(false)
{
    //预留接口对解裁判系统或自瞄
    hp_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/robot/hp", 10,
      std::bind(&LcRobotCmdNode::hpCallback, this, std::placeholders::_1));

    game_state_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/robot/game_state", 10,
      std::bind(&LcRobotCmdNode::GameStateCallback, this, std::placeholders::_1));

    enemy_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/auto_aim/target_pose", 10,
      std::bind(&LcRobotCmdNode::enemyCallback, this, std::placeholders::_1));

    nav_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot/nav_state", 10);

    //nav2 Action接口
    nav_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    /* -------- FSM Timer -------- */
    fsm_timer_ = create_wall_timer(
      50ms, std::bind(&LcRobotCmdNode::stateUpdate, this));

    /* -------- Home Pose -------- */
    home_pose_.header.frame_id = "map";
    //================
    home_pose_.pose.position.x = 0.38555294275283813;
    home_pose_.pose.position.y = -1.133955717086792;
    home_pose_.pose.orientation.w = 1.0;

    /* -------- gohome_path_point -------- */
    geometry_msgs::msg::PoseStamped go_home_points_temp_;
    go_home_points_temp_.header.frame_id = "map";
    go_home_points_temp_.pose.position.x = 5.454476833343506;
    go_home_points_temp_.pose.position.y = 3.68198299407959;
    go_home_points_temp_.pose.orientation.w = 1.0;
    go_home_points_.push_back(go_home_points_temp_);
    go_home_points_temp_.pose.position.x = 5.634023666381836;
    go_home_points_temp_.pose.position.y = -0.6312038898468018;
    go_home_points_temp_.pose.orientation.w = 1.0;
    go_home_points_.push_back(go_home_points_temp_);
    go_home_points_temp_.pose.position.x = -0.04996013641357422;
    go_home_points_temp_.pose.position.y = -1.1820600032806396;
    go_home_points_temp_.pose.orientation.w = 1.0;
    go_home_points_.push_back(go_home_points_temp_);

    geometry_msgs::msg::PoseStamped go_patrol_temp_;
    go_patrol_temp_.header.frame_id = "map";
    go_patrol_temp_.pose.position.x = 5.634023666381836;
    go_patrol_temp_.pose.position.y = -0.6312038898468018;
    go_patrol_temp_.pose.orientation.w = 1.0;
    go_patrol_points_.push_back(go_patrol_temp_);
    go_patrol_temp_.pose.position.x = 5.454476833343506;
    go_patrol_temp_.pose.position.y = 3.68198299407959;
    go_patrol_temp_.pose.orientation.w = 1.0;
    go_patrol_points_.push_back(go_patrol_temp_);

    /* -------- patrol points -------- */
    geometry_msgs::msg::PoseStamped patrol_points_temp;
    patrol_points_temp.header.frame_id = "map";
    patrol_points_temp.pose.position.x = 3.7750399112701416;
    patrol_points_temp.pose.position.y = 4.593683242797852;
    patrol_points_temp.pose.orientation.w = 1.0;
    patrol_points_.push_back(patrol_points_temp);
    patrol_points_temp.pose.position.x = 2.361743927001953;
    patrol_points_temp.pose.position.y = 3.376885414123535;
    patrol_points_temp.pose.orientation.w = 1.0;
    patrol_points_.push_back(patrol_points_temp);
    patrol_points_temp.pose.position.x = 2.5257375240325928;
    patrol_points_temp.pose.position.y = 4.544539451599121;
    patrol_points_temp.pose.orientation.w = 1.0;
    patrol_points_.push_back(patrol_points_temp);
    patrol_points_temp.pose.position.x = 3.6110732555389404;
    patrol_points_temp.pose.position.y = 3.6965317726135254;
    patrol_points_temp.pose.orientation.w = 1.0;
    patrol_points_.push_back(patrol_points_temp);

    current_state_ = RobotState::IDLE;
    nav_status_ = NavState::IDLE;
    hp_ = 100;

    passage_sub_ = create_subscription<lc_passage_msgs::msg::PassageState>("passage/state",
      rclcpp::QoS(1).reliable().transient_local(), [this](lc_passage_msgs::msg::PassageState::ConstSharedPtr s) {
        passage_locked_ = s->locked;
      });
    RCLCPP_INFO(get_logger(), "LcRobotCmdNode node started.");
}

void LogPrint(){

}

/* ================= FSM ================= */

void LcRobotCmdNode::stateUpdate()
{
  switch (current_state_)
  {
    case RobotState::IDLE:

        if(game_state_ == 4){
            switchState(RobotState::PATROL);
        }else{
            RCLCPP_INFO(this->get_logger(), "game_state = %d", game_state_);
        }
        nav_state_msg.data = 1;
        break;
    case RobotState::PATROL:
        if (hp_ < 160) {
          switchState(RobotState::RETURN_HOME);
        }
        go_home_idx = 0;
        nav_state_msg.data = 1;
        break;
    case RobotState::CHASE:
        if (hp_ < 30) {
          switchState(RobotState::RETURN_HOME);
        }
        break;

    case RobotState::RETURN_HOME:
        if(nav_status_ != NavState::ACTIVE && nav_status_ != NavState::SENDING){
            RCLCPP_INFO(this->get_logger(), "Healing!!!!");
        }
        if (hp_ >= 350) {
          switchState(RobotState::PATROL);
        }
        nav_state_msg.data = 0;
        break;
    }

    if (nav_status_ == NavState::SENDING)
    {
        if ((now() - nav_start_time_).seconds() > 5.0) {
            RCLCPP_ERROR(get_logger(),
                "Nav watchdog timeout, force reset");
            nav_status_ = NavState::FAILED;
            current_state_ = RobotState::IDLE;
            cancelNav();
        }
    }
    nav_state_pub_->publish(nav_state_msg);
}

void LcRobotCmdNode::switchState(RobotState new_state)
{
    // 正常任务切换延迟到出口释放后；急停仍可直接取消 action 并停车。
    if (passage_locked_ && new_state != RobotState::IDLE) return;
    if (new_state == current_state_ && 
        (nav_status_ == NavState::ACTIVE || nav_status_ == NavState::SENDING)) {
        return;
    }

    RCLCPP_INFO(
      get_logger(),
      "State switch: %d -> %d",
      static_cast<int>(current_state_),
      static_cast<int>(new_state));

    cancelNav();

    current_state_ = new_state;

    if (current_state_ == RobotState::PATROL) {
        patrol_index_ = 0;
        go_patrol_idx = 0;
        if(!sendNavGoal(go_patrol_points_[go_patrol_idx])){
            nav_status_ = NavState::FAILED;
        }else{

        };
        
    }
    else if (current_state_ == RobotState::CHASE) {
        sendNavGoal(enemy_pose_);
    }
    else if (current_state_ == RobotState::RETURN_HOME) {
        go_home_idx = 0;
        sendNavGoal(go_home_points_[go_home_idx]);

    }
}

/* ================= Nav2 ================= */

bool LcRobotCmdNode::sendNavGoal(const geometry_msgs::msg::PoseStamped & pose)
{
    if (!nav_client_->wait_for_action_server(500ms)) {
        RCLCPP_WARN(get_logger(), "Nav2 action server not available.");
        current_state_ = RobotState::IDLE;
        return false;
    }

    if(nav_status_ == NavState::ACTIVE || nav_status_ == NavState::SENDING){
        RCLCPP_WARN(get_logger(), "Nav2 is Running, send fail!");
        return false;
    }
    nav_status_ = NavState::SENDING;

    NavigateToPose::Goal goal;
    goal.pose = pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback =
      std::bind(&LcRobotCmdNode::navResultCallback, this, std::placeholders::_1);

    options.feedback_callback =
    std::bind(&LcRobotCmdNode::navFeedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2);

    nav_client_->async_send_goal(goal, options);
    nav_owner_state_ = current_state_;
    nav_start_time_ = now();
    return true;
}

void LcRobotCmdNode::cancelNav()
{
    if (nav_client_ && (nav_status_ == NavState::ACTIVE ||
        nav_status_ == NavState::SENDING)) {

        nav_client_->async_cancel_all_goals();
        nav_status_ = NavState::CANCELED;

    }
}

void LcRobotCmdNode::navFeedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    if(nav_status_ == NavState::SENDING){
        nav_status_ = NavState::ACTIVE;
    }
    // 当前机器人位置
    auto & pose = feedback->current_pose.pose.position;

    // 剩余距离
    double dist = feedback->distance_remaining;

    // 导航耗时
    rclcpp::Duration time = feedback->navigation_time;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Nav active: dist=%.2f, x=%.2f y=%.2f",
        dist, pose.x, pose.y
    );
}

void LcRobotCmdNode::navResultCallback(
  const GoalHandleNav::WrappedResult & result)
{
    switch(result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation succeeded.");
        nav_status_ = NavState::IDLE;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Navigation Canceled.");
        nav_status_ = NavState::CANCELED;
        // current_state_ = RobotState::IDLE;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(get_logger(), "Navigation Aborted.");
        nav_status_ = NavState::ABORTED;
        // current_state_ = RobotState::IDLE;
        break;
    default:
        RCLCPP_INFO(get_logger(), "Navigation Falid.");
        nav_status_ = NavState::FAILED;
        // current_state_ = RobotState::IDLE;
        break;
    }

    if (current_state_ == RobotState::PATROL) {
        UpdatePatrol(result);
    }
    if(current_state_ == RobotState::RETURN_HOME){
        UpdateReturingHome(result);
    }

}
void LcRobotCmdNode::UpdateReturingHome(GoalHandleNav::WrappedResult result){
    if(nav_status_ == NavState::IDLE){
        if(go_home_idx + 1 < go_home_points_.size()){
            go_home_idx++;
            sendNavGoal(go_home_points_[go_home_idx]);
        }
    }else if(nav_status_ != NavState::IDLE){
        RCLCPP_WARN(get_logger(), "go home nav failed, retrying...");
        sendNavGoal(go_home_points_[go_home_idx]);
    }
}

void LcRobotCmdNode::UpdatePatrol(GoalHandleNav::WrappedResult result){
    if (nav_status_ == NavState::IDLE) {
        if(go_patrol_idx + 1 < go_patrol_points_.size()){
            go_patrol_idx++;
            sendNavGoal(go_patrol_points_[go_patrol_idx]);
            nav_state_msg.data = 1;
        }
        // 切换到下一个点
        patrol_index_ = (patrol_index_ + 1) % patrol_points_.size();
        // patrol_index_++;
        sendNavGoal(patrol_points_[patrol_index_]);
        // if(patrol_index_ >= patrol_points_.size()){
        //     // switchState(RobotState::RETURN_HOME);
        //     // patrol_count = 0;
        // }else{
        // }
        nav_state_msg.data = 1;
        patrol_count++;
    } else if(nav_status_ == NavState::FAILED){
        RCLCPP_WARN(get_logger(), "Patrol nav failed, retrying...");
        if(go_patrol_idx + 1 < go_patrol_points_.size()){
            sendNavGoal(go_patrol_points_[go_patrol_idx]);
        }else{
            sendNavGoal(patrol_points_[patrol_index_]);
        }
        patrol_count++;
        nav_state_msg.data = 1;
    }
}

/* ================= Callbacks ================= */

void LcRobotCmdNode::hpCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    hp_ = msg->data;
}

void LcRobotCmdNode::GameStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    game_state_ = msg->data;
}

void LcRobotCmdNode::enemyCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    enemy_pose_ = *msg;
    enemy_detected_ = true;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto lc_robot_cmd = std::make_shared<LcRobotCmdNode>();
    rclcpp::spin(lc_robot_cmd);
    rclcpp::shutdown();
    return 0;
}
