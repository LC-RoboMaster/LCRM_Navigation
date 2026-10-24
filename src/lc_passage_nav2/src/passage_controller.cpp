#include "lc_passage_nav2/passage_controller.hpp"
#include <nav2_util/node_utils.hpp>

namespace lc_passage_nav2
{
    double PassageController::steadyNow()
    {
        return std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    namespace
    {
        bool samePose(const geometry_msgs::msg::PoseStamped& a,
                      const geometry_msgs::msg::PoseStamped& b)
        {
            return std::abs(a.pose.position.x - b.pose.position.x) < 1e-7 &&
                std::abs(a.pose.position.y - b.pose.position.y) < 1e-7 &&
                std::abs(a.pose.orientation.z - b.pose.orientation.z) < 1e-7 &&
                std::abs(a.pose.orientation.w - b.pose.orientation.w) < 1e-7;
        }

        ExecutionPlan convert(const lc_passage_msgs::msg::PassagePlan& m)
        {
            ExecutionPlan p;
            p.id = m.plan_id;
            p.arcs = m.arc_lengths;
            for (const auto& v : m.path.poses)
                p.points.push_back({v.pose.position.x, v.pose.position.y});
            for (const auto& v : m.traversals)
                p.traversals.push_back({
                    v.passage_id, v.direction, v.mode, v.prepare_s,
                    v.enter_s, v.exit_s, v.release_s, v.speed,
                    v.fold_hold_seconds, v.restore_hold_seconds
                });
            p.validate();
            return p;
        }
    } // namespace
    PassageController::PassageController(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        const std::string& prefix)
        : node_(node)
    {
        nav2_util::declare_parameter_if_not_declared(
            node, prefix + ".passage_file", rclcpp::ParameterValue(std::string("")));
        config_ =
            loadPassages(node->get_parameter(prefix + ".passage_file").as_string());
        auto param = [&](const std::string& key, double fallback, double& v)
        {
            nav2_util::declare_parameter_if_not_declared(
                node, prefix + "." + key, rclcpp::ParameterValue(fallback));
            node->get_parameter(prefix + "." + key, v);
        };
        param("passage_stop_tolerance", .04, execution_.stop_tolerance);
        param("passage_action_timeout", 8., execution_.action_timeout);
        param("passage_deceleration", .8, execution_.deceleration);
        if (execution_.stop_tolerance <= 0 || execution_.action_timeout <= 0 ||
            execution_.deceleration <= 0)
            throw std::runtime_error("Invalid passage controller parameters");
        auto qos = rclcpp::QoS(1).reliable().transient_local();
        state_pub_ = node->create_publisher<lc_passage_msgs::msg::PassageState>(
            "passage/state", qos);
        active_pub_ = node->create_publisher<lc_passage_msgs::msg::PassagePlan>(
            "passage/active_plan", qos);
        command_pub_ =
            node->create_publisher<lc_passage_msgs::msg::DeformationCommand>(
                "passage/deformation_command", qos);
        // 此对象自己管理发布器激活，停用跟踪后仍需公布“保持变形并停车”的锁定状态。
        state_pub_->on_activate();
        active_pub_->on_activate();
        command_pub_->on_activate();
        plan_sub_ = node->create_subscription<lc_passage_msgs::msg::PassagePlan>(
            "passage/plan", rclcpp::QoS(32).reliable().transient_local(),
            [this](lc_passage_msgs::msg::PassagePlan::ConstSharedPtr m)
            {
                std::lock_guard<std::mutex> guard(mutex_);
                try
                {
                    convert(*m);
                    candidates_.push_back(*m);
                    if (candidates_.size() > 64)
                        candidates_.pop_front();
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Invalid passage metadata: %s",
                                 e.what());
                }
            });
        feedback_sub_ =
            node->create_subscription<lc_passage_msgs::msg::DeformationFeedback>(
                "passage/deformation_feedback", rclcpp::QoS(10).reliable(),
                [this](lc_passage_msgs::msg::DeformationFeedback::ConstSharedPtr m)
                {
                    std::lock_guard<std::mutex> guard(mutex_);
                    execution_.feedback(m->request_id, m->mode, m->completed, m->fault,
                                        steadyNow());
                });
        watchdog_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]
        {
            std::lock_guard<std::mutex> guard(mutex_);
            if (execution_.output().locked &&
                (suspended_ || steadyNow() - last_step_ > .5))
            {
                auto out = execution_.output();
                out.stop = true;
                out.intentional_wait = false;
                publish(out);
            }
        });
    }

    bool PassageController::insideSpecial(Point p) const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!execution_.hasPlan())
            return false;
        // 仅在已提交的固定中线上使用真实占据检查，其他路径仍采用正常膨胀代价。
        for (const auto& t : execution_.plan().traversals)
        {
            auto a = execution_.plan().at(t.enter), b = execution_.plan().at(t.release);
            double dx = b.x - a.x, dy = b.y - a.y;
            double u = ((p.x - a.x) * dx + (p.y - a.y) * dy) /
                std::max(1e-12, dx * dx + dy * dy);
            if (u >= 0 && u <= 1 && distance(p, {a.x + u * dx, a.y + u * dy}) <= .06)
                return true;
        }
        return false;
    }

    bool PassageController::locked() const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return execution_.output().locked;
    }

    std::string PassageController::frame() const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return pending_valid_ ? pending_.header.frame_id : active_.header.frame_id;
    }

    void PassageController::setPlan(const nav_msgs::msg::Path& path)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        suspended_ = false;
        last_step_ = steadyNow();
        if (execution_.output().locked)
            return;
        pending_ = path;
        pending_valid_ = true;
        pending_since_ = steadyNow();
        adopt();
    }

    bool PassageController::adopt()
    {
        if (!pending_valid_)
            return execution_.hasPlan();
        // 支持 ComputePathThroughPoses 拼接多段路径：每段必须与候选路径逐点匹配。
        lc_passage_msgs::msg::PassagePlan joined;
        joined.header = pending_.header;
        joined.path = pending_;
        size_t offset = 0;
        double arc_offset = 0;
        while (offset < pending_.poses.size())
        {
            const lc_passage_msgs::msg::PassagePlan* found = nullptr;
            for (auto it = candidates_.rbegin(); it != candidates_.rend(); ++it)
            {
                if (it->path.header.frame_id != pending_.header.frame_id ||
                    it->path.poses.empty() ||
                    it->path.poses.size() > pending_.poses.size() - offset)
                    continue;
                bool matches = true;
                for (size_t i = 0; i < it->path.poses.size(); ++i)
                    if (!samePose(it->path.poses[i], pending_.poses[offset + i]))
                    {
                        matches = false;
                        break;
                    }
                if (matches)
                {
                    found = &*it;
                    break;
                }
            }
            if (!found)
                return false;
            if (offset && distance({
                                       pending_.poses[offset - 1].pose.position.x,
                                       pending_.poses[offset - 1].pose.position.y
                                   },
                                   {
                                       pending_.poses[offset].pose.position.x,
                                       pending_.poses[offset].pose.position.y
                                   }) > 1e-5)
                return false;
            if (!joined.plan_id.empty())
                joined.plan_id += "+";
            joined.plan_id += found->plan_id;
            for (auto a : found->arc_lengths)
                joined.arc_lengths.push_back(arc_offset + a);
            for (auto t : found->traversals)
            {
                t.prepare_s += arc_offset;
                t.enter_s += arc_offset;
                t.exit_s += arc_offset;
                t.release_s += arc_offset;
                joined.traversals.push_back(t);
            }
            arc_offset += found->arc_lengths.back();
            offset += found->path.poses.size();
        }
        if (joined.path.poses.empty())
            return false;
        if (!execution_.bind(convert(joined)))
            return false;
        active_ = std::move(joined);
        pending_valid_ = false;
        active_pub_->publish(active_);
        return true;
    }

    ExecutionOutput PassageController::step(Point robot, double speed,
                                            double angular_speed)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        last_step_ = steadyNow();
        suspended_ = false;
        try
        {
            if (!adopt())
            {
                ExecutionOutput out;
                out.stop = true;
                out.phase = "WAIT_METADATA";
                out.intentional_wait = last_step_ - pending_since_ < 1.;
                if (!out.intentional_wait)
                {
                    out.fault = true;
                    out.phase = "METADATA_FAULT";
                }
                publish(out);
                return out;
            }
            auto out = execution_.tick(robot, speed, angular_speed, last_step_);
            publish(out);
            return out;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Passage execution failed: %s", e.what());
            execution_.fail();
            publish(execution_.output());
            return execution_.output();
        }
    }

    Point PassageController::target(double lookahead) const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        return execution_.plan().at(std::min(execution_.output().progress + lookahead,
                                             execution_.output().target_limit));
    }

    void PassageController::hold()
    {
        std::lock_guard<std::mutex> guard(mutex_);
        auto out = execution_.output();
        out.stop = true;
        out.intentional_wait = false;
        out.phase = "BLOCKED";
        publish(out);
    }

    void PassageController::suspend()
    {
        std::lock_guard<std::mutex> guard(mutex_);
        suspended_ = true;
        if (execution_.output().locked)
        {
            auto out = execution_.output();
            out.stop = true;
            out.intentional_wait = false;
            publish(out);
        }
    }

    void PassageController::publish(const ExecutionOutput& out)
    {
        lc_passage_msgs::msg::PassageState s;
        s.header.stamp = node_->now();
        s.header.frame_id = active_.header.frame_id;
        s.plan_id = active_.plan_id;
        s.passage_id = out.passage_id;
        s.phase = out.phase;
        s.progress = out.progress;
        s.locked = out.locked;
        s.folded = out.folded;
        s.stop = out.stop;
        s.fault = out.fault;
        s.intentional_wait = out.intentional_wait;
        state_pub_->publish(s);
        if (!out.request_id.empty())
        {
            lc_passage_msgs::msg::DeformationCommand c;
            c.header = s.header;
            c.request_id = out.request_id;
            c.plan_id = s.plan_id;
            c.passage_id = out.passage_id;
            c.mode = out.request_mode;
            command_pub_->publish(c);
        }
    }
} // namespace lc_passage_nav2
