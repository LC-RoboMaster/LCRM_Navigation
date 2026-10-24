#pragma once
// 无 ROS 的控制状态机，便于离线回放、单元测试及移植到其他控制器。
#include "lc_passage_nav2/passage_config.hpp"
#include <algorithm>
#include <limits>
#include <utility>

namespace lc_passage_nav2
{
    struct Traversal
    {
        std::string id, direction, mode = "folded";
        double prepare = 0, enter = 0, exit = 0, release = 0, speed = .3,
               fold_hold = .3, restore_hold = .3;
    };

    struct ExecutionPlan
    {
        std::string id;
        std::vector<Point> points;
        std::vector<double> arcs;
        std::vector<Traversal> traversals;

        void validate() const
        {
            if (id.empty() || points.empty() || points.size() != arcs.size())
                throw std::runtime_error("Invalid annotated path");
            double previous = -1.;
            for (size_t i = 0; i < arcs.size(); ++i)
            {
                if (!std::isfinite(arcs[i]) || arcs[i] < previous ||
                    !std::isfinite(points[i].x) || !std::isfinite(points[i].y))
                    throw std::runtime_error("Invalid path samples");
                if (i && std::abs(arcs[i] - arcs[i - 1] -
                    distance(points[i], points[i - 1])) > 1e-4)
                    throw std::runtime_error("Arc length does not match path");
                previous = arcs[i];
            }
            if (std::abs(arcs.front()) > 1e-6)
                throw std::runtime_error("Arc must start at zero");
            previous = -1;
            for (const auto& t : traversals)
            {
                if (t.id.empty() || t.mode != "folded" || !std::isfinite(t.prepare) ||
                    !std::isfinite(t.enter) || !std::isfinite(t.exit) ||
                    !std::isfinite(t.release) || t.prepare < 0 || t.prepare < previous ||
                    t.prepare > t.enter || t.enter >= t.exit || t.exit > t.release ||
                    t.release > arcs.back() + 1e-5 || !std::isfinite(t.speed) ||
                    t.speed <= 0 || !std::isfinite(t.fold_hold) || t.fold_hold < 0 ||
                    !std::isfinite(t.restore_hold) || t.restore_hold < 0)
                    throw std::runtime_error("Invalid traversal intervals");
                previous = t.prepare;
            }
        }

        Point at(double s) const
        {
            if (points.empty())
                return {};
            auto it = std::upper_bound(arcs.begin(), arcs.end(), s);
            if (it == arcs.begin())
                return points.front();
            if (it == arcs.end())
                return points.back();
            size_t i = size_t(it - arcs.begin());
            double a = (s - arcs[i - 1]) / std::max(1e-12, arcs[i] - arcs[i - 1]);
            return {
                points[i - 1].x + a * (points[i].x - points[i - 1].x),
                points[i - 1].y + a * (points[i].y - points[i - 1].y)
            };
        }
    };

    struct ExecutionOutput
    {
        std::string phase = "NORMAL", passage_id, request_id, request_mode;
        double progress = 0, target_limit = std::numeric_limits<double>::infinity(),
               speed_limit = std::numeric_limits<double>::infinity();
        bool stop = false, locked = false, folded = false, fault = false,
             intentional_wait = false;
    };

    class Execution
    {
    public:
        double stop_tolerance = .04, stopped_speed = .03, action_timeout = 8.,
               deceleration = .8;
        bool hasPlan() const { return !plan_.points.empty(); }
        const ExecutionPlan& plan() const { return plan_; }
        const ExecutionOutput& output() const { return out_; }
        // 同一路径重发不重置状态；变形请求后不允许异步重规划覆盖正在执行的通道。
        bool bind(ExecutionPlan p)
        {
            p.validate();
            if (p.id == plan_.id)
                return true;
            if (out_.locked)
                return false;
            plan_ = std::move(p);
            out_ = ExecutionOutput{};
            index_ = 0;
            group_end_ = 0;
            ack_ = false;
            previous_time_ = -1;
            return true;
        }

        void feedback(const std::string& id, const std::string& mode, bool completed,
                      bool fault,
                      double now = std::numeric_limits<double>::quiet_NaN())
        {
            if (id != out_.request_id || mode != out_.request_mode || id.empty())
                return;
            if (fault)
                fail();
            else if (completed && !ack_)
            {
                ack_ = true;
                acknowledged_at_ = std::isfinite(now) ? now : previous_time_;
            }
        }

        void fail()
        {
            out_.phase = "FAULT";
            out_.fault = true;
            out_.locked = true;
            out_.stop = true;
            out_.intentional_wait = false;
        }

        ExecutionOutput tick(Point robot, double speed, double angular_speed,
                             double now)
        {
            if (!hasPlan())
                return out_;
            if (!std::isfinite(now) || !std::isfinite(speed) ||
                !std::isfinite(angular_speed) || !std::isfinite(robot.x) ||
                !std::isfinite(robot.y))
            {
                fail();
                return out_;
            }
            // 使用 steady_clock；即使外部传入时钟回退，也不能误判停留已经结束。
            if (previous_time_ >= 0 && now < previous_time_)
            {
                fail();
                return out_;
            }
            previous_time_ = now;
            project(robot);
            if (out_.fault)
                return out_;
            out_.stop = false;
            out_.intentional_wait = false;
            out_.speed_limit = std::numeric_limits<double>::infinity();
            out_.target_limit = plan_.arcs.back();
            if (index_ >= plan_.traversals.size())
            {
                out_.phase = "NORMAL";
                out_.passage_id.clear();
                return out_;
            }
            const auto& t = plan_.traversals[index_];
            out_.passage_id = t.id;
            if (!out_.locked)
            {
                out_.phase = "APPROACH";
                out_.target_limit = t.prepare;
                const double remaining = std::max(0., t.prepare - out_.progress);
                out_.speed_limit = std::min(
                    t.speed,
                    std::sqrt(2 * deceleration *
                        std::max(remaining, distance(robot, plan_.at(t.prepare)) -
                                 stop_tolerance * .5)));
                if (remaining <= stop_tolerance &&
                    distance(robot, plan_.at(t.prepare)) <= stop_tolerance)
                {
                    out_.stop = true;
                    if (speed <= stopped_speed && std::abs(angular_speed) <= .05)
                    {
                        group_end_ = index_;
                        while (group_end_ + 1 < plan_.traversals.size() &&
                            plan_.traversals[group_end_ + 1].prepare <=
                            plan_.traversals[group_end_].release + 1e-6)
                            ++group_end_;
                        out_.folded = true;
                        out_.locked = true;
                        request("folded", now);
                        out_.phase = "WAIT_FOLD";
                    }
                }
            }
            if (out_.phase == "WAIT_FOLD" || out_.phase == "WAIT_RESTORE")
            {
                out_.stop = true;
                out_.intentional_wait = true;
                if (now - action_started_ > action_timeout)
                {
                    fail();
                    return out_;
                }
                double hold = out_.phase == "WAIT_FOLD"
                                  ? t.fold_hold
                                  : plan_.traversals[group_end_].restore_hold;
                if (speed > stopped_speed || std::abs(angular_speed) > .05)
                    stopped_since_ = now;
                if (ack_ && now - std::max(stopped_since_, acknowledged_at_) >= hold)
                {
                    if (out_.phase == "WAIT_FOLD")
                        out_.phase = "PASSING";
                    else
                    {
                        out_.folded = false;
                        out_.locked = false;
                        out_.phase = "NORMAL";
                        index_ = group_end_ + 1;
                        out_.request_id.clear();
                        out_.request_mode.clear();
                    }
                    out_.stop = false;
                    out_.intentional_wait = false;
                }
            }
            if (out_.phase == "PASSING")
            {
                const double release = plan_.traversals[group_end_].release;
                out_.target_limit = release;
                for (size_t i = index_; i <= group_end_; ++i)
                    out_.speed_limit =
                        std::min(out_.speed_limit, plan_.traversals[i].speed);
                out_.speed_limit =
                    std::min(out_.speed_limit,
                             std::sqrt(2 * deceleration *
                                 std::max({
                                     0., release - out_.progress,
                                     distance(robot, plan_.at(release)) -
                                     stop_tolerance * .5
                                 })));
                if (release - out_.progress <= stop_tolerance &&
                    distance(robot, plan_.at(release)) <= stop_tolerance)
                {
                    out_.stop = true;
                    if (speed <= stopped_speed && std::abs(angular_speed) <= .05)
                    {
                        request("normal", now);
                        out_.phase = "WAIT_RESTORE";
                        out_.intentional_wait = true;
                    }
                }
            }
            return out_;
        }

    private:
        ExecutionPlan plan_;
        ExecutionOutput out_;
        size_t index_ = 0, group_end_ = 0;
        uint64_t sequence_ = 0;
        double action_started_ = 0, stopped_since_ = 0, previous_time_ = -1,
               acknowledged_at_ = 0;
        bool ack_ = false;

        void request(const std::string& mode, double now)
        {
            out_.request_id = plan_.id + ":" + std::to_string(++sequence_);
            out_.request_mode = mode;
            ack_ = false;
            action_started_ = stopped_since_ = now;
        }

        void project(Point p)
        {
            // 局部弧长窗口避免路径折返、自交或定位跳变时提前触发出口事件。
            double best = std::numeric_limits<double>::infinity(), s = out_.progress;
            for (size_t i = 1; i < plan_.points.size(); ++i)
            {
                if (plan_.arcs[i] < out_.progress - .1 ||
                    plan_.arcs[i - 1] > out_.progress + 1.)
                    continue;
                auto a = plan_.points[i - 1], b = plan_.points[i];
                double dx = b.x - a.x, dy = b.y - a.y;
                double u = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) /
                                      std::max(1e-12, dx * dx + dy * dy),
                                      0., 1.);
                double d = distance(p, {a.x + u * dx, a.y + u * dy});
                if (d < best)
                {
                    best = d;
                    s = plan_.arcs[i - 1] + u * (plan_.arcs[i] - plan_.arcs[i - 1]);
                }
            }
            if (out_.locked && best > .25)
            {
                fail();
                return;
            }
            if (best < .5)
                out_.progress = std::max(out_.progress, s);
        }
    };
} // namespace lc_passage_nav2
