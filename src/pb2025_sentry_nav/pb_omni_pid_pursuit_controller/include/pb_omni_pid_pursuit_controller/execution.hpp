#pragma once
// 无 ROS 的控制状态机，便于离线回放、单元测试及移植到其他控制器。
#include "lc_passage_common/geometry.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <utility>

namespace pb_omni_pid_pursuit_controller
{
    using lc_passage_common::Point;
    using lc_passage_common::distance;
    //通道对象
    struct Traversal
    {
        std::string id, direction, mode = "folded"; //通道编号，通行方向

        double prepare = 0; // 准备动作点弧长
        double enter = 0; // 入口弧长
        double exit = 0; // 出口弧长
        double release = 0; // 释放点弧长
        double speed = 3.0; // 穿越速度上限
        double fold_hold = .3;
        double restore_hold = .3;
    };

    struct ExecutionPlan
    {
        std::string id; // 当前执行的路径编号
        std::vector<Point> points; // 完整路径点
        std::vector<double> arcs; // 每个路径点对应的累计弧长
        std::vector<Traversal> traversals; // 这条路径需要执行的通道穿越安排

        /**
         * @brief 合法性检查
         */
        void validate() const
        {
            // 非空检查，路径点和弧长点数量是否相等
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

        /**
         * @brief 计算路径当前弧长对应的坐标
         * @param s 路径上的弧长
         * @return  对应弧长的地图坐标
         */
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

    // 态机当前执行到哪一步，以及本周期给控制器的要求
    struct ExecutionOutput
    {
        std::string phase = "NORMAL"; // 当前阶段
        std::string passage_id; // 当前通道编号
        std::string request_id; // 当前动作请求编号
        std::string request_mode; // 请求 folded 或 normal

        double progress = 0; // 当前路径进度，单位 m

        double target_limit =
            std::numeric_limits<double>::infinity(); //控制器前视目标允许到达的最远弧长距离

        double speed_limit =
            std::numeric_limits<double>::infinity(); //本周期状态机施加平移速度上限

        bool stop = false;
        bool locked = false;
        bool folded = false;
        bool fault = false;
        bool intentional_wait = false;
    };

    class Execution
    {
    public:
        double stop_tolerance = .04; //到路标点判断范围
        double stopped_speed = .03; //实际速度低于此值认为停止
        double action_timeout = 8.; //等待机器人动作执行完成超时时间
        double deceleration = .8;

        /**
         * @brief 检查路径是否为空
         * @return bool是否为空
         */
        bool hasPlan() const
        {
            return !plan_.points.empty();
        }

        /**
         * @brief 获取路径
         * @return ExecutionPlan路径
         */
        const ExecutionPlan& plan() const
        {
            return plan_;
        }

        /**
         * @brief 当前状态查询
         * @return ExecutionOutput
         */
        const ExecutionOutput& output() const
        {
            return out_;
        }

        /**
         * @brief 绑定新的路径计划
         * @param p 需要绑定的计划
         * @return  是否替换成功
         */
        bool bind(ExecutionPlan p)
        {
            //检查是否合法
            p.validate();
            //如果是一样的路径，接受并返回true
            if (p.id == plan_.id)
            {
                return true;
            }
            //如果当前状态锁定，拒绝接受新计划
            if (out_.locked)
            {
                return false;
            }
            //接受新计划，重置状态
            plan_ = std::move(p);
            out_ = ExecutionOutput{};
            index_ = 0;
            group_end_ = 0;
            ack_ = false;
            previous_time_ = -1;
            return true;
        }

        /**
         * @brief 接收动作执行反馈
         * @param id 动作请求ID
         * @param mode 请求的模式
         * @param completed 是否完成
         * @param fault 是否失败
         * @param now 当前时间
         */
        void feedback(const std::string& id, const std::string& mode, bool completed, bool fault,
                      double now = std::numeric_limits<double>::quiet_NaN())
        {
            if (id != out_.request_id || mode != out_.request_mode || id.empty())
            {
                return;
            }
            if (fault)
            {
                fail();
            }
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

        /**
         *
         * @param robot 当前机器人位置
         * @param speed 机器人平移速度大小
         * @param angular_speed 机器人角速度
         * @param now 当前时间
         * @return ExecutionOutput
         */
        ExecutionOutput tick(Point robot, double speed, double angular_speed, double now)
        {
            //检查参数
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
            //跟新路径进度
            project(robot);
            if (out_.fault)
            {
                return out_;
            }
            //重新初始化本周期的临时控制约束
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
                // 洞外只把准备点作为当前跟踪终点，由普通 PID 和现有限速控制。
                // 不在这里施加通道 speed 或额外制动包络；通道限速仅在 PASSING 生效。
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
                double hold = out_.phase == "WAIT_FOLD" ? t.fold_hold : plan_.traversals[group_end_].restore_hold;
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
                // 变形确认后至释放点为一次穿越；连续通道组采用最小通道速度。
                // 制动包络仅用于本次穿越末尾停稳，不影响洞外普通接近路段。
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
        ExecutionPlan plan_; //当前采用的完整路径计划
        ExecutionOutput out_; //当前执行结果

        size_t index_ = 0; //当前待处理通道在traversals中的下标
        size_t group_end_ = 0; //本次连续穿越通道组的最后一个下标

        uint64_t sequence_ = 0; //动作请求序号，用来生成不同的request_id
        double action_started_ = 0; //当前动作请求开始的时间
        double stopped_since_ = 0;
        double previous_time_ = -1; //上一次tick()的时间
        double acknowledged_at_ = 0; //首次收到当前动作成功反馈的时间
        bool ack_ = false; //当前动作是否已经收到成功的确认

        /**
         * @brief 准备一次新的机械动作请求
         * @param mode 请求模式
         * @param now 请求时间
         */
        void request(const std::string& mode, double now)
        {
            out_.request_id = plan_.id + ":" + std::to_string(++sequence_);
            out_.request_mode = mode;
            ack_ = false;
            action_started_ = stopped_since_ = now;
        }

        /**
         *
         * @param p 将当前机器人位置转换为路径进度
         */
        void project(Point p)
        {
            // 局部弧长窗口避免路径折返、自交或定位跳变时提前触发出口事件。
            double best = std::numeric_limits<double>::infinity();
            double s = out_.progress;
            for (size_t i = 1; i < plan_.points.size(); ++i)
            {
                if (plan_.arcs[i] < out_.progress - 0.1 || plan_.arcs[i - 1] > out_.progress + 1.0)
                {
                    continue;
                }
                auto a = plan_.points[i - 1];
                auto b = plan_.points[i];
                double dx = b.x - a.x, dy = b.y - a.y;
                double u = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / std::max(1e-12, dx * dx + dy * dy),
                                      0.0, 1.0);
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
            {
                out_.progress = std::max(out_.progress, s);
            }
        }
    };
} // namespace pb_omni_pid_pursuit_controller
