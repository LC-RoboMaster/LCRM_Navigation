#include "lc_passage_nav2/tdt_passage_planner.hpp"
#include "lc_passage_nav2/passage_markers.hpp"
#include <chrono>
#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace lc_passage_nav2
{
    void TdtPassagePlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto node = parent.lock();
        if (!node || !costmap)
            throw nav2_core::PlannerException("Planner parent/costmap unavailable");
        node_ = parent;
        name_ = std::move(name);
        tf_ = std::move(tf);
        costmap_ros_ = std::move(costmap);
        global_frame_ = costmap_ros_->getGlobalFrameID();
        auto param = [&](const std::string& key, const auto& fallback, auto& out)
        {
            nav2_util::declare_parameter_if_not_declared(
                node, name_ + "." + key, rclcpp::ParameterValue(fallback));
            node->get_parameter(name_ + "." + key, out);
        };

        //读取通道参数文件
        param("passage_file", std::string(""), project_file_);
        config_ = loadPassages(project_file_);
        if (!config_.passages.empty() && config_.frame != global_frame_)
            throw nav2_core::PlannerException(
                "Passage frame must equal global costmap frame");
        param("unknown_is_obstacle", true, unknown_is_obstacle_);
        auto& c = project_.planner;
        double radius = costmap_ros_->getRobotRadius();
        param("robot_radius", radius, c.robot_radius);
        param("safety_margin", .02, c.safety_margin);
        param("obstacle_weight", 2., c.obstacle_weight);
        param("obstacle_decay", .35, c.obstacle_decay);
        param("source_cost_weight", .5, c.source_cost_weight);
        param("optimize", true, c.optimize);
        param("allow_geometric_fallback", true, c.allow_geometric_fallback);
        param("search_timeout_ms", 200., c.search_timeout_ms);
        param("optimization_timeout_ms", 200., c.optimization_timeout_ms);
        param("graph_budget_ms", 1000., project_.graph_budget_ms);
        param("sample_spacing", .025, project_.sample_spacing);
        param("cruise_speed", 1., project_.cruise_speed);
        c.collect_debug_images = false;
        c.validate();
        if (!std::isfinite(project_.graph_budget_ms) ||
            project_.graph_budget_ms <= 0 ||
            !std::isfinite(project_.sample_spacing) || project_.sample_spacing <= 0 ||
            !std::isfinite(project_.cruise_speed) || project_.cruise_speed <= 0)
            throw nav2_core::PlannerException("Invalid graph budget/spacing/speed");
        for (const auto& g : config_.passages)
        {
            tdt_passage::Passage p;
            p.id = g.id;
            p.region.center = {g.center.x, g.center.y};
            p.region.length = g.length;
            p.region.width = g.width;
            p.region.yaw = g.yaw;
            p.a = {g.a.x, g.a.y};
            p.b = {g.b.x, g.b.y};
            p.enabled = g.enabled;
            p.direction = g.direction;
            p.mode = g.mode;
            p.prepare_distance = g.prepare;
            p.release_distance = g.release;
            p.speed = g.speed;
            p.action_cost_seconds = g.action_cost;
            p.clear_static =
                false; // 静态清理由 passage_map 完成，不能擦除合并后的实时障碍。
            p.clear_unknown = false; // 同样不擦除合并后的未知/实时观测。
            project_.passages.push_back(p);
        }
        plan_pub_ = node->create_publisher<lc_passage_msgs::msg::PassagePlan>(
            "passage/plan", rclcpp::QoS(32).reliable().transient_local());
        diagnostics_pub_ = node->create_publisher<std_msgs::msg::String>(
            "passage/planner_diagnostics", rclcpp::QoS(1).reliable().transient_local());
        marker_pub_ = node->create_publisher<MarkerArray>(
            "passage/planner_markers", rclcpp::QoS(1).reliable().transient_local());
        RCLCPP_INFO(node->get_logger(),
            "Passage planner configured: file='%s', %zu passages, radius=%.3f m, margin=%.3f m",
            project_file_.c_str(), project_.passages.size(), c.robot_radius, c.safety_margin);
        state_sub_ = node->create_subscription<lc_passage_msgs::msg::PassageState>(
            "passage/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](lc_passage_msgs::msg::PassageState::ConstSharedPtr s)
            {
                std::lock_guard<std::mutex> guard(state_mutex_);
                locked_ = s->locked;
            });
        active_sub_ = node->create_subscription<lc_passage_msgs::msg::PassagePlan>(
            "passage/active_plan", rclcpp::QoS(1).reliable().transient_local(),
            [this](lc_passage_msgs::msg::PassagePlan::ConstSharedPtr p)
            {
                std::lock_guard<std::mutex> guard(state_mutex_);
                committed_ = *p;
            });
        configured_ = true;
    }

    void TdtPassagePlanner::activate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_pub_->on_activate();
        diagnostics_pub_->on_activate();
        marker_pub_->on_activate();
        active_ = true;
    }

    void TdtPassagePlanner::deactivate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = false;
        if (plan_pub_)
            plan_pub_->on_deactivate();
        if (diagnostics_pub_) diagnostics_pub_->on_deactivate();
        if (marker_pub_) marker_pub_->on_deactivate();
    }

    void TdtPassagePlanner::cleanup()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = configured_ = false;
        plan_pub_.reset();
        diagnostics_pub_.reset();
        marker_pub_.reset();
        last_diagnostic_state_.clear();
        state_sub_.reset();
        active_sub_.reset();
        costmap_ros_.reset();
        tf_.reset();
        project_ = {};
        node_.reset();
        std::lock_guard<std::mutex> guard(state_mutex_);
        committed_ = lc_passage_msgs::msg::PassagePlan{};
        locked_ = false;
    }

    nav_msgs::msg::Path
    TdtPassagePlanner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto node = node_.lock();
        if (!node || !configured_ || !active_)
            throw nav2_core::PlannerException("Planner inactive");
        if (start.header.frame_id != global_frame_ ||
            goal.header.frame_id != global_frame_)
            throw nav2_core::PlannerException("Start/goal frame mismatch");
        {
            std::lock_guard<std::mutex> guard(state_mutex_);
            if (locked_)
            {
                if (committed_.path.poses.empty())
                    throw nav2_core::PlannerException("Waiting for committed passage path");
                plan_pub_->publish(committed_);
                return committed_.path;
            }
        }
        try
        {
            auto request = project_;
            std::vector<uint8_t> cells;
            int w, h;
            tdt_nav::MapInfo info;
            auto* map = costmap_ros_->getCostmap();
            {
                // 只在复制地图时加锁，A*、距离场、OSQP 不阻塞传感器更新。
                std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> guard(
                    *map->getMutex());
                w = map->getSizeInCellsX();
                h = map->getSizeInCellsY();
                info.resolution = map->getResolution();
                info.origin = {map->getOriginX(), map->getOriginY()};
                info.frame_id = global_frame_;
                cells.assign(map->getCharMap(), map->getCharMap() + size_t(w) * h);
            }
            // SLAM 可改变地图宽、高、原点。254 是实际障碍；253 及以下仅用作软代价。
            request.map =
                tdt_nav::Maps::costmap(cells, w, h, info, 254, unknown_is_obstacle_);
            auto result = tdt_passage::Planner().plan(
                request, {start.pose.position.x, start.pose.position.y},
                {goal.pose.position.x, goal.pose.position.y});
            // Publish on BOTH success and failure; graph warnings used to be
            // discarded here, hiding why a configured passage was bypassed.
            nlohmann::json diagnostic = {
                {"status",result.status}, {"message",result.message},
                {"success",result.success}, {"selected_count",result.traversals.size()},
                {"route_cost_seconds",result.cost_seconds}, {"length",result.length},
                {"elapsed_ms",result.elapsed_ms}, {"passage_file",project_file_},
                {"map",{{"origin",{info.origin.x(),info.origin.y()}},
                         {"resolution",info.resolution},{"width",w},{"height",h}}},
                {"robot_radius",request.planner.robot_radius},
                {"safety_margin",request.planner.safety_margin},
                {"passage_checks",nlohmann::json::array()}, {"warnings",result.warnings},
                {"ordinary_route_cost_seconds",nullptr}};
            for (const auto &e : result.edges)
                if (e.kind == "ordinary" && e.from == 0 && e.to == 1)
                    diagnostic["ordinary_route_cost_seconds"] = e.cost;
            auto markers = emptyMarkers(global_frame_);
            std::string summary = result.status;
            size_t usable = 0;
            for (const auto &c : result.passage_checks) {
                diagnostic["passage_checks"].push_back({
                    {"passage_id",c.passage_id},{"direction",c.direction},
                    {"valid",c.valid},{"selected",c.selected},{"reason",c.reason},
                    {"entry",{c.entry.x(),c.entry.y()}},{"exit",{c.exit.x(),c.exit.y()}},
                    {"release",{c.release.x(),c.release.y()}},{"cost_seconds",c.cost_seconds}});
                usable += c.valid;
                summary += ";" + c.passage_id + ":" + c.direction + ":" + c.reason;
                const auto color = !c.valid ? markerColor(1.f,.15f,.1f) :
                    c.selected ? markerColor(.15f,1.f,.2f) : markerColor(1.f,.75f,.1f);
                Point a{c.entry.x(),c.entry.y()}, b{c.exit.x(),c.exit.y()};
                const Point middle{(a.x+b.x)/2,(a.y+b.y)/2};
                markerLine(markers,global_frame_,"edge_status",a,middle,.22,color,true);
                const double len = std::max(distance(a,b),1e-6);
                Point label{middle.x-(b.y-a.y)/len*.3,middle.y+(b.x-a.x)/len*.3};
                markerText(markers,global_frame_,"edge_reasons",label,.52,
                    c.passage_id+" "+c.direction+"\n"+c.reason,color);
            }
            std_msgs::msg::String message;
            message.data = diagnostic.dump();
            diagnostics_pub_->publish(message);
            marker_pub_->publish(markers);
            if (summary != last_diagnostic_state_) {
                RCLCPP_INFO(node->get_logger(),
                    "Passage plan: %s, usable directions=%zu, selected=%zu, route cost=%.3f",
                    result.status.c_str(),usable,result.traversals.size(),result.cost_seconds);
                for (const auto &warning : result.warnings)
                    RCLCPP_WARN(node->get_logger(),"%s",warning.c_str());
                last_diagnostic_state_ = summary;
            }
            if (!result.success)
                throw nav2_core::PlannerException(result.status + ": " + result.message);
            lc_passage_msgs::msg::PassagePlan meta;
            meta.header.frame_id = global_frame_;
            meta.header.stamp = node->now();
            meta.path.header = meta.header;
            meta.plan_id =
                std::to_string(
                    std::chrono::steady_clock::now().time_since_epoch().count()) +
                ":" + std::to_string(result.path_id);
            meta.arc_lengths = result.arc_lengths;
            for (size_t i = 0; i < result.path.size(); ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = meta.header;
                pose.pose.position.x = result.path[i].x();
                pose.pose.position.y = result.path[i].y();
                double yaw = 0;
                if (result.path.size() > 1)
                {
                    tdt_passage::Vec2 d = i + 1 < result.path.size()
                                              ? result.path[i + 1] - result.path[i]
                                              : result.path[i] - result.path[i - 1];
                    yaw = std::atan2(d.y(), d.x());
                }
                pose.pose.orientation.z = std::sin(yaw / 2);
                pose.pose.orientation.w = std::cos(yaw / 2);
                meta.path.poses.push_back(pose);
            }
            const auto& q = goal.pose.orientation;
            double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (std::isfinite(norm) && norm > 1e-9)
            {
                auto& out = meta.path.poses.back().pose.orientation;
                out.x = q.x / norm;
                out.y = q.y / norm;
                out.z = q.z / norm;
                out.w = q.w / norm;
            }
            for (const auto& t : result.traversals)
            {
                lc_passage_msgs::msg::PassageTraversal v;
                v.passage_id = t.passage_id;
                v.direction = t.direction;
                v.mode = t.mode;
                v.prepare_s = t.prepare_s;
                v.enter_s = t.enter_s;
                v.exit_s = t.exit_s;
                v.release_s = t.release_s;
                v.speed = t.speed;
                for (const auto& g : config_.passages)
                    if (g.id == t.passage_id)
                    {
                        v.fold_hold_seconds = g.fold_hold;
                        v.restore_hold_seconds = g.restore_hold;
                    }
                meta.traversals.push_back(v);
            }
            plan_pub_->publish(meta);
            return meta.path;
        }
        catch (const nav2_core::PlannerException&)
        {
            throw;
        }
        catch (const std::exception& e)
        {
            throw nav2_core::PlannerException(std::string("Passage planner: ") +
                e.what());
        }
    }
} // namespace lc_passage_nav2
PLUGINLIB_EXPORT_CLASS(lc_passage_nav2::TdtPassagePlanner,
                       nav2_core::GlobalPlanner)
