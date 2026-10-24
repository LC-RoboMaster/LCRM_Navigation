#include <atomic>
#include <fstream>
#include <numeric>
#include <set>
#include <tdt_passage/semantic_planner.hpp>

namespace tdt_passage {
namespace {
constexpr double EPS = 1e-8;
Vec2 vec(const Json &v) {
  if (!v.is_array() || v.size() != 2)
    throw std::invalid_argument("position must have two numbers");
  Vec2 p(v.at(0).get<double>(), v.at(1).get<double>());
  if (!p.allFinite())
    throw std::invalid_argument("position must be finite");
  return p;
}
Json jvec(const Vec2 &p) { return Json::array({p.x(), p.y()}); }
Json jpath(const Path &p) {
  Json a = Json::array();
  for (const auto &v : p)
    a.push_back(jvec(v));
  return a;
}
RotatedRect rect(const Json &j) {
  RotatedRect r;
  r.center = vec(j.at("center"));
  r.length = j.at("length");
  r.width = j.at("width");
  r.yaw = j.value("yaw", 0.);
  return r;
}
void validateRect(const RotatedRect &r) {
  if (!r.center.allFinite() || !std::isfinite(r.yaw) ||
      !std::isfinite(r.length) || !std::isfinite(r.width) || r.length <= 0 ||
      r.width <= 0)
    throw std::invalid_argument(
        "rectangle dimensions must be positive and finite");
}
bool intersects(const RotatedRect &a, const RotatedRect &b) {
  const auto ca = a.corners(), cb = b.corners();
  const Vec2 ad = a.axis(), bd = b.axis();
  const std::array<Vec2, 4> axes{ad, Vec2(-ad.y(), ad.x()), bd,
                                 Vec2(-bd.y(), bd.x())};
  for (const auto &axis : axes) {
    double amin = 1e100, amax = -1e100, bmin = 1e100, bmax = -1e100;
    for (auto &p : ca) {
      amin = std::min(amin, p.dot(axis));
      amax = std::max(amax, p.dot(axis));
    }
    for (auto &p : cb) {
      bmin = std::min(bmin, p.dot(axis));
      bmax = std::max(bmax, p.dot(axis));
    }
    if (amax < bmin - EPS || bmax < amin - EPS)
      return false;
  }
  return true;
}
// Rotate a segment to the rectangle's local frame and clip it against all four
// sides.
bool crossesRegion(const RotatedRect &r, const Vec2 &a, const Vec2 &b) {
  Vec2 d = r.axis(), n(-d.y(), d.x());
  Vec2 q = a - r.center, v = b - a;
  double t0 = 0, t1 = 1;
  for (const auto &e : std::array<std::pair<Vec2, double>, 2>{
           {{d, r.length * .5}, {n, r.width * .5}}}) {
    double x = q.dot(e.first), dx = v.dot(e.first);
    if (std::abs(dx) < EPS) {
      if (std::abs(x) > e.second)
        return false;
      continue;
    }
    double lo = (-e.second - x) / dx, hi = (e.second - x) / dx;
    if (lo > hi)
      std::swap(lo, hi);
    t0 = std::max(t0, lo);
    t1 = std::min(t1, hi);
    if (t0 > t1)
      return false;
  }
  return t1 - t0 > EPS;
}
void paintRegion(tdt_nav::GridMap &m, const RotatedRect &r, bool clear,
                 bool clear_unknown = false) {
  for (int y = 0; y < m.free.rows; ++y)
    for (int x = 0; x < m.free.cols; ++x) {
      if (!r.contains(m.cellCenterToWorld(x, y)))
        continue;
      if (clear && m.unknown.at<uint8_t>(y, x) && !clear_unknown)
        continue; // annotation does not invent observed free space
      m.free.at<uint8_t>(y, x) = clear ? 255 : 0;
      if (clear && clear_unknown) m.unknown.at<uint8_t>(y, x) = 0;
      if (clear)
        m.soft.at<float>(y, x) = 0;
    }
}
void append(Path &dest, const Path &src) {
  for (const auto &p : src)
    if (dest.empty() || (dest.back() - p).norm() > EPS)
      dest.push_back(p);
}
double length(const Path &p) {
  double s = 0;
  for (size_t i = 1; i < p.size(); ++i)
    s += (p[i] - p[i - 1]).norm();
  return s;
}
Path sampleLine(const Vec2 &a, const Vec2 &b, double step) {
  double dist = (b - a).norm();
  int count = std::max(1, static_cast<int>(std::ceil(dist / step)));
  Path p;
  for (int i = 0; i <= count; ++i)
    p.push_back(a + (b - a) * (double(i) / count));
  return p;
}
Path densify(const Path &p, double step) {
  if (p.empty())
    return {};
  Path out{p.front()};
  for (size_t i = 1; i < p.size(); ++i)
    append(out, sampleLine(p[i - 1], p[i], step));
  return out;
}
// Keep physical/unknown rejection distinct from ordinary endpoint inflation.
// A closed gate must never silently look like an intentional cheaper detour.
std::string lineProblem(const tdt_nav::GridMap &map, const Vec2 &a,
                        const Vec2 &b, double step) {
  cv::Point last(-1, -1);
  auto problem = [&](cv::Point c) -> std::string {
    if (map.free.at<uint8_t>(c)) return {};
    const auto p = map.cellCenterToWorld(c.x, c.y);
    std::ostringstream s;
    s << (map.unknown.at<uint8_t>(c) ? "centerline_unknown" : "centerline_obstacle")
      << " at (" << std::fixed << std::setprecision(3) << p.x() << ", " << p.y() << ")";
    return s.str();
  };
  for (const auto &p : sampleLine(a, b, step)) {
    cv::Point c;
    if (!map.worldToCell(p, c)) return "centerline_outside_map";
    auto reason = problem(c);
    if (!reason.empty()) return reason;
    if (last.x >= 0 && c.x != last.x && c.y != last.y) {
      for (auto corner : {cv::Point(c.x, last.y), cv::Point(last.x, c.y)}) {
        reason = problem(corner);
        if (!reason.empty()) return reason;
      }
    }
    last = c;
  }
  return {};
}
double pathCost(const Path &path, const tdt_nav::GridMap &map,
                const cv::Mat &cost, double speed) {
  double total = 0.;
  for (size_t i = 1; i < path.size(); ++i) {
    const double dist = (path[i] - path[i - 1]).norm();
    const int steps =
        std::max(1, int(std::ceil(dist / (map.info.resolution * .5))));
    double sum = 0.;
    for (int k = 0; k < steps; ++k) {
      Vec2 p = path[i - 1] + (path[i] - path[i - 1]) * ((k + .5) / steps);
      cv::Point c;
      if (!map.worldToCell(p, c))
        return 1e100;
      sum += cost.at<float>(c);
    }
    total += dist * sum / steps / speed;
  }
  return total;
}
struct EdgeData {
  GraphEdge edge;
  Path raw, simple;
  int gate = -1;
  bool reverse = false;
};
std::string nodeName(const Passage &p, bool reverse, bool release) {
  return p.id + (reverse ? ":B_to_A:" : ":A_to_B:") +
         (release ? "release" : "entry");
}
} // namespace

Vec2 RotatedRect::axis() const { return {std::cos(yaw), std::sin(yaw)}; }
double RotatedRect::signedDistance(const Vec2 &p) const {
  Vec2 d = axis(), n(-d.y(), d.x()), v = p - center;
  Vec2 q(std::abs(v.dot(d)) - length * .5, std::abs(v.dot(n)) - width * .5);
  return q.cwiseMax(0.).norm() + std::min(std::max(q.x(), q.y()), 0.);
}
bool RotatedRect::contains(const Vec2 &p, double padding) const {
  return signedDistance(p) <= padding;
}
std::array<Vec2, 4> RotatedRect::corners() const {
  Vec2 d = axis() * length * .5,
       n(-std::sin(yaw) * width * .5, std::cos(yaw) * width * .5);
  return {center - d - n, center + d - n, center + d + n, center - d + n};
}

tdt_nav::GridMap prepareEnvironment(const Project &project) {
  project.map.validate();
  auto effective = project.map;
  effective.free = project.map.free.clone();
  effective.unknown = project.map.unknown.clone();
  effective.soft = project.map.soft.clone();
  for (const auto &g : project.passages) {
    validateRect(g.region);
    if (g.enabled && g.clear_static)
      paintRegion(effective, g.region, true, g.clear_unknown);
  }
  for (const auto &b : project.blockers) {
    validateRect(b);
    paintRegion(effective, b, false);
  }
  if (!project.runtime_obstacles.empty()) {
    if (project.runtime_obstacles.type() != CV_8UC1 ||
        project.runtime_obstacles.size() != effective.free.size())
      throw std::invalid_argument("runtime obstacle mask must be CV_8UC1 and match the static map");
    effective.free.setTo(0, project.runtime_obstacles);
    effective.soft.setTo(0, project.runtime_obstacles);
  }
  return effective;
}

Result Planner::plan(const Project &project, const Vec2 &start,
                     const Vec2 &goal) const {
  Result out;
  const auto begin = tdt_nav::Clock::now();
  static std::atomic<uint64_t> next{1};
  out.path_id =
      static_cast<uint64_t>(
          std::chrono::system_clock::now().time_since_epoch().count()) +
      next++;
  auto fail = [&](const std::string &status, const std::string &message) {
    out.status = status;
    out.message = message;
    out.elapsed_ms = tdt_nav::elapsedMs(begin);
    return out;
  };
  try {
    project.map.validate();
    project.planner.validate();
    if (!start.allFinite() || !goal.allFinite())
      return fail("invalid_request", "start/goal must be finite");
    if (project.passages.size() > 16)
      return fail("invalid_project",
                  "at most 16 passages per configured map");
    if (!std::isfinite(project.cruise_speed) || project.cruise_speed <= 0 ||
        !std::isfinite(project.graph_budget_ms) ||
        project.graph_budget_ms <= 0 ||
        !std::isfinite(project.sample_spacing) || project.sample_spacing <= 0)
      return fail("invalid_project", "invalid cruise speed or planner budgets");
    std::set<std::string> ids;
    for (size_t i = 0; i < project.passages.size(); ++i) {
      const auto &g = project.passages[i];
      validateRect(g.region);
      if (g.id.empty() || !ids.insert(g.id).second)
        return fail("invalid_project",
                    "passage IDs must be nonempty and unique");
      if (g.direction != "both" && g.direction != "a_to_b" &&
          g.direction != "b_to_a")
        return fail("invalid_project", g.id + ": invalid direction");
      if (!g.a.allFinite() || !g.b.allFinite() || (g.b - g.a).norm() < 1e-5)
        return fail("invalid_project", g.id + ": entry points must differ");
      if (!std::isfinite(g.prepare_distance) || g.prepare_distance < 0 ||
          !std::isfinite(g.release_distance) || g.release_distance < 0 ||
          !std::isfinite(g.speed) || g.speed <= 0 ||
          g.speed > project.planner.max_speed ||
          !std::isfinite(g.action_cost_seconds) || g.action_cost_seconds < 0)
        return fail("invalid_project",
                    g.id + ": invalid distances/speed/action cost");
      if (g.region.contains(g.a) || g.region.contains(g.b) ||
          !crossesRegion(g.region, g.a, g.b))
        return fail("invalid_project",
                    g.id + ": A and B must be outside the rectangle and their "
                           "segment must cross it");
      const Vec2 axis = g.region.axis(), normal(-axis.y(), axis.x());
      if (std::abs((g.a - g.region.center).dot(normal)) > .001 ||
          std::abs((g.b - g.region.center).dot(normal)) > .001 ||
          (g.a - g.region.center).dot(axis) *
              (g.b - g.region.center).dot(axis) >= 0)
        return fail("invalid_project", g.id + ": A/B must lie on opposite sides of the centreline");
      for (size_t j = 0; j < i; ++j)
        if (intersects(g.region, project.passages[j].region))
          return fail("invalid_project", "special rectangles must not overlap");
    }
    auto effective = prepareEnvironment(project);
    auto ordinary = effective;
    ordinary.free = effective.free.clone();
    ordinary.soft = effective.soft.clone();
    for (const auto &g : project.passages)
      paintRegion(ordinary, g.region, false);
    cv::Mat forbidden(effective.free.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < forbidden.rows; ++y)
      for (int x = 0; x < forbidden.cols; ++x)
        for (const auto &g : project.passages)
          if (g.region.contains(effective.cellCenterToWorld(x, y))) {
            forbidden.at<uint8_t>(y, x) = 255;
            break;
          }
    auto searchCfg = project.planner;
    searchCfg.optimize = false;
    searchCfg.collect_debug_images = false;
    tdt_nav::Planner search(searchCfg), safety(searchCfg);
    search.setMap(effective, forbidden);
    safety.setMap(effective);
    auto normalFree = search.inflatedFree(), safeFree = safety.inflatedFree(),
         cost = search.costField();
    // 特殊区由机械保证中心线可通过；只检查中心线上的真实占据/未知，
    // 不用普通圆形膨胀把窄门重新封死。区外连接与释放点仍做正常膨胀检查。
    safeFree = safeFree.clone();
    for (const auto& gate : project.passages) if (gate.enabled) {
      for (int y=0; y<safeFree.rows; ++y) for (int x=0; x<safeFree.cols; ++x)
        if (gate.region.contains(effective.cellCenterToWorld(x,y)))
          safeFree.at<uint8_t>(y,x) = effective.free.at<uint8_t>(y,x);
    }
    out.input_map = project.map.free.clone();
    out.passable_map = safeFree;
    out.ordinary_map = normalFree;
    cv::Mat distance;
    cv::distanceTransform(normalFree, distance, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE);
    distance *= float(project.map.info.resolution);
    double hi = 1;
    cv::minMaxLoc(distance, nullptr, &hi);
    cv::Mat gray;
    distance.convertTo(gray, CV_8U, 255. / std::max(hi, .01));
    cv::applyColorMap(gray, out.distance_image, cv::COLORMAP_TURBO);
    out.distance_image.setTo(cv::Scalar(0, 0, 0), normalFree == 0);
    cv::Point cell;
    if (!ordinary.worldToCell(start, cell) || !normalFree.at<uint8_t>(cell))
      return fail("start_blocked", "start is outside ordinary free space (move "
                                   "it outside special rectangles/obstacles)");
    if (!ordinary.worldToCell(goal, cell) || !normalFree.at<uint8_t>(cell))
      return fail("goal_blocked", "goal is outside ordinary free space (move "
                                  "it outside special rectangles/obstacles)");
    out.nodes = {{"start", start}, {"goal", goal}};
    std::vector<EdgeData> edges;
    const double spacing =
        std::min(project.sample_spacing, project.map.info.resolution * .25);
    for (size_t i = 0; i < project.passages.size(); ++i) {
      const auto &g = project.passages[i];
      if (!g.enabled)
        continue;
      for (bool rev : {false, true}) {
        if ((rev && g.direction == "a_to_b") ||
            (!rev && g.direction == "b_to_a"))
          continue;
        Vec2 a = rev ? g.b : g.a, b = rev ? g.a : g.b, d = (b - a).normalized(),
             release = b + d * g.release_distance;
        PassageCheck check;
        check.passage_id = g.id;
        check.direction = rev ? "b_to_a" : "a_to_b";
        check.entry = a;
        check.exit = b;
        check.release = release;
        check.cost_seconds = (release - a).norm() / g.speed + g.action_cost_seconds;
        check.reason = lineProblem(effective, a, release, spacing);
        for (size_t j = 0; j < project.passages.size(); ++j)
          if (j != i && crossesRegion(project.passages[j].region, a, release))
            check.reason = "crosses_other_region: " + project.passages[j].id;
        for (const auto &end : std::array<std::pair<Vec2, std::string>, 2>{{
                 {a, "entry"}, {release, "release"}}}) {
          if (!check.reason.empty()) break;
          if (!ordinary.worldToCell(end.first, cell))
            check.reason = end.second + "_outside_map";
          else if (!normalFree.at<uint8_t>(cell))
            check.reason = end.second + "_inflation_or_forbidden";
        }
        check.valid = check.reason.empty();
        if (check.valid) check.reason = "available";
        out.passage_checks.push_back(check);
        if (!check.valid) {
          out.warnings.push_back(g.id + " " + check.direction + ": " + check.reason);
          continue;
        }
        int from = int(out.nodes.size());
        out.nodes.push_back({nodeName(g, rev, false), a});
        int to = int(out.nodes.size());
        out.nodes.push_back({nodeName(g, rev, true), release});
        EdgeData e;
        e.edge = {from,
                  to,
                  (release - a).norm() / g.speed + g.action_cost_seconds,
                  "passage",
                  g.id,
                  rev ? "b_to_a" : "a_to_b",
                  false};
        e.gate = int(i);
        e.reverse = rev;
        e.raw = sampleLine(a, release, spacing);
        e.simple = {a, b, release};
        edges.push_back(std::move(e));
      }
    }
    // Only graph geometry is searched here. OSQP is run for selected ordinary
    // edges afterwards.
    for (size_t i = 0; i < out.nodes.size(); ++i)
      for (size_t j = i + 1; j < out.nodes.size(); ++j) {
        if (tdt_nav::elapsedMs(begin) > project.graph_budget_ms)
          return fail("graph_timeout",
                      "graph search budget exceeded; increase graph_budget_ms "
                      "or reduce passage count");
        tdt_nav::Request req;
        req.start = out.nodes[i].position;
        req.goal = out.nodes[j].position;
        auto r = search.plan(req);
        if (!r.ok()) {
          if (r.status == tdt_nav::Status::SearchLimit)
            return fail("search_limit",
                        "ordinary A* reached its search budget; route "
                        "optimality not established");
          continue;
        }
        double c = pathCost(r.raw_path, ordinary, cost, project.cruise_speed);
        EdgeData e;
        e.edge = {int(i), int(j), c, "ordinary", "", "", false};
        e.raw = r.raw_path;
        e.simple = r.simplified_path;
        edges.push_back(e);
        std::swap(e.edge.from, e.edge.to);
        std::reverse(e.raw.begin(), e.raw.end());
        std::reverse(e.simple.begin(), e.simple.end());
        edges.push_back(std::move(e));
      }
    std::vector<double> dist(out.nodes.size(), 1e100);
    std::vector<int> parent(out.nodes.size(), -1);
    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> q;
    dist[0] = 0;
    q.push({0, 0});
    while (!q.empty()) {
      auto [d, u] = q.top();
      q.pop();
      if (d > dist[u] + EPS)
        continue;
      if (u == 1)
        break;
      for (size_t k = 0; k < edges.size(); ++k) {
        const auto &e = edges[k].edge;
        if (e.from != u)
          continue;
        double nd = d + e.cost;
        if (nd + EPS < dist[e.to]) {
          dist[e.to] = nd;
          parent[e.to] = int(k);
          q.push({nd, e.to});
        }
      }
    }
    if (parent[1] < 0)
      return fail(
          "no_route",
          "no route through ordinary roads and enabled directed passages");
    std::vector<int> route;
    for (int at = 1; at != 0;) {
      int k = parent[at];
      if (k < 0)
        return fail("internal_error", "invalid graph parent chain");
      route.push_back(k);
      at = edges[k].edge.from;
    }
    std::reverse(route.begin(), route.end());
    auto optCfg = project.planner;
    optCfg.collect_debug_images = false;
    tdt_nav::Planner optimizer(optCfg);
    optimizer.setMap(effective, forbidden);
    double progress = 0.;
    for (size_t ri = 0; ri < route.size(); ++ri) {
      auto &e = edges[route[ri]];
      e.edge.selected = true;
      Path selected;
      if (e.gate >= 0) {
        selected = e.raw;
        const auto &g = project.passages[e.gate];
        Vec2 b = e.reverse ? g.a : g.b;
        Traversal t;
        t.passage_id = g.id;
        t.direction = e.edge.direction;
        t.mode = g.mode;
        t.enter_s = progress;
        t.exit_s = progress + (b - selected.front()).norm();
        t.release_s = progress + length(selected);
        t.prepare_s = std::max(0., progress - g.prepare_distance);
        t.speed = g.speed;
        out.traversals.push_back(t);
      } else {
        tdt_nav::Request req;
        req.start = out.nodes[e.edge.from].position;
        req.goal = out.nodes[e.edge.to].position;
        if (ri > 0 && edges[route[ri - 1]].gate >= 0) {
          const auto &prior = edges[route[ri - 1]];
          const auto &g = project.passages[prior.gate];
          req.start_velocity =
              (prior.raw.back() - prior.raw.front()).normalized() * g.speed;
        }
        if (ri + 1 < route.size() && edges[route[ri + 1]].gate >= 0) {
          const auto &next = edges[route[ri + 1]];
          const auto &g = project.passages[next.gate];
          req.goal_velocity =
              (next.raw.back() - next.raw.front()).normalized() * g.speed;
        }
        auto r = optimizer.plan(req);
        bool accepted = r.ok();
        if (accepted) {
          double c =
              pathCost(r.final_path, ordinary, cost, project.cruise_speed);
          if (c > e.edge.cost * 1.12 + 1e-4) {
            accepted = false;
            out.warnings.push_back("ordinary smoothing increased wall-distance "
                                   "cost; retained the cost-aware A* skeleton");
          }
        }
        if (accepted) {
          selected = r.final_path;
          out.corridors.insert(out.corridors.end(), r.corridors.begin(),
                               r.corridors.end());
        } else if (project.planner.allow_geometric_fallback) {
          selected = e.raw;
          out.warnings.push_back("ordinary optimizer fallback: " + r.message);
        } else
          return fail("optimization_failed",
                      r.message + " (enable allow_geometric_fallback for a "
                                  "validated geometric route)");
        out.stages.push_back(
            {{"edge", route[ri]},
             {"kind", "ordinary"},
             {"status",
              accepted ? tdt_nav::statusName(r.status) : "raw_fallback"},
             {"search_ms", r.timing.search_ms},
             {"optimization_ms", r.timing.optimize_ms},
             {"attempts", r.attempts.size()}});
      }
      if (!selected.empty()) {
        selected.front() = out.nodes[e.edge.from].position;
        selected.back() = out.nodes[e.edge.to].position;
      }
      selected = densify(selected, project.sample_spacing);
      append(out.path, selected);
      append(out.raw_path, e.raw);
      append(out.simplified_path, e.simple);
      progress += length(selected);
    }
    // The first result point may be equal to the goal; the graph still has a
    // zero-cost ordinary edge.
    if (out.path.empty())
      out.path = {start};
    out.arc_lengths = {0.};
    for (size_t i = 1; i < out.path.size(); ++i)
      out.arc_lengths.push_back(out.arc_lengths.back() +
                                (out.path[i] - out.path[i - 1]).norm());
    out.length = out.arc_lengths.back();
    out.cost_seconds = dist[1];
    for (auto &check : out.passage_checks)
      for (const auto &t : out.traversals)
        if (check.passage_id == t.passage_id && check.direction == t.direction) {
          check.selected = true;
          check.reason = "selected";
        }
    for (const auto &t : out.traversals) {
      out.events.push_back(
          {t.passage_id, t.direction, "DEFORM", t.mode, t.prepare_s});
      out.events.push_back(
          {t.passage_id, t.direction, "ENTER", t.mode, t.enter_s});
      out.events.push_back(
          {t.passage_id, t.direction, "EXIT", t.mode, t.exit_s});
      out.events.push_back(
          {t.passage_id, t.direction, "RELEASE", "normal", t.release_s});
    }
    std::stable_sort(out.events.begin(), out.events.end(),
                     [](const auto &a, const auto &b) { return a.s < b.s; });
    for (const auto &e : edges)
      out.edges.push_back(e.edge);
    out.success = true;
    out.status = "success";
    out.message =
        "ordinary TDT routes + immutable passage lines + arc-length events";
    out.elapsed_ms = tdt_nav::elapsedMs(begin);
    return out;
  } catch (const std::exception &e) {
    return fail("invalid_project", e.what());
  }
}

ExecutionState stateAt(const Result &r, double s) {
  ExecutionState state;
  state.progress = std::clamp(s, 0., r.length);
  if (!r.success)
    return state;
  // Union of requests avoids unfolding between nearby passages whose
  // prepare/release windows overlap.
  for (const auto &t : r.traversals) {
    if (state.progress + EPS < t.prepare_s ||
        state.progress >= t.release_s - EPS)
      continue;
    bool locked = state.progress + EPS >= t.enter_s;
    if (state.fold_request && !locked)
      continue;
    state.fold_request = true;
    state.path_locked = locked;
    state.passage_id = t.passage_id;
    state.mode = t.mode;
    state.phase =
        !locked ? "PREPARE"
                : (state.progress < t.exit_s - EPS ? "PASSING" : "RELEASING");
  }
  return state;
}
Vec2 positionAt(const Result &r, double s) {
  if (r.path.empty())
    return Vec2::Zero();
  s = std::clamp(s, 0., r.length);
  auto it = std::lower_bound(r.arc_lengths.begin(), r.arc_lengths.end(), s);
  size_t i = size_t(it - r.arc_lengths.begin());
  if (i == 0)
    return r.path.front();
  if (i >= r.path.size())
    return r.path.back();
  double l = r.arc_lengths[i] - r.arc_lengths[i - 1];
  return r.path[i - 1] + (r.path[i] - r.path[i - 1]) *
                             (l > EPS ? (s - r.arc_lengths[i - 1]) / l : 0.);
}
void PassageTracker::bind(Result r, bool force) {
  if (state_.path_locked && !force)
    throw std::logic_error(
        "committed passage: path replacement is locked until release");
  if (!r.success)
    throw std::invalid_argument("cannot bind an unsuccessful route");
  result_ = std::move(r);
  progress_ = -1e-8;
  event_index_ = 0;
  state_ = {};
}
std::pair<ExecutionState, std::vector<Event>>
PassageTracker::advance(double s) {
  if (!std::isfinite(s))
    throw std::invalid_argument("progress must be finite");
  if (!result_.success)
    throw std::logic_error("bind a route first");
  double next = std::clamp(std::max(progress_, s), 0., result_.length);
  std::vector<Event> events;
  while (event_index_ < result_.events.size() &&
         result_.events[event_index_].s <= next + EPS)
    events.push_back(result_.events[event_index_++]);
  progress_ = next;
  state_ = stateAt(result_, next);
  return {state_, events};
}

Project loadProject(const std::string &filename) {
  namespace fs = std::filesystem;
  std::ifstream file(filename);
  if (!file)
    throw std::runtime_error("cannot open project: " + filename);
  Json j;
  file >> j;
  if (j.value("schema_version", 0) != 1)
    throw std::invalid_argument(
        "unsupported project schema_version (expected 1)");
  Project p;
  const auto &m = j.at("map");
  fs::path image = m.at("image").get<std::string>();
  if (image.is_relative())
    image = fs::absolute(fs::path(filename)).parent_path() / image;
  tdt_nav::MapInfo info;
  info.resolution = m.at("resolution");
  info.origin = vec(m.value("origin", Json::array({0., 0.})));
  info.yaw = m.value("yaw", 0.);
  info.frame_id = m.value("frame_id", std::string("map"));
  cv::Mat gray = tdt_image::read(image.string(), cv::IMREAD_GRAYSCALE);
  if (gray.empty())
    throw std::runtime_error("cannot load map image: " + image.string());
  if (m.value("negate", false))
    cv::bitwise_not(gray, gray);
  p.map = tdt_nav::Maps::image(gray, info, m.value("occupied_max", 65),
                               m.value("free_min", 196),
                               !m.value("allow_unknown", false));
  const auto &c = j.value("planner", Json::object());
  p.planner.robot_radius = c.value("robot_radius", .15);
  p.planner.safety_margin = c.value("safety_margin", .02);
  p.planner.obstacle_weight = c.value("obstacle_weight", .5);
  p.planner.obstacle_decay = c.value("obstacle_decay", .1);
  p.planner.optimize = c.value("optimize", true);
  p.planner.allow_geometric_fallback =
      c.value("allow_geometric_fallback", true);
  p.planner.max_speed = c.value("max_speed", 2.);
  p.planner.max_acceleration = c.value("max_acceleration", 2.);
  p.planner.corridor_range = c.value("corridor_range", .5);
  p.planner.sample_dt = c.value("sample_dt", .03);
  p.planner.optimization_iterations = c.value("optimization_iterations", 8);
  p.planner.search_timeout_ms = c.value("search_timeout_ms", 1500.);
  p.planner.optimization_timeout_ms = c.value("optimization_timeout_ms", 1500.);
  p.planner.collect_debug_images = false;
  p.cruise_speed = c.value("cruise_speed", 1.);
  p.graph_budget_ms = c.value("graph_budget_ms", 15000.);
  p.sample_spacing = c.value("sample_spacing", .025);
  for (const auto &v : j.value("passages", Json::array())) {
    Passage g;
    g.id = v.at("id");
    g.region = rect(v.at("region"));
    g.a = vec(v.at("endpoint_a"));
    g.b = vec(v.at("endpoint_b"));
    g.direction = v.value("direction", std::string("both"));
    g.enabled = v.value("enabled", true);
    g.clear_static = v.value("clear_static", true);
    g.clear_unknown = v.value("clear_unknown", false);
    g.prepare_distance = v.value("prepare_distance", .8);
    g.release_distance = v.value("release_distance", .4);
    g.speed = v.value("speed", .5);
    g.action_cost_seconds = v.value("action_cost_seconds", 1.);
    g.mode = v.value("mode", std::string("folded"));
    p.passages.push_back(g);
  }
  for (const auto &v : j.value("blockers", Json::array()))
    p.blockers.push_back(rect(v));
  return p;
}
Json toJson(const Result &r) {
  Json j = {{"schema_version", 1},
            {"success", r.success},
            {"status", r.status},
            {"message", r.message},
            {"path_id", r.path_id},
            {"length", r.length},
            {"graph_cost_seconds", r.cost_seconds},
            {"elapsed_ms", r.elapsed_ms},
            {"has_timed_trajectory", false},
            {"path", jpath(r.path)},
            {"raw_path", jpath(r.raw_path)},
            {"simplified_path", jpath(r.simplified_path)},
            {"arc_lengths", r.arc_lengths},
            {"warnings", r.warnings},
            {"stages", r.stages}};
  j["passage_checks"] = Json::array();
  for (const auto &c : r.passage_checks)
    j["passage_checks"].push_back({{"passage_id", c.passage_id},
      {"direction", c.direction}, {"valid", c.valid}, {"selected", c.selected},
      {"reason", c.reason}, {"entry", jvec(c.entry)}, {"exit", jvec(c.exit)},
      {"release", jvec(c.release)}, {"cost_seconds", c.cost_seconds}});
  j["events"] = Json::array();
  for (const auto &e : r.events)
    j["events"].push_back({{"passage_id", e.passage_id},
                           {"direction", e.direction},
                           {"type", e.type},
                           {"mode", e.mode},
                           {"s", e.s},
                           {"position", jvec(positionAt(r, e.s))}});
  j["traversals"] = Json::array();
  for (const auto &t : r.traversals)
    j["traversals"].push_back({{"passage_id", t.passage_id},
                               {"direction", t.direction},
                               {"mode", t.mode},
                               {"prepare_s", t.prepare_s},
                               {"enter_s", t.enter_s},
                               {"exit_s", t.exit_s},
                               {"release_s", t.release_s},
                               {"speed", t.speed}});
  j["graph"] = {{"nodes", Json::array()}, {"edges", Json::array()}};
  for (const auto &n : r.nodes)
    j["graph"]["nodes"].push_back(
        {{"name", n.name}, {"position", jvec(n.position)}});
  for (const auto &e : r.edges)
    j["graph"]["edges"].push_back({{"from", e.from},
                                   {"to", e.to},
                                   {"cost", e.cost},
                                   {"kind", e.kind},
                                   {"passage_id", e.passage_id},
                                   {"direction", e.direction},
                                   {"selected", e.selected}});
  j["corridors"] = Json::array();
  for (const auto &c : r.corridors) {
    Json corners = Json::array();
    for (const auto &v : c.corners)
      corners.push_back(jvec(v));
    j["corridors"].push_back(corners);
  }
  return j;
}
void saveResult(const Result &r, const std::string &filename) {
  std::ofstream f(filename);
  if (!f)
    throw std::runtime_error("cannot write result");
  f << toJson(r).dump(2) << '\n';
}
void saveDebug(const Result &r, const Project &p,
               const std::string &directory) {
  namespace fs = std::filesystem;
  fs::create_directories(directory);
  auto save = [&](const std::string &name, const cv::Mat &mat) {
    if (mat.empty())
      return;
    cv::Mat display;
    cv::flip(mat, display, 0);
    tdt_image::write((fs::path(directory) / name).string(), display);
  };
  save("00_input.png", r.input_map);
  save("01_passable.png", r.passable_map);
  save("02_ordinary.png", r.ordinary_map);
  save("03_distance.png", r.distance_image);
  cv::Mat overlay;
  cv::cvtColor(p.map.free, overlay, cv::COLOR_GRAY2BGR);
  auto pixel = [&](const Vec2 &v) {
    Vec2 c = p.map.info.toLocal(v) / p.map.info.resolution;
    return cv::Point(int(std::floor(c.x())), int(std::floor(c.y())));
  };
  auto draw = [&](const Path &path, cv::Scalar color, int width) {
    for (size_t i = 1; i < path.size(); ++i)
      cv::line(overlay, pixel(path[i - 1]), pixel(path[i]), color, width,
               cv::LINE_AA);
  };
  for (const auto &g : p.passages) {
    auto c = g.region.corners();
    for (int i = 0; i < 4; ++i)
      cv::line(overlay, pixel(c[i]), pixel(c[(i + 1) % 4]), {180, 90, 20}, 1);
  }
  draw(r.raw_path, {0, 150, 255}, 1);
  save("04_raw_path.png", overlay);
  draw(r.simplified_path, {50, 190, 20}, 1);
  save("05_simplified.png", overlay);
  for (const auto &c : r.corridors)
    for (int i = 0; i < 4; ++i)
      cv::line(overlay, pixel(c.corners[i]), pixel(c.corners[(i + 1) % 4]),
               {220, 130, 0}, 1);
  save("06_corridors.png", overlay);
  draw(r.path, {40, 50, 240}, 2);
  for (const auto &e : r.events)
    cv::circle(overlay, pixel(positionAt(r, e.s)), 3, {200, 30, 180}, -1);
  save("07_final_events.png", overlay);
  saveResult(r, (fs::path(directory) / "result.json").string());
  std::ofstream csv(fs::path(directory) / "path.csv");
  csv << "s,x,y,fold_request,path_locked,phase,passage_id\n";
  for (size_t i = 0; i < r.path.size(); ++i) {
    auto state = stateAt(r, r.arc_lengths[i]);
    csv << std::setprecision(12) << r.arc_lengths[i] << ',' << r.path[i].x()
        << ',' << r.path[i].y() << ',' << state.fold_request << ','
        << state.path_locked << ',' << state.phase << ',' << state.passage_id
        << '\n';
  }
}
} // namespace tdt_passage
