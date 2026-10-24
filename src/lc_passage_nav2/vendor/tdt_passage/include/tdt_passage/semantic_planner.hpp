#pragma once

// Standalone C++ API. No ROS, Nav2, Qt or Python types enter the planner.
#include "planner.hpp"
#include <nlohmann/json.hpp>
#include <optional>

namespace tdt_passage {
using Json = nlohmann::json;
using Vec2 = tdt_nav::Vec2;
using Path = tdt_nav::Path;

struct RotatedRect {
  Vec2 center = Vec2::Zero();
  double length = 1., width = 1., yaw = 0.; // metres, radians, map/world axes
  Vec2 axis() const;
  double signedDistance(const Vec2 &p) const;
  bool contains(const Vec2 &p, double padding = 0.) const;
  std::array<Vec2, 4> corners() const;
};

struct Passage {
  std::string id;
  RotatedRect region;
  Vec2 a = Vec2::Zero(), b = Vec2::Zero(); // ordinary-search connection points
  std::string direction = "both";          // both / a_to_b / b_to_a
  bool enabled = true, clear_static = true, clear_unknown = false;
  double prepare_distance = .8, release_distance = .4;
  double speed = .5, action_cost_seconds = 1.;
  std::string mode = "folded";
};

struct Project {
  tdt_nav::GridMap map;
  tdt_nav::Config planner;
  std::vector<Passage> passages;
  std::vector<RotatedRect> blockers; // never cleared by passage semantics
  // Optional current obstacles in the same bottom-up grid as map.free.
  // CV_8UC1: zero means no additional obstacle, nonzero means blocked.
  // Applied AFTER static passage clearing, so live obstacles cannot disappear.
  cv::Mat runtime_obstacles;
  double cruise_speed = 1., graph_budget_ms = 15000., sample_spacing = .025;
};

struct Traversal {
  std::string passage_id, direction, mode;
  double prepare_s = 0., enter_s = 0., exit_s = 0., release_s = 0., speed = 0.;
};

struct Event {
  std::string passage_id, direction, type, mode;
  double s = 0.;
};

struct GraphNode {
  std::string name;
  Vec2 position;
};
struct GraphEdge {
  int from = 0, to = 0;
  double cost = 0.;
  std::string kind, passage_id, direction;
  bool selected = false;
};

// Inspectable even when graph search fails. A usable edge need not be selected:
// the complete route also includes ordinary-road costs and deformation cost.
struct PassageCheck {
  std::string passage_id, direction, reason;
  Vec2 entry = Vec2::Zero(), exit = Vec2::Zero(), release = Vec2::Zero();
  bool valid = false, selected = false;
  double cost_seconds = 0.;
};

struct Result {
  bool success = false;
  std::string status = "not_planned", message;
  uint64_t path_id = 0;
  double length = 0., cost_seconds = 0., elapsed_ms = 0.;
  Path path, raw_path, simplified_path;
  std::vector<double> arc_lengths;
  std::vector<tdt_nav::Corridor> corridors;
  std::vector<Traversal> traversals;
  std::vector<Event> events;
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
  std::vector<PassageCheck> passage_checks;
  std::vector<std::string> warnings;
  Json stages = Json::array();
  cv::Mat input_map, passable_map, ordinary_map, distance_image;
};

// Computes a graph route, optimizes its ordinary edges using the uploaded TDT
// wrapper, then expands selected passage edges as immutable straight segments.
class Planner {
public:
  Result plan(const Project &project, const Vec2 &start,
              const Vec2 &goal) const;
};

struct ExecutionState {
  bool fold_request = false, path_locked = false;
  std::string phase = "NORMAL", passage_id, mode = "normal";
  double progress = 0.;
};

// Pure query for editor scrubbing; no timer or distance-to-centre triggers.
ExecutionState stateAt(const Result &result, double progress);
Vec2 positionAt(const Result &result, double progress);

// Runtime example: phase never regresses due to small projection noise, events
// are emitted once, and replacing a committed path requires an explicit force.
class PassageTracker {
public:
  void bind(Result result, bool force = false);
  std::pair<ExecutionState, std::vector<Event>> advance(double progress);
  bool hasPath() const { return result_.success; }

private:
  Result result_;
  double progress_ = -1e-8;
  ExecutionState state_;
  size_t event_index_ = 0;
};

Project loadProject(const std::string &filename);
// Physical occupancy before robot inflation and before ordinary-only passage
// masking. Also used to export a consistent Nav2 static map.
tdt_nav::GridMap prepareEnvironment(const Project &project);
Json toJson(const Result &result);
void saveResult(const Result &result, const std::string &filename);
void saveDebug(const Result &result, const Project &project,
               const std::string &directory);
} // namespace tdt_passage
