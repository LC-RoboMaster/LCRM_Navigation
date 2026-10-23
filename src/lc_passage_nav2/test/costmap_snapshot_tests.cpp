#include "lc_passage_nav2/costmap_snapshot.hpp"
#include <iostream>
#include <stdexcept>
using namespace lc_passage_nav2;
namespace {
void check(bool value, const std::string &message) {
  if (!value)
    throw std::runtime_error(message);
}
tdt_passage::Project field() {
  tdt_passage::Project p;
  tdt_nav::MapInfo info;
  info.resolution = .05;
  cv::Mat image(160, 240, CV_8UC1, cv::Scalar(255));
  image.row(0).setTo(0);
  image.row(159).setTo(0);
  image.col(0).setTo(0);
  image.col(239).setTo(0);
  image.colRange(117, 123).setTo(0);
  p.map = tdt_nav::Maps::binary(image, info, tdt_nav::RowOrder::BottomUp);
  tdt_passage::Passage g;
  g.id = "gate";
  g.region.center = {6, 4};
  g.region.length = 1.;
  g.region.width = 1.2;
  g.a = {4.9, 4};
  g.b = {7.1, 4};
  p.passages = {g};
  p.planner.robot_radius = .15;
  p.planner.safety_margin = .01;
  p.planner.optimize = false;
  p.planner.collect_debug_images = false;
  return p;
}
CostmapSnapshot emptySnapshot(const tdt_nav::GridMap &map) {
  CostmapSnapshot s;
  s.width = map.free.cols;
  s.height = map.free.rows;
  s.resolution = map.info.resolution;
  s.origin_x = map.info.origin.x();
  s.origin_y = map.info.origin.y();
  s.cells.resize(size_t(s.width) * s.height, 0);
  return s;
}
} // namespace
int main() {
  int n = 0;
  auto run = [&](const char *name, auto test) {
    test();
    ++n;
    std::cout << "PASS " << name << '\n';
  };
  try {
    run("row_order_and_physical_lethal", [] {
      auto p = field();
      auto s = emptySnapshot(p.map);
      s.cells[3 * s.width + 2] = 254;
      auto mask = runtimeMask(p.map, s);
      check(mask.at<uint8_t>(3, 2) == 255, "bottom-up cell must not flip");
      check(cv::countNonZero(mask) == 1, "one physical obstacle");
    });
    run("inflation_is_not_applied_twice", [] {
      auto p = field();
      auto s = emptySnapshot(p.map);
      s.cells[10] = 253;
      s.cells[11] = 180;
      check(cv::countNonZero(runtimeMask(p.map, s)) == 0,
            "inflation costs must not become physical occupancy");
    });
    run("unknown_policy", [] {
      auto p = field();
      auto s = emptySnapshot(p.map);
      s.cells[10] = 255;
      check(cv::countNonZero(runtimeMask(p.map, s, true)) == 1,
            "unknown blocked");
      check(cv::countNonZero(runtimeMask(p.map, s, false)) == 0,
            "explicit unknown policy");
    });
    run("alignment_rejected", [] {
      auto p = field();
      auto base = emptySnapshot(p.map);
      for (int i = 0; i < 4; ++i) {
        auto s = base;
        if (i == 0)
          s.width++;
        if (i == 1)
          s.origin_x = .01;
        if (i == 2)
          s.resolution = .1;
        if (i == 3)
          s.cells.pop_back();
        bool rejected = false;
        try {
          runtimeMask(p.map, s);
        } catch (const std::invalid_argument &) {
          rejected = true;
        }
        check(rejected, "misaligned snapshots must be rejected");
      }
    });
    run("rotated_map_rejected_only_at_nav2_boundary", [] {
      auto p = field();
      p.map.info.yaw = .2;
      auto s = emptySnapshot(p.map);
      bool rejected = false;
      try {
        runtimeMask(p.map, s);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      check(rejected, "Nav2 boundary does not silently ignore map yaw");
    });
    run("live_obstacle_survives_passage_clearing", [] {
      auto p = field();
      auto s = emptySnapshot(p.map);
      for (unsigned int y = 65; y < 95; ++y)
        s.cells[y * s.width + 120] = 254;
      p.runtime_obstacles = runtimeMask(p.map, s);
      auto r = tdt_passage::Planner().plan(p, {2, 4}, {10, 4});
      check(!r.success, "live barrier through gate must block passage");
    });
    run("physical_export_has_opening_without_inflation", [] {
      auto p = field();
      auto raw = p.map.free.clone();
      auto map = tdt_passage::prepareEnvironment(p);
      check(map.free.at<uint8_t>(80, 120) == 255, "static beam removed");
      check(map.free.at<uint8_t>(30, 120) == 0, "wall retained");
      check(cv::countNonZero(raw != p.map.free) == 0,
            "source static map not mutated");
      auto r = tdt_passage::Planner().plan(p, {2, 4}, {10, 4});
      check(r.success, r.message);
    });
    run("bad_mask_reported", [] {
      auto p = field();
      p.runtime_obstacles = cv::Mat(2, 2, CV_8UC1, cv::Scalar(1));
      auto r = tdt_passage::Planner().plan(p, {2, 4}, {10, 4});
      check(!r.success && r.status == "invalid_project",
            "bad mask is not silently ignored");
    });
    std::cout << "All " << n << " adapter scenarios passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
