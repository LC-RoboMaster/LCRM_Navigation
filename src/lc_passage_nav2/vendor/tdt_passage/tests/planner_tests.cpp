#include <iostream>
#include <stdexcept>
#include <tdt_passage/semantic_planner.hpp>

using namespace tdt_passage;
void require(bool value, const std::string &why) {
  if (!value)
    throw std::runtime_error(why);
}
Project field(double yaw = 0.) {
  Project p;
  tdt_nav::MapInfo info;
  info.resolution = .05;
  cv::Mat image(160, 240, CV_8U, cv::Scalar(255));
  image.row(0).setTo(0);
  image.row(159).setTo(0);
  image.col(0).setTo(0);
  image.col(239).setTo(0);
  image.colRange(117, 123).setTo(0);
  p.map = tdt_nav::Maps::binary(image, info, tdt_nav::RowOrder::BottomUp);
  Passage g;
  g.id = "door";
  g.region.center = {6., 4.};
  g.region.length = 1.;
  g.region.width = 1.2;
  g.a = {4.9, 4.};
  g.b = {7.1, 4.};
  g.prepare_distance = .7;
  g.release_distance = .4;
  g.action_cost_seconds = .2;
  p.passages = {g};
  p.planner.robot_radius = .15;
  p.planner.safety_margin = .01;
  p.planner.optimize = false;
  p.planner.allow_geometric_fallback = true;
  p.planner.collect_debug_images = false;
  p.map.info.yaw = yaw;
  p.map.info.origin = {-2., -1.};
  for (auto &gate : p.passages) {
    gate.region.center = p.map.info.toWorld(gate.region.center);
    gate.region.yaw += yaw;
    gate.a = p.map.info.toWorld(gate.a);
    gate.b = p.map.info.toWorld(gate.b);
  }
  return p;
}
int main() {
  int count = 0;
  auto run = [&](const std::string &name, auto fn) {
    try {
      fn();
      ++count;
      std::cout << "PASS " << name << '\n';
    } catch (const std::exception &e) {
      std::cerr << "FAIL " << name << ": " << e.what() << '\n';
      throw;
    }
  };
  try {
    run("forward_fixed_line_and_events", [] {
      auto p = field();
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(r.success, r.message);
      require(r.traversals.size() == 1, "one selected passage");
      require(r.events.size() == 4, "four milestones");
      auto t = r.traversals[0];
      require(t.prepare_s < t.enter_s && t.enter_s < t.exit_s &&
                  t.exit_s < t.release_s,
              "ordered milestones");
      for (size_t i = 0; i < r.path.size(); ++i)
        if (r.arc_lengths[i] >= t.enter_s && r.arc_lengths[i] <= t.release_s)
          require(std::abs(r.path[i].y() - 3.) < 1e-6,
                  "passage is an exact line");
      require(stateAt(r, t.prepare_s + .01).fold_request,
              "fold starts before entry");
      require(!stateAt(r, t.prepare_s + .01).path_locked,
              "pre-entry replaceable");
      require(stateAt(r, t.enter_s + .01).path_locked, "crossing locked");
      require(!stateAt(r, t.release_s).fold_request, "release restores normal");
    });
    run("reverse_direction_and_release_side", [] {
      auto p = field();
      auto r = Planner().plan(p, p.map.info.toWorld({10, 4}),
                              p.map.info.toWorld({2, 4}));
      require(r.success, r.message);
      require(r.traversals[0].direction == "b_to_a", "direction flips");
      require((positionAt(r, r.traversals[0].exit_s) - p.passages[0].a).norm() <
                  1e-5,
              "reverse exits at A");
      require(positionAt(r, r.traversals[0].release_s).x() <
                  p.passages[0].a.x(),
              "release is on reverse exit side");
    });
    run("one_way_blocks_reverse", [] {
      auto p = field();
      p.passages[0].direction = "a_to_b";
      auto r = Planner().plan(p, p.map.info.toWorld({10, 4}),
                              p.map.info.toWorld({2, 4}));
      require(!r.success && r.status == "no_route",
              "reverse must not bypass semantic edge");
    });
    run("disabled_passage_is_not_ordinary_free_space", [] {
      auto p = field();
      p.passages[0].enabled = false;
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(!r.success, "disabled door blocks crossing");
    });
    run("dynamic_obstacle_not_cleared", [] {
      auto p = field();
      RotatedRect b;
      b.center = p.passages[0].region.center;
      b.length = .5;
      b.width = 1.2;
      p.blockers = {b};
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(!r.success, "dynamic blocker vetoes fixed edge");
      require(!r.warnings.empty(), "blocked edge diagnostic");
    });
    run("map_rotation_and_origin", [] {
      auto p = field(.47);
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(r.success, r.message);
      auto t = r.traversals[0];
      Vec2 d = p.passages[0].region.axis(), n(-d.y(), d.x());
      for (size_t i = 0; i < r.path.size(); ++i)
        if (r.arc_lengths[i] >= t.enter_s && r.arc_lengths[i] <= t.release_s)
          require(std::abs((r.path[i] - p.passages[0].a).dot(n)) < 1e-5,
                  "rotated line remains exact");
    });
    run("ordinary_path_does_not_trigger_door", [] {
      auto p = field();
      auto r = Planner().plan(p, p.map.info.toWorld({2, 2}),
                              p.map.info.toWorld({4, 2}));
      require(r.success, r.message);
      require(r.traversals.empty(), "nearby door not selected");
    });
    run("multiple_passages_chain", [] {
      auto p = field();
      p.map.free.colRange(177, 183).setTo(0);
      auto g = p.passages[0];
      g.id = "second";
      g.region.center.x() += 3.;
      g.a.x() += 3.;
      g.b.x() += 3.;
      g.prepare_distance = 4.; // its preparation overlaps the first crossing
      p.passages.push_back(g);
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10.8, 4}));
      require(r.success, r.message);
      require(r.traversals.size() == 2, "route traverses both walls");
      require(r.traversals[0].passage_id == "door" &&
                  r.traversals[1].passage_id == "second",
              "correct graph order");
      require(r.traversals[1].prepare_s < r.traversals[0].release_s,
              "fold windows really overlap in this scenario");
      for (double s = r.traversals[0].enter_s;
           s < r.traversals[1].release_s - .01; s += .01)
        require(stateAt(r, s).fold_request,
                "overlapping windows never restore between doors");
    });
    run("overlapping_regions_rejected", [] {
      auto p = field();
      auto g = p.passages[0];
      g.id = "duplicate_region";
      p.passages.push_back(g);
      auto r = Planner().plan(p, {0, 0}, {1, 1});
      require(!r.success && r.status == "invalid_project", "overlap rejected");
    });
    run("unknown_cells_not_opened_by_static_annotation", [] {
      auto p = field();
      p.map.unknown.colRange(119, 121).setTo(255);
      p.map.free.colRange(119, 121).setTo(0);
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(!r.success, "unknown barrier remains unknown/blocked");
    });
    run("tracker_latches_progress_and_replacement_lock", [] {
      auto p = field();
      auto r = Planner().plan(p, p.map.info.toWorld({2, 4}),
                              p.map.info.toWorld({10, 4}));
      require(r.success, r.message);
      PassageTracker tracker;
      tracker.bind(r);
      auto t = r.traversals[0];
      auto [a, events] = tracker.advance(t.enter_s + .1);
      require(a.path_locked && events.size() == 2, "deform and enter events");
      auto [b, again] = tracker.advance(t.enter_s - .1);
      require(b.path_locked && again.empty(),
              "noise cannot roll back or repeat events");
      require(tracker.advance(t.enter_s + .1 + 1e-10).second.empty(),
              "tiny forward movement does not repeat boundary events");
      bool threw = false;
      try {
        tracker.bind(r);
      } catch (const std::logic_error &) {
        threw = true;
      }
      require(threw, "committed route replacement rejected");
      tracker.advance(t.release_s);
      tracker.bind(r);
    });
    run("same_start_goal", [] {
      auto p = field();
      auto q = p.map.info.toWorld({2, 2});
      auto r = Planner().plan(p, q, q);
      require(r.success && r.path.size() == 1 && r.length == 0,
              "zero length route");
    });
    run("TDT_OSQP_selected_ordinary_edges", [] {
      auto p = field();
      p.planner.optimize = true;
      p.planner.optimization_iterations = 5;
      auto r = Planner().plan(p, p.map.info.toWorld({2, 3.8}),
                              p.map.info.toWorld({10, 4.2}));
      require(r.success, r.message);
      require(r.traversals.size() == 1, "semantic line kept through optimizer");
      require(r.stages.size() >= 2, "ordinary optimization diagnostics");
    });
    std::cout << "All " << count << " core regression scenarios passed.\n";
    return 0;
  } catch (...) {
    return 1;
  }
}
