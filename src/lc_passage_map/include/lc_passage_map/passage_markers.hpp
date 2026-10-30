#pragma once
// RViz only: these markers never change occupancy or passage execution state.
#include "lc_passage_common/passage_config.hpp"
#include "lc_passage_common/marker_utils.hpp"
#include <lc_passage_msgs/msg/passage_plan.hpp>
#include <lc_passage_msgs/msg/passage_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>

namespace lc_passage_map {
using lc_passage_common::Point;
using lc_passage_common::PassageFile;
using lc_passage_common::MarkerArray;
using lc_passage_common::Marker;
using lc_passage_common::markerColor;
using lc_passage_common::markerPoint;
using lc_passage_common::emptyMarkers;
using lc_passage_common::addMarker;
using lc_passage_common::markerText;
using lc_passage_common::markerLine;
// Stop markers must be interpolated on the ADOPTED path: prepare_s is arc
// length, not necessarily a point on the straight extension behind A/B.
inline Point planPoint(const lc_passage_msgs::msg::PassagePlan &p, double s) {
  auto it = std::lower_bound(p.arc_lengths.begin(), p.arc_lengths.end(), s);
  size_t i = std::min<size_t>(it - p.arc_lengths.begin(), p.path.poses.size() - 1);
  const auto &b = p.path.poses[i].pose.position;
  if (!i) return {b.x, b.y};
  const auto &a = p.path.poses[i - 1].pose.position;
  const double ds = p.arc_lengths[i] - p.arc_lengths[i - 1];
  const double t = ds > 1e-9 ? std::clamp((s - p.arc_lengths[i - 1]) / ds, 0., 1.) : 0.;
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}
inline MarkerArray passageMarkers(const PassageFile &f,
    const lc_passage_msgs::msg::PassagePlan &plan = lc_passage_msgs::msg::PassagePlan{},
    const lc_passage_msgs::msg::PassageState &state = lc_passage_msgs::msg::PassageState{}) {
  auto out = emptyMarkers(f.frame);
  for (const auto &g : f.passages) {
    auto color = g.enabled ? markerColor(.15f, .7f, 1.f) : markerColor(.5f, .5f, .5f);
    if (state.passage_id == g.id && state.locked)
      color = state.fault ? markerColor(1.f, .15f, .1f) : markerColor(.2f, 1.f, .3f);
    auto &region = addMarker(out, f.frame, "regions", Marker::CUBE, g.center, .035, color);
    region.color.a = .22f;
    region.scale.x = g.length; region.scale.y = g.width; region.scale.z = .025;
    region.pose.orientation.z = std::sin(g.yaw / 2);
    region.pose.orientation.w = std::cos(g.yaw / 2);
    const double c = std::cos(g.yaw), s = std::sin(g.yaw);
    auto local = [&](double x, double y) { return Point{g.center.x + c*x - s*y, g.center.y + s*x + c*y}; };
    auto &outline = addMarker(out, f.frame, "outlines", Marker::LINE_STRIP, {}, 0., color);
    outline.scale.x = .035;
    for (auto xy : std::array<Point, 5>{{{-1,-1},{1,-1},{1,1},{-1,1},{-1,-1}}})
      outline.points.push_back(markerPoint(local(xy.x*g.length/2, xy.y*g.width/2), .06));
    for (const auto &end : std::array<std::pair<Point, std::string>, 2>{{{g.a,"A"},{g.b,"B"}}}) {
      auto &m = addMarker(out, f.frame, "endpoints", Marker::SPHERE, end.first, .10,
                          end.second == "A" ? markerColor(0.f, 1.f, 1.f) : markerColor(1.f, .3f, 1.f));
      m.scale.x = m.scale.y = m.scale.z = .13;
      markerText(out, f.frame, "endpoint_labels", end.first, .3, g.id + " " + end.second, color);
    }
    markerLine(out, f.frame, "centerlines", g.a, g.b, .08, color);
    // Short arrows stay separated for a bidirectional passage.
    const Point middle{(g.a.x+g.b.x)/2, (g.a.y+g.b.y)/2};
    if (g.enabled && g.direction != "b_to_a")
      markerLine(out, f.frame, "directions", g.a, middle, .13, color, true);
    if (g.enabled && g.direction != "a_to_b")
      markerLine(out, f.frame, "directions", g.b, middle, .13, color, true);
    std::ostringstream label;
    label << g.id << " [" << (g.enabled ? g.direction : "disabled") << "]\n"
          << std::fixed << std::setprecision(2) << "v=" << g.speed
          << " m/s  beam=" << g.beam_min << ".." << g.beam_max << " m";
    if (g.clear_static && g.clear_unknown) label << "\nclear static unknown";
    if (state.passage_id == g.id && !state.phase.empty()) label << "\n" << state.phase;
    markerText(out, f.frame, "info", local(0., g.width/2+.25), .32, label.str(), color);
  }
  if (plan.header.frame_id != f.frame || plan.path.poses.empty() ||
      plan.arc_lengths.size() != plan.path.poses.size() ||
      !std::is_sorted(plan.arc_lengths.begin(), plan.arc_lengths.end())) return out;
  for (const auto &t : plan.traversals) {
    markerLine(out, f.frame, "adopted_centerlines", planPoint(plan,t.enter_s),
               planPoint(plan,t.release_s), .18, markerColor(.2f,1.f,.2f));
    const std::array<std::pair<double,std::string>,4> events{{
      {t.prepare_s,"PREPARE / FOLD"},{t.enter_s,"ENTER"},
      {t.exit_s,"EXIT"},{t.release_s,"RELEASE / RESTORE"}}};
    for (size_t i=0;i<events.size();++i) {
      auto p=planPoint(plan,events[i].first);
      auto color=i==0 ? markerColor(1.f,.7f,.05f) :
                 i==3 ? markerColor(.2f,1.f,.3f) : markerColor(1.f,1.f,1.f);
      auto &m=addMarker(out,f.frame,"events",Marker::SPHERE,p,.17,color);
      m.scale.x=m.scale.y=m.scale.z=.15;
      markerText(out,f.frame,"event_labels",p,.48,t.passage_id+" "+events[i].second,color);
    }
  }
  return out;
}
} // namespace lc_passage_map
