#pragma once
#include "lc_passage_common/geometry.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>

namespace lc_passage_common {
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
inline std_msgs::msg::ColorRGBA markerColor(float r, float g, float b, float a = 1.f) {
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}
inline geometry_msgs::msg::Point markerPoint(Point p, double z) {
  geometry_msgs::msg::Point q;
  q.x = p.x; q.y = p.y; q.z = z;
  return q;
}
// Each array is a complete snapshot. DELETEALL removes a previous longer plan
// or a previous configuration when RViz reconnects to this durable topic.
inline MarkerArray emptyMarkers(const std::string &frame) {
  MarkerArray a;
  Marker m;
  m.header.frame_id = frame;
  m.action = Marker::DELETEALL;
  a.markers.push_back(m);
  return a;
}
inline Marker &addMarker(MarkerArray &a, const std::string &frame,
                         const std::string &ns, int type, Point p,
                         double z, std_msgs::msg::ColorRGBA color) {
  Marker m;
  m.header.frame_id = frame; // stamp zero: fixed map geometry, no old-TF requirement
  m.ns = ns;
  m.id = static_cast<int>(a.markers.size());
  m.type = type;
  m.action = Marker::ADD;
  m.pose.position = markerPoint(p, z);
  m.pose.orientation.w = 1.;
  m.color = color;
  a.markers.push_back(m);
  return a.markers.back();
}
inline void markerText(MarkerArray &a, const std::string &frame,
                        const std::string &ns, Point p, double z,
                        const std::string &text, std_msgs::msg::ColorRGBA color) {
  auto &m = addMarker(a, frame, ns, Marker::TEXT_VIEW_FACING, p, z, color);
  m.scale.z = .18;
  m.text = text;
}
inline void markerLine(MarkerArray &a, const std::string &frame,
                        const std::string &ns, Point from, Point to,
                        double z, std_msgs::msg::ColorRGBA color,
                        bool arrow = false) {
  auto &m = addMarker(a, frame, ns, arrow ? Marker::ARROW : Marker::LINE_STRIP,
                      {}, 0., color);
  m.points = {markerPoint(from, z), markerPoint(to, z)};
  m.scale.x = .035;
  if (arrow) { m.scale.y = .10; m.scale.z = .13; }
}
}
