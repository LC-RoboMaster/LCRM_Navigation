#pragma once
// 通道文件只保存世界坐标语义，不依赖地图像素尺寸，SLAM 扩图后仍可使用。
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace lc_passage_nav2 {
struct Point {
  double x = 0, y = 0;
};
inline double distance(Point a, Point b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
struct PassageConfig {
  std::string id, direction = "both", mode = "folded";
  Point center, a, b;
  double length = 1, width = 1, yaw = 0, prepare = .8, release = .4, speed = .3,
         action_cost = 1;
  double fold_hold = .3, restore_hold = .3, beam_min = .26, beam_max = 2.;
  bool enabled = true, clear_static = true;
  bool contains(Point p) const {
    const double dx = p.x - center.x, dy = p.y - center.y, c = std::cos(yaw),
                 s = std::sin(yaw);
    return std::abs(c * dx + s * dy) <= length / 2 &&
           std::abs(-s * dx + c * dy) <= width / 2;
  }
};
struct PassageFile {
  std::string frame = "map";
  std::vector<PassageConfig> passages;
};
inline PassageFile loadPassages(const std::string &filename) {
  PassageFile out;
  if (filename.empty())
    return out; // 空文件路径是正常的在线建图导航模式。
  std::ifstream f(filename);
  if (!f)
    throw std::runtime_error("Cannot open passage_file: " + filename);
  nlohmann::json j;
  f >> j;
  if (j.value("schema_version", 0) != 1)
    throw std::runtime_error("Unsupported passage schema_version");
  out.frame = j.value("frame_id", j.value("map", nlohmann::json::object())
                                      .value("frame_id", std::string("map")));
  if (out.frame.empty() || !j.contains("passages") ||
      !j.at("passages").is_array() || j.at("passages").size() > 16)
    throw std::runtime_error("Invalid passage frame/count (maximum 16)");
  auto point = [](const nlohmann::json &v) {
    if (!v.is_array() || v.size() != 2)
      throw std::runtime_error("Point must be [x,y]");
    Point p{v[0].get<double>(), v[1].get<double>()};
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::runtime_error("Nonfinite point");
    return p;
  };
  std::set<std::string> ids;
  for (const auto &v : j.at("passages")) {
    PassageConfig p;
    p.id = v.at("id").get<std::string>();
    if (p.id.empty() || !ids.insert(p.id).second)
      throw std::runtime_error("Empty/duplicate passage id");
    const auto &r = v.at("region");
    p.center = point(r.at("center"));
    p.length = r.at("length");
    p.width = r.at("width");
    p.yaw = r.at("yaw");
    p.a = point(v.at("endpoint_a"));
    p.b = point(v.at("endpoint_b"));
    p.direction = v.value("direction", p.direction);
    p.mode = v.value("mode", p.mode);
    p.enabled = v.value("enabled", true);
    p.clear_static = v.value("clear_static", true);
    p.prepare = v.value("prepare_distance", p.prepare);
    p.release = v.value("release_distance", p.release);
    p.speed = v.value("speed", p.speed);
    p.action_cost = v.value("action_cost_seconds", p.action_cost);
    p.fold_hold = v.value("fold_hold_seconds", p.fold_hold);
    p.restore_hold = v.value("restore_hold_seconds", p.restore_hold);
    p.beam_min = v.value("beam_min_height", p.beam_min);
    p.beam_max = v.value("beam_max_height", p.beam_max);
    for (double n : {p.length, p.width, p.speed, p.prepare, p.release,
                     p.beam_min, p.beam_max})
      if (!std::isfinite(n) || n <= 0)
        throw std::runtime_error("Invalid positive passage parameter: " + p.id);
    for (double n : {p.action_cost, p.fold_hold, p.restore_hold})
      if (!std::isfinite(n) || n < 0)
        throw std::runtime_error("Invalid hold/cost: " + p.id);
    if (!std::isfinite(p.yaw) || p.mode != "folded" ||
        p.beam_min >= p.beam_max || distance(p.a, p.b) < 1e-5 ||
        (p.direction != "both" && p.direction != "a_to_b" &&
         p.direction != "b_to_a"))
      throw std::runtime_error("Invalid direction/mode/geometry: " + p.id);
    // A/B 必须落在矩形长轴上且分居两端，不能用斜穿矩形的线冒充中线。
    double sa = 0, sb = 0;
    for (const auto &end : {p.a, p.b}) {
      double dx = end.x - p.center.x, dy = end.y - p.center.y;
      double along = std::cos(p.yaw) * dx + std::sin(p.yaw) * dy;
      double across = -std::sin(p.yaw) * dx + std::cos(p.yaw) * dy;
      if (std::abs(across) > 1e-3 || std::abs(along) <= p.length / 2)
        throw std::runtime_error(
            "Endpoints must be outside the region on its centreline: " + p.id);
      if (sa == 0)
        sa = along;
      else
        sb = along;
    }
    if (sa * sb >= 0)
      throw std::runtime_error("Endpoints must be on opposite sides: " + p.id);
    // SAT
    // 检查旋转矩形重叠；同一规则同时用于地图过滤和规划，避免无效配置先清地图。
    for (const auto &other : out.passages) {
      bool separated = false;
      for (double angle : {p.yaw, p.yaw + 1.5707963267948966, other.yaw,
                           other.yaw + 1.5707963267948966}) {
        double c = std::cos(angle), ss = std::sin(angle);
        auto extent = [&](const PassageConfig &r) {
          return std::abs(std::cos(r.yaw) * c + std::sin(r.yaw) * ss) *
                     r.length / 2 +
                 std::abs(-std::sin(r.yaw) * c + std::cos(r.yaw) * ss) *
                     r.width / 2;
        };
        if (std::abs((p.center.x - other.center.x) * c +
                     (p.center.y - other.center.y) * ss) >
            extent(p) + extent(other) + 1e-8)
          separated = true;
      }
      if (!separated)
        throw std::runtime_error("Passage regions overlap: " + p.id + " / " +
                                 other.id);
    }
    out.passages.push_back(p);
  }
  return out;
}
// 仅对标注的梁高区间进行实时点云过滤，地面和低矮物体始终保留。
inline bool isBeam(const PassageFile &f, Point p, double relative_height) {
  for (const auto &gate : f.passages)
    if (gate.enabled && gate.clear_static && gate.contains(p) &&
        relative_height >= gate.beam_min && relative_height <= gate.beam_max)
      return true;
  return false;
}
} // namespace lc_passage_nav2
