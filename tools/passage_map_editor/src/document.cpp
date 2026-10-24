#include "document.hpp"
#include <QBuffer>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QSaveFile>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace passage_editor {
namespace {
[[noreturn]] void error(const QString &s) {
  throw std::runtime_error(s.toStdString());
}
QJsonObject merge(QJsonObject base, const QJsonObject &other) {
  for (auto i = other.begin(); i != other.end(); ++i)
    base[i.key()] = i.value();
  return base;
}
void saveImage(const QImage &image, const QString &path) {
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly) || !image.save(&file, "PNG") ||
      !file.commit())
    error("无法保存地图：" + path);
}
void validateMap(const QJsonObject &m) {
  if (!std::isfinite(m["resolution"].toDouble()) ||
      m["resolution"].toDouble() <= 0 || m["origin"].toArray().size() != 2 ||
      m["occupied_max"].toInt() >= m["free_min"].toInt())
    error("地图分辨率、原点或灰度阈值无效");
}
} // namespace
QPointF point(const QJsonValue &v) {
  auto a = v.toArray();
  return {a[0].toDouble(), a[1].toDouble()};
}
QJsonArray jsonPoint(const QPointF &p) { return {p.x(), p.y()}; }
QPointF axis(double a) { return {std::cos(a), std::sin(a)}; }
double norm(const QPointF &p) { return std::hypot(p.x(), p.y()); }
double dot(const QPointF &a, const QPointF &b) {
  return a.x() * b.x() + a.y() * b.y();
}
QVector<QPointF> corners(const QJsonObject &r) {
  auto c = point(r["center"]),
       d = axis(r["yaw"].toDouble()) * (r["length"].toDouble() / 2),
       n = axis(r["yaw"].toDouble() + Pi / 2) * (r["width"].toDouble() / 2);
  return {c - d - n, c + d - n, c + d + n, c - d + n};
}
bool contains(const QJsonObject &r, QPointF p) {
  auto v = p - point(r["center"]);
  double yaw = r["yaw"].toDouble();
  return std::abs(dot(v, axis(yaw))) <= r["length"].toDouble() / 2 &&
         std::abs(dot(v, axis(yaw + Pi / 2))) <= r["width"].toDouble() / 2;
}
QJsonObject defaultProject() {
  return {{"schema_version", 1},
          {"map", QJsonObject{{"image", ""},
                              {"resolution", .05},
                              {"origin", QJsonArray{0., 0.}},
                              {"yaw", 0.},
                              {"frame_id", "map"},
                              {"occupied_max", 65},
                              {"free_min", 196},
                              {"allow_unknown", false}}},
          {"planner", QJsonObject{{"robot_radius", .15},
                                  {"safety_margin", .02},
                                  {"obstacle_weight", .5},
                                  {"obstacle_decay", .1},
                                  {"optimize", true},
                                  {"allow_geometric_fallback", true},
                                  {"max_speed", 2.},
                                  {"max_acceleration", 2.},
                                  {"cruise_speed", 1.},
                                  {"corridor_range", .5},
                                  {"sample_spacing", .025},
                                  {"optimization_iterations", 8},
                                  {"graph_budget_ms", 15000.}}},
          {"passages", QJsonArray{}},
          {"blockers", QJsonArray{}},
          {"test", QJsonObject{{"start", QJsonArray{1., 1.}},
                               {"goal", QJsonArray{8., 5.}}}}};
}
void autoEndpoints(QJsonObject &g, const QJsonObject &project) {
  auto r = g["region"].toObject(), cfg = project["planner"].toObject();
  double offset =
      r["length"].toDouble() / 2 + cfg["robot_radius"].toDouble() +
      cfg["safety_margin"].toDouble() +
      std::max(.15, project["map"].toObject()["resolution"].toDouble() * 2);
  auto c = point(r["center"]), d = axis(r["yaw"].toDouble());
  g["endpoint_a"] = jsonPoint(c - d * offset);
  g["endpoint_b"] = jsonPoint(c + d * offset);
  g["auto_endpoints"] = true;
}
QJsonObject newPassage(const QJsonObject &r, const QJsonObject &p) {
  QString id;
  int i = 0;
  bool found;
  do {
    id = QString("passage_%1").arg(++i, 2, 10, QChar('0'));
    found = false;
    for (const auto &v : p["passages"].toArray())
      if (v.toObject()["id"].toString() == id)
        found = true;
  } while (found);
  QJsonObject g{{"id", id},
                {"region", r},
                {"direction", "both"},
                {"enabled", true},
                {"clear_static", true},
                {"clear_unknown", false},
                {"prepare_distance", .8},
                {"release_distance", .4},
                {"speed", .5},
                {"action_cost_seconds", 1.},
                {"fold_hold_seconds", .3}, {"restore_hold_seconds", .3},
                {"beam_min_height", .26}, {"beam_max_height", 2.},
                {"mode", "folded"}};
  autoEndpoints(g, p);
  return g;
}
QJsonObject readObject(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly))
    error("无法读取：" + path);
  QJsonParseError e;
  auto d = QJsonDocument::fromJson(file.readAll(), &e);
  if (e.error != QJsonParseError::NoError || !d.isObject())
    error("JSON 错误：" + e.errorString());
  return d.object();
}
void writeObject(const QString &path, const QJsonObject &o) {
  QSaveFile file(path);
  auto bytes = QJsonDocument(o).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() ||
      !file.commit())
    error("无法写入：" + path);
}
Document::Document() {
  image = QImage(240, 160, QImage::Format_Grayscale8);
  image.fill(255);
}
void Document::checkpoint() {
  undo_.append({data, image});
  if (undo_.size() > 30)
    undo_.removeFirst();
  redo_.clear();
}
void Document::touch() {
  dirty = true;
  ++revision;
}
bool Document::undo() {
  if (undo_.isEmpty())
    return false;
  redo_.append({data, image});
  auto s = undo_.takeLast();
  data = s.data;
  image = s.image;
  touch();
  return true;
}
bool Document::redo() {
  if (redo_.isEmpty())
    return false;
  undo_.append({data, image});
  auto s = redo_.takeLast();
  data = s.data;
  image = s.image;
  touch();
  return true;
}
void Document::open(const QString &path) {
  auto incoming = readObject(path), next = defaultProject();
  if (incoming["schema_version"].toInt() != 1)
    error("不支持的工程版本");
  for (auto i = incoming.begin(); i != incoming.end(); ++i)
    next[i.key()] = i.value();
  for (const auto &key : {"map", "planner", "test"})
    next[key] =
        merge(defaultProject()[key].toObject(), incoming[key].toObject());
  auto m = next["map"].toObject();
  validateMap(m);
  QString imagePath = m["image"].toString();
  if (QFileInfo(imagePath).isRelative())
    imagePath = QFileInfo(path).dir().filePath(imagePath);
  QImage loaded(imagePath);
  if (loaded.isNull())
    error("无法读取地图：" + imagePath);
  loaded = loaded.convertToFormat(QImage::Format_Grayscale8);
  if (m["negate"].toBool()) {
    loaded.invertPixels();
    m["negate"] = false;
    next["map"] = m;
  }
  data = next;
  image = loaded;
  filename = QFileInfo(path).absoluteFilePath();
  dirty = false;
  ++revision;
  undo_.clear();
  redo_.clear();
}
void Document::newMap(int w, int h, double resolution) {
  if (w < 8 || h < 8 || w > 20000 || h > 20000 || !std::isfinite(resolution) ||
      resolution <= 0)
    error("地图尺寸或分辨率无效");
  QImage next(w, h, QImage::Format_Grayscale8);
  if (next.isNull())
    error("地图内存分配失败");
  next.fill(255);
  data = defaultProject();
  auto m = data["map"].toObject();
  m["resolution"] = resolution;
  data["map"] = m;
  image = next;
  data["test"] =
      QJsonObject{{"start", jsonPoint(pixelToWorld({w * .15, h * .5}))},
                  {"goal", jsonPoint(pixelToWorld({w * .85, h * .5}))}};
  filename.clear();
  undo_.clear();
  redo_.clear();
  touch();
}
void Document::importMap(const QString &path) {
  auto next = defaultProject(), m = next["map"].toObject();
  QString imagePath = path;
  bool negate = false;
  if (QFileInfo(path).suffix().toLower() == "yaml" ||
      QFileInfo(path).suffix().toLower() == "yml") {
    auto y = YAML::LoadFile(path.toStdString());
    imagePath = QString::fromStdString(y["image"].as<std::string>());
    if (QFileInfo(imagePath).isRelative())
      imagePath = QFileInfo(path).dir().filePath(imagePath);
    m["resolution"] = y["resolution"].as<double>();
    if (y["origin"]) {
      auto o = y["origin"];
      if (!o.IsSequence() || o.size() < 3)
        error("YAML origin 需要 x/y/yaw");
      m["origin"] = QJsonArray{o[0].as<double>(), o[1].as<double>()};
      m["yaw"] = o[2].as<double>();
    }
    m["occupied_max"] = int(std::floor(
        (1 - (y["occupied_thresh"] ? y["occupied_thresh"].as<double>() : .65)) *
        255));
    m["free_min"] = int(std::ceil(
        (1 - (y["free_thresh"] ? y["free_thresh"].as<double>() : .196)) * 255));
    negate = y["negate"] && y["negate"].as<int>() != 0;
  }
  validateMap(m);
  QImage loaded(imagePath);
  if (loaded.isNull())
    error("无法读取地图：" + imagePath);
  loaded = loaded.convertToFormat(QImage::Format_Grayscale8);
  if (negate)
    loaded.invertPixels();
  m["image"] = QFileInfo(imagePath).absoluteFilePath();
  next["map"] = m;
  data = next;
  image = loaded;
  data["test"] = QJsonObject{
      {"start",
       jsonPoint(pixelToWorld({image.width() * .15, image.height() * .5}))},
      {"goal",
       jsonPoint(pixelToWorld({image.width() * .85, image.height() * .5}))}};
  filename.clear();
  undo_.clear();
  redo_.clear();
  touch();
}
void Document::exportPassages(const QString &path) const {
  // 运行时不依赖图片。只输出通道，测试障碍、测试起终点和编辑器状态不进入导航。
  const auto gates = data["passages"].toArray();
  if (gates.size() > 16) error("导航最多支持 16 个通道");
  QSet<QString> ids;
  for (const auto &v : gates) {
    const auto g = v.toObject(), r = g["region"].toObject();
    const auto id = g["id"].toString();
    if (id.isEmpty() || ids.contains(id)) error("通道编号为空或重复：" + id);
    ids.insert(id);
    const auto c = point(r["center"]), d = axis(r["yaw"].toDouble());
    double along[2]; int i = 0;
    for (const auto &key : {"endpoint_a", "endpoint_b"}) {
      const auto p = point(g[key]) - c;
      along[i++] = dot(p, d);
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) ||
          std::abs(dot(p, QPointF(-d.y(), d.x()))) > .001 ||
          std::abs(dot(p, d)) <= r["length"].toDouble()/2)
        error(id + "：A/B 必须位于长轴中线上、矩形外侧。请点击“按区域自动对齐 A / B”。");
    }
    if (along[0] * along[1] >= 0) error(id + "：A/B 必须分居区域两端");
  }
  const auto map = data["map"].toObject();
  const QJsonObject reference{{"resolution",map["resolution"]},
                              {"origin",map["origin"]},{"yaw",map["yaw"]}};
  QJsonObject output{{"schema_version", 1}, {"frame_id", data["map"].toObject()["frame_id"]},
                     {"map_reference", reference},
                     {"passages", data["passages"]}};
  writeObject(path, output);
}
void Document::importPassages(const QString &path) {
  auto j = readObject(path);
  if (j["schema_version"].toInt() != 1 || !j["passages"].isArray()) error("通道文件格式错误");
  const auto frame = j["frame_id"].toString(j["map"].toObject()["frame_id"].toString("map"));
  if (frame != data["map"].toObject()["frame_id"].toString("map")) error("通道与地图 frame_id 不一致");
  const auto reference = j.contains("map_reference") ? j["map_reference"].toObject() : j["map"].toObject();
  if (!reference.isEmpty()) {
    const auto m = data["map"].toObject();
    if (norm(point(reference["origin"])-point(m["origin"])) > 1e-6 ||
        std::abs(reference["resolution"].toDouble()-m["resolution"].toDouble()) > 1e-6 ||
        std::abs(std::remainder(reference["yaw"].toDouble()-m["yaw"].toDouble(),2*Pi)) > 1e-6)
      error("通道参考地图与当前地图的原点、分辨率或角度不同，请先导入对应的地图 YAML。");
  }
  checkpoint(); data["passages"] = j["passages"]; touch();
}
void Document::save(const QString &path) {
  auto info = QFileInfo(path);
  QDir().mkpath(info.absolutePath());
  QString sidecar = info.completeBaseName() + "_map.png";
  auto next = data, m = next["map"].toObject();
  m["image"] = sidecar;
  m["negate"] = false;
  next["map"] = m;
  saveImage(image, info.dir().filePath(sidecar));
  writeObject(info.absoluteFilePath(), next);
  data = next;
  filename = info.absoluteFilePath();
  dirty = false;
}
QString Document::writeSnapshot(const QString &directory) const {
  if (!QDir().mkpath(directory))
    error("无法创建快照目录");
  auto next = data, m = next["map"].toObject();
  m["image"] = "map.png";
  m["negate"] = false;
  next["map"] = m;
  saveImage(image, QDir(directory).filePath("map.png"));
  auto path = QDir(directory).filePath("project.json");
  writeObject(path, next);
  return path;
}
QPointF Document::pixelToWorld(QPointF p) const {
  auto m = data["map"].toObject();
  double yaw = m["yaw"].toDouble(), res = m["resolution"].toDouble();
  return point(m["origin"]) + axis(yaw) * (p.x() * res) +
         axis(yaw + Pi / 2) * ((image.height() - p.y()) * res);
}
QPointF Document::worldToPixel(QPointF p) const {
  auto m = data["map"].toObject();
  auto v = p - point(m["origin"]);
  double yaw = m["yaw"].toDouble(), res = m["resolution"].toDouble();
  return {dot(v, axis(yaw)) / res,
          image.height() - dot(v, axis(yaw + Pi / 2)) / res};
}
ReplayState replayState(const QJsonObject &result, double s) {
  ReplayState state;
  for (const auto &v : result["traversals"].toArray()) {
    auto t = v.toObject();
    if (s + 1e-8 < t["prepare_s"].toDouble() ||
        s >= t["release_s"].toDouble() - 1e-8)
      continue;
    bool locked = s + 1e-8 >= t["enter_s"].toDouble();
    if (state.folded && !locked)
      continue;
    state.folded = true;
    state.locked = locked;
    state.passageId = t["passage_id"].toString();
    state.mode = t["mode"].toString();
    state.phase = !locked                             ? "PREPARE"
                  : s < t["exit_s"].toDouble() - 1e-8 ? "PASSING"
                                                      : "RELEASING";
  }
  return state;
}
QPointF positionAt(const QJsonObject &r, double s) {
  auto path = r["path"].toArray(), arcs = r["arc_lengths"].toArray();
  if (path.isEmpty() || arcs.size() != path.size())
    return {};
  int lo = 0, hi = arcs.size();
  while (lo < hi) {
    int mid = (lo + hi) / 2;
    if (arcs[mid].toDouble() < s)
      lo = mid + 1;
    else
      hi = mid;
  }
  if (lo == 0)
    return point(path[0]);
  if (lo >= path.size())
    return point(path.last());
  double a = arcs[lo - 1].toDouble(), b = arcs[lo].toDouble();
  return point(path[lo - 1]) + (point(path[lo]) - point(path[lo - 1])) *
                                   ((s - a) / std::max(1e-12, b - a));
}
} // namespace passage_editor
