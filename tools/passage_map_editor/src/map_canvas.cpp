#include "map_canvas.hpp"
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace passage_editor {
namespace {
QPointF mousePos(QMouseEvent *e) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  return e->position();
#else
  return e->localPos();
#endif
}
QPointF wheelPos(QWheelEvent *e) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
  return e->position();
#else
  return e->posF();
#endif
}
QVector<QPointF> pathPoints(const QJsonArray &a) {
  QVector<QPointF> p;
  for (const auto &v : a)
    p.append(point(v));
  return p;
}
} // namespace
MapCanvas::MapCanvas(Document *d, QWidget *parent) : QWidget(parent), doc(d) {
  setMinimumSize(450, 400);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setObjectName("mapCanvas");
}
QPointF MapCanvas::screen(QPointF p) const {
  return pan + doc->worldToPixel(p) * scale;
}
QPointF MapCanvas::world(QPointF p) const {
  return doc->pixelToWorld((p - pan) / scale);
}
void MapCanvas::fitMap() {
  scale = std::max(.01, std::min((width() - 64.) / doc->image.width(),
                                 (height() - 64.) / doc->image.height()));
  pan = {(width() - doc->image.width() * scale) / 2,
         (height() - doc->image.height() * scale) / 2};
  update();
}
void MapCanvas::setTool(const QString &name) {
  tool = name;
  points_.clear();
  drag_.clear();
  setCursor(name == "select" ? Qt::ArrowCursor : Qt::CrossCursor);
  static const QMap<QString, QString> hints{
      {"region", "旋转区域：依次点击长轴两端，再点击宽度。Esc 取消。"},
      {"blocker", "真实障碍：点击长轴两端和宽度；不会被通道清除。"},
      {"select", "拖中心移动、角点缩放、蓝色圆点旋转；A/B 可独立拖动。"},
      {"pick_a", "点击设置 A；接口点应位于区域外，连线穿过区域。"},
      {"pick_b", "点击设置 B；反向自动交换出入口。"},
      {"start", "点击普通可通行空间设置起点。"},
      {"goal", "点击普通可通行空间设置终点。"},
      {"wall", "按住左键画静态墙。"},
      {"erase", "按住左键擦除静态墙。"}};
  emit hintChanged(hints.value(name, "中键拖动平移，滚轮以鼠标为中心缩放。"));
  update();
}
QPair<QJsonObject, QJsonObject> MapCanvas::selected() const {
  auto gates = doc->data["passages"].toArray(),
       blocks = doc->data["blockers"].toArray();
  if (selectedGate >= 0 && selectedGate < gates.size()) {
    auto g = gates[selectedGate].toObject();
    return {g, g["region"].toObject()};
  }
  if (selectedBlocker >= 0 && selectedBlocker < blocks.size())
    return {{}, blocks[selectedBlocker].toObject()};
  return {};
}
void MapCanvas::setSelection(const QString &kind, int i) {
  selectedGate = kind == "passage" ? i : -1;
  selectedBlocker = kind == "blocker" ? i : -1;
  emit selectionChanged(kind, i);
  update();
}
void MapCanvas::updateSelected(const QJsonObject &gate,
                               const QJsonObject &rect) {
  if (selectedGate >= 0) {
    auto a = doc->data["passages"].toArray();
    if (selectedGate < a.size()) {
      auto g = gate;
      g["region"] = rect;
      a[selectedGate] = g;
      doc->data["passages"] = a;
    }
  } else if (selectedBlocker >= 0) {
    auto a = doc->data["blockers"].toArray();
    if (selectedBlocker < a.size()) {
      a[selectedBlocker] = rect;
      doc->data["blockers"] = a;
    }
  }
}
QPointF MapCanvas::rotationHandle(const QJsonObject &r) const {
  return point(r["center"]) +
         axis(r["yaw"].toDouble() + Pi / 2) * (r["width"].toDouble() / 2 + .4);
}
QJsonObject MapCanvas::previewRect(QPointF cursor) const {
  if (points_.size() < 2)
    return {};
  auto a = points_[0], b = points_[1], v = b - a, c = (a + b) / 2;
  double yaw = std::atan2(v.y(), v.x());
  return {{"center", jsonPoint(c)},
          {"length", std::max(.05, norm(v))},
          {"width",
           std::max(.05, 2 * std::abs(dot(cursor - c, axis(yaw + Pi / 2))))},
          {"yaw", yaw}};
}
void MapCanvas::mousePressEvent(QMouseEvent *e) {
  setFocus();
  auto pos = mousePos(e), w = world(pos);
  if (e->button() == Qt::MiddleButton ||
      (e->button() == Qt::LeftButton && tool == "pan")) {
    panning_ = true;
    lastPos_ = pos;
    return;
  }
  if (e->button() == Qt::RightButton) {
    setTool("select");
    return;
  }
  if (e->button() != Qt::LeftButton)
    return;
  if (tool == "region" || tool == "blocker") {
    if (points_.size() < 2) {
      points_.append(w);
      emit hintChanged(points_.size() == 1 ? "再点击长轴另一端。"
                                           : "点击确定宽度。 ");
    } else {
      emit regionCreated(tool, previewRect(w));
      points_.clear();
    }
    update();
    return;
  }
  if (tool == "start" || tool == "goal") {
    doc->checkpoint();
    auto test = doc->data["test"].toObject();
    test[tool] = jsonPoint(w);
    doc->data["test"] = test;
    doc->touch();
    emit changed();
    update();
    return;
  }
  auto pair = selected();
  auto gate = pair.first, r = pair.second;
  if (tool == "pick_a" || tool == "pick_b") {
    if (!gate.isEmpty()) {
      doc->checkpoint();
      gate[tool == "pick_a" ? "endpoint_a" : "endpoint_b"] = jsonPoint(w);
      gate["auto_endpoints"] = false;
      updateSelected(gate, r);
      doc->touch();
      emit changed();
    }
    setTool("select");
    return;
  }
  if (tool == "wall" || tool == "erase") {
    doc->checkpoint();
    stroke_ = true;
    lastStroke_ = w;
    paintBrush(w);
    return;
  }
  QString handle;
  if (!r.isEmpty()) {
    QVector<QPair<QString, QPointF>> handles;
    if (!gate.isEmpty())
      handles = {{"a", point(gate["endpoint_a"])},
                 {"b", point(gate["endpoint_b"])}};
    handles.append({"move", point(r["center"])});
    handles.append({"rotate", rotationHandle(r)});
    for (auto c : corners(r))
      handles.append({"corner", c});
    for (const auto &h : handles)
      if ((screen(h.second) - pos).manhattanLength() < 16) {
        handle = h.first;
        break;
      }
  }
  if (handle.isEmpty()) {
    auto a = doc->data["passages"].toArray(),
         b = doc->data["blockers"].toArray();
    for (int i = a.size() - 1; i >= 0; --i)
      if (contains(a[i].toObject()["region"].toObject(), w)) {
        setSelection("passage", i);
        handle = "move";
        break;
      }
    if (handle.isEmpty())
      for (int i = b.size() - 1; i >= 0; --i)
        if (contains(b[i].toObject(), w)) {
          setSelection("blocker", i);
          handle = "move";
          break;
        }
    if (handle.isEmpty())
      setSelection("none", -1);
  }
  if (!handle.isEmpty()) {
    auto s = selected();
    doc->checkpoint();
    drag_ = handle;
    press_ = w;
    oldGate_ = s.first;
    oldRect_ = s.second;
  }
}
void MapCanvas::paintBrush(QPointF w) {
  auto p = doc->worldToPixel(w);
  double r = brushRadius / doc->data["map"].toObject()["resolution"].toDouble();
  int x0 = std::max(0, int(std::floor(p.x() - r))),
      x1 = std::min(doc->image.width(), int(std::ceil(p.x() + r)));
  int y0 = std::max(0, int(std::floor(p.y() - r))),
      y1 = std::min(doc->image.height(), int(std::ceil(p.y() + r)));
  for (int y = y0; y < y1; ++y) {
    auto row = doc->image.scanLine(y);
    for (int x = x0; x < x1; ++x)
      if (std::pow(x + .5 - p.x(), 2) + std::pow(y + .5 - p.y(), 2) <= r * r)
        row[x] = tool == "wall" ? 0 : 255;
  }
  update();
}
void MapCanvas::mouseMoveEvent(QMouseEvent *e) {
  auto pos = mousePos(e), w = world(pos);
  hover_ = w;
  haveHover_ = true;
  emit positionChanged(w);
  if (panning_) {
    pan += pos - lastPos_;
    lastPos_ = pos;
    update();
    return;
  }
  if (stroke_) {
    auto delta = w - lastStroke_;
    int n = std::max(
        1, int(std::ceil(norm(delta) / std::max(.005, brushRadius * .5))));
    for (int i = 1; i <= n; ++i)
      paintBrush(lastStroke_ + delta * (double(i) / n));
    lastStroke_ = w;
    return;
  }
  if (!drag_.isEmpty()) {
    auto s = selected();
    auto g = s.first, r = s.second;
    if (r.isEmpty())
      return;
    if ((drag_ == "a" || drag_ == "b") && !g.isEmpty()) {
      g[drag_ == "a" ? "endpoint_a" : "endpoint_b"] = jsonPoint(w);
      g["auto_endpoints"] = false;
    } else {
      if (drag_ == "move")
        r["center"] = jsonPoint(point(oldRect_["center"]) + w - press_);
      else if (drag_ == "rotate") {
        auto v = w - point(r["center"]);
        r["yaw"] = std::atan2(v.y(), v.x()) - Pi / 2;
      } else if (drag_ == "corner") {
        auto v = w - point(r["center"]);
        double yaw = r["yaw"].toDouble();
        r["length"] = std::max(.1, 2 * std::abs(dot(v, axis(yaw))));
        r["width"] = std::max(.1, 2 * std::abs(dot(v, axis(yaw + Pi / 2))));
      }
      if (!g.isEmpty()) {
        g["region"] = r;
        if (g["auto_endpoints"].toBool(true))
          autoEndpoints(g, doc->data);
        else if (drag_ == "move" || drag_ == "rotate") {
          double delta = r["yaw"].toDouble() - oldRect_["yaw"].toDouble();
          for (const auto &key : {"endpoint_a", "endpoint_b"}) {
            auto v = point(oldGate_[key]) - point(oldRect_["center"]);
            g[key] = jsonPoint(point(r["center"]) + axis(delta) * v.x() +
                               axis(delta + Pi / 2) * v.y());
          }
        }
      }
    }
    updateSelected(g, r);
  }
  update();
}
void MapCanvas::mouseReleaseEvent(QMouseEvent *e) {
  if (e->button() == Qt::MiddleButton || tool == "pan")
    panning_ = false;
  if (e->button() == Qt::LeftButton && (stroke_ || !drag_.isEmpty())) {
    stroke_ = false;
    drag_.clear();
    doc->touch();
    emit changed();
    update();
  }
}
void MapCanvas::wheelEvent(QWheelEvent *e) {
  auto p = wheelPos(e), before = (p - pan) / scale;
  scale =
      std::clamp(scale * (e->angleDelta().y() > 0 ? 1.2 : 1 / 1.2), .05, 80.);
  pan = p - before * scale;
  update();
}
void MapCanvas::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Escape)
    setTool("select");
  else
    QWidget::keyPressEvent(e);
}
void MapCanvas::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor("#101923"));
  QRectF target(
      pan, QSizeF(doc->image.width() * scale, doc->image.height() * scale));
  p.drawImage(target, background.isNull() ? doc->image : background);
  p.setPen(QColor("#405267"));
  p.drawRect(target);
  auto line = [&](const QVector<QPointF> &points, const QColor &color,
                  double width = 2., Qt::PenStyle style = Qt::SolidLine) {
    if (points.size() < 2)
      return;
    p.setPen(QPen(color, width, style));
    p.setBrush(Qt::NoBrush);
    QPainterPath path(screen(points[0]));
    for (int i = 1; i < points.size(); ++i)
      path.lineTo(screen(points[i]));
    p.drawPath(path);
  };
  auto marker = [&](QPointF w, const QColor &color,
                    const QString &label = QString(), double radius = 5.,
                    QPointF offset = QPointF(8, -17), bool lead = false) {
    auto q = screen(w);
    p.setPen(QPen(QColor("#13212d"), 1));
    p.setBrush(color);
    p.drawEllipse(q, radius, radius);
    if (label.isEmpty())
      return;
    QRectF box(
        q + offset,
        QSizeF(std::max(36, p.fontMetrics().horizontalAdvance(label) + 10),
               23));
    p.setPen(QColor("#14232e"));
    p.setBrush(QColor(255, 255, 255, 225));
    if (lead)
      p.drawLine(q, box.center());
    p.drawRoundedRect(box, 3, 3);
    p.drawText(box.adjusted(5, 0, 0, 0), Qt::AlignVCenter, label);
  };
  if (!result.isEmpty() && layers["graph"]) {
    auto graph = result["graph"].toObject();
    auto nodes = graph["nodes"].toArray();
    for (const auto &v : graph["edges"].toArray()) {
      auto e = v.toObject();
      int a = e["from"].toInt(), b = e["to"].toInt();
      if (a < 0 || b < 0 || a >= nodes.size() || b >= nodes.size())
        continue;
      line({point(nodes[a].toObject()["position"]),
            point(nodes[b].toObject()["position"])},
           QColor(e["selected"].toBool() ? "#0d9488" : "#6a8499"), 1,
           e["kind"].toString() == "ordinary" ? Qt::DotLine : Qt::DashLine);
    }
  }
  auto gates = doc->data["passages"].toArray(),
       blocks = doc->data["blockers"].toArray();
  for (int i = 0; i < gates.size(); ++i) {
    auto g = gates[i].toObject(), r = g["region"].toObject();
    QPolygonF polygon;
    for (auto c : corners(r))
      polygon << screen(c);
    QColor color(g["enabled"].toBool(true) ? "#d88922" : "#8593a0"),
        fill = color;
    fill.setAlpha(55);
    p.setPen(QPen(color, i == selectedGate ? 3 : 1.5));
    p.setBrush(fill);
    p.drawPolygon(polygon);
    auto a = point(g["endpoint_a"]), b = point(g["endpoint_b"]),
         d = (b - a) / std::max(1e-12, norm(b - a));
    line({a, b}, QColor("#a85d10"), 1.5, Qt::DashLine);
    if (g["direction"].toString() != "b_to_a")
      line({b, b + d * g["release_distance"].toDouble()}, QColor("#c46718"), 2,
           Qt::DotLine);
    if (g["direction"].toString() != "a_to_b")
      line({a, a - d * g["release_distance"].toDouble()}, QColor("#c46718"), 2,
           Qt::DotLine);
    marker(a, QColor("#f0b34f"), "A");
    marker(b, QColor("#f0b34f"), "B");
    p.setPen(QColor("#70400a"));
    p.drawText(screen(point(r["center"])) + QPointF(0, -13),
               g["id"].toString());
  }
  for (int i = 0; i < blocks.size(); ++i) {
    QPolygonF poly;
    for (auto c : corners(blocks[i].toObject()))
      poly << screen(c);
    p.setPen(QPen(QColor("#ef4444"), i == selectedBlocker ? 3 : 1.5));
    p.setBrush(QColor(239, 68, 68, 180));
    p.drawPolygon(poly);
  }
  if (!result.isEmpty()) {
    if (layers["raw"])
      line(pathPoints(result["raw_path"].toArray()), QColor("#ef8c32"), 1);
    if (layers["simple"])
      line(pathPoints(result["simplified_path"].toArray()), QColor("#60a846"),
           1.5);
    if (layers["corridors"])
      for (const auto &v : result["corridors"].toArray()) {
        auto c = pathPoints(v.toArray());
        if (!c.isEmpty())
          c.append(c[0]);
        line(c, QColor("#61a1c3"), 1);
      }
    if (layers["final"])
      line(pathPoints(result["path"].toArray()), QColor("#087f8c"), 3);
    if (layers["events"]) {
      const QMap<QString, QColor> colors{{"DEFORM", QColor("#b96ad1")},
                                         {"ENTER", QColor("#e5a126")},
                                         {"EXIT", QColor("#e5a126")},
                                         {"RELEASE", QColor("#4b80d4")}};
      const QMap<QString, QPointF> offsets{{"DEFORM", {-52, -49}},
                                           {"ENTER", {-25, -77}},
                                           {"EXIT", {0, 24}},
                                           {"RELEASE", {12, 50}}};
      for (const auto &v : result["events"].toArray()) {
        auto e = v.toObject();
        auto type = e["type"].toString();
        marker(point(e["position"]), colors.value(type, Qt::gray), type, 4,
               offsets.value(type), true);
      }
    }
    if (progress >= 0 && result["success"].toBool())
      marker(
          positionAt(result, progress), QColor("#27bed0"), "robot",
          std::max(6.,
                   doc->data["planner"].toObject()["robot_radius"].toDouble() /
                       doc->data["map"].toObject()["resolution"].toDouble() *
                       scale));
  }
  auto test = doc->data["test"].toObject();
  marker(point(test["start"]), QColor("#20ad74"), "START", 6);
  marker(point(test["goal"]), QColor("#eb608b"), "GOAL", 6);
  auto r = selected().second;
  if (!r.isEmpty()) {
    for (auto c : corners(r)) {
      auto q = screen(c);
      p.setPen(QPen(QColor("#17618b"), 1));
      p.setBrush(Qt::white);
      p.drawRect(QRectF(q - QPointF(4, 4), QSizeF(8, 8)));
    }
    line({point(r["center"]), rotationHandle(r)}, QColor("#17618b"), 1);
    marker(rotationHandle(r), QColor("#63b9f1"), {}, 6);
    marker(point(r["center"]), Qt::white, {}, 4);
  }
  if (!points_.isEmpty() && haveHover_) {
    if (points_.size() == 1)
      line({points_[0], hover_}, QColor("#4bbcf0"), 2, Qt::DashLine);
    else {
      QPolygonF poly;
      for (auto c : corners(previewRect(hover_)))
        poly << screen(c);
      p.setBrush(QColor(70, 185, 230, 50));
      p.setPen(QPen(QColor("#4bbcf0"), 2, Qt::DashLine));
      p.drawPolygon(poly);
    }
  }
  double metres = 1.,
         res = doc->data["map"].toObject()["resolution"].toDouble();
  while (metres / res * scale > width() * .25)
    metres /= 2;
  double length = metres / res * scale;
  QPointF base(22, height() - 24);
  p.setPen(QPen(QColor("#bcd0de"), 2));
  p.drawLine(base, base + QPointF(length, 0));
  p.drawLine(base + QPointF(0, -4), base + QPointF(0, 4));
  p.drawLine(base + QPointF(length, -4), base + QPointF(length, 4));
  p.drawText(base + QPointF(0, -9), QString::number(metres, 'g', 3) + " m");
}
} // namespace passage_editor
