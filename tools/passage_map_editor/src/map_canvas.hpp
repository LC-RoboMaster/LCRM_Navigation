#pragma once
#include "document.hpp"
#include <QMap>
#include <QWidget>

namespace passage_editor {
class MapCanvas : public QWidget {
  Q_OBJECT
public:
  explicit MapCanvas(Document *document, QWidget *parent = nullptr);
  Document *doc;
  QString tool = "select";
  int selectedGate = -1, selectedBlocker = -1;
  double brushRadius = .12, scale = 3., progress = -1.;
  QPointF pan{40, 40};
  QJsonObject result;
  QImage background;
  QMap<QString, bool> layers{{"graph", false},  {"raw", false},
                             {"simple", false}, {"corridors", false},
                             {"final", true},   {"events", true}};
  QPointF screen(QPointF world) const;
  QPointF world(QPointF screen) const;
  QPair<QJsonObject, QJsonObject> selected() const;
  void setSelection(const QString &kind, int index);
  void updateSelected(const QJsonObject &gate, const QJsonObject &rect);
  QPointF rotationHandle(const QJsonObject &rect) const;
  QJsonObject previewRect(QPointF cursor) const;
public slots:
  void fitMap();
  void setTool(const QString &name);
signals:
  void changed();
  void selectionChanged(QString kind, int index);
  void regionCreated(QString kind, QJsonObject region);
  void positionChanged(QPointF world);
  void hintChanged(QString text);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override;
  void wheelEvent(QWheelEvent *) override;
  void keyPressEvent(QKeyEvent *) override;

private:
  QVector<QPointF> points_;
  QPointF hover_, lastPos_, press_, lastStroke_;
  bool haveHover_ = false, panning_ = false, stroke_ = false;
  QString drag_;
  QJsonObject oldGate_, oldRect_;
  void paintBrush(QPointF world);
};
} // namespace passage_editor
