#pragma once
#include <QImage>
#include <QJsonArray>
#include <QJsonObject>
#include <QPointF>
#include <QString>
#include <QVector>

namespace passage_editor {
constexpr double Pi = 3.14159265358979323846;
QPointF point(const QJsonValue &v);
QJsonArray jsonPoint(const QPointF &p);
QPointF axis(double yaw);
double norm(const QPointF &p);
double dot(const QPointF &a, const QPointF &b);
QVector<QPointF> corners(const QJsonObject &r);
bool contains(const QJsonObject &r, QPointF p);
QJsonObject defaultProject();
void autoEndpoints(QJsonObject &gate, const QJsonObject &project);
QJsonObject newPassage(const QJsonObject &rect, const QJsonObject &project);
QJsonObject readObject(const QString &path);
void writeObject(const QString &path, const QJsonObject &object);

class Document {
public:
  QJsonObject data = defaultProject();
  QImage image;
  QString filename;
  bool dirty = false;
  quint64 revision = 0;
  Document();
  void checkpoint();
  void touch();
  bool undo();
  bool redo();
  void open(const QString &path);
  void importMap(const QString &path);
  void newMap(int width, int height, double resolution);
  void save(const QString &path);
  void exportPassages(const QString &path) const;
  void importPassages(const QString &path);
  QString writeSnapshot(const QString &directory) const;
  QPointF pixelToWorld(QPointF p) const;
  QPointF worldToPixel(QPointF p) const;

private:
  struct Snapshot {
    QJsonObject data;
    QImage image;
  };
  QVector<Snapshot> undo_, redo_;
};

struct ReplayState {
  bool folded = false, locked = false;
  QString phase = "NORMAL", passageId, mode = "normal";
};
ReplayState replayState(const QJsonObject &result, double s);
QPointF positionAt(const QJsonObject &result, double s);
} // namespace passage_editor
