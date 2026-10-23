#pragma once
#include "document.hpp"
#include <QObject>
#include <QProcess>
#include <QTemporaryDir>
#include <QTimer>
#include <memory>

namespace passage_editor {
// IPC adapter only. It knows the file protocol, never the planner
// implementation.
class PlannerClient : public QObject {
  Q_OBJECT
public:
  explicit PlannerClient(QObject *parent = nullptr);
  ~PlannerClient() override;
  QString executable;
  int timeoutMs = 180000;
  bool busy() const { return running_; }
  QString runDirectory() const;
  bool start(const Document &document);
  void cancel();
signals:
  void completed(QJsonObject result, quint64 sourceRevision);
  void failed(QString message);
  void log(QString text);
  void runningChanged(bool running);

private:
  QProcess process_;
  QTimer timeout_;
  std::unique_ptr<QTemporaryDir> work_;
  quint64 revision_ = 0;
  bool running_ = false, cancelled_ = false, timedOut_ = false;
  void finish(int code, QProcess::ExitStatus status);
  void finishError(const QString &message);
};
} // namespace passage_editor
