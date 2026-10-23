#include "planner_client.hpp"
#include <QDir>
#include <QFileInfo>
#include <cmath>
#include <stdexcept>

namespace passage_editor {
PlannerClient::PlannerClient(QObject *parent) : QObject(parent) {
  process_.setProcessChannelMode(QProcess::MergedChannels);
  timeout_.setSingleShot(true);
  connect(&process_, &QProcess::readyReadStandardOutput, this, [this] {
    emit log(QString::fromUtf8(process_.readAllStandardOutput()));
  });
  connect(&process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          this, &PlannerClient::finish);
  connect(&process_, &QProcess::errorOccurred, this,
          [this](QProcess::ProcessError e) {
            if (e == QProcess::FailedToStart)
              finishError("无法启动规划器：" + process_.errorString());
          });
  connect(&timeout_, &QTimer::timeout, this, [this] {
    timedOut_ = true;
    process_.kill();
  });
}
PlannerClient::~PlannerClient() {
  if (process_.state() != QProcess::NotRunning) {
    process_.kill();
    process_.waitForFinished(1000);
  }
}
QString PlannerClient::runDirectory() const {
  return work_ ? work_->path() : QString();
}
bool PlannerClient::start(const Document &document) {
  if (running_ || process_.state() != QProcess::NotRunning)
    return false;
  try {
    if (executable.isEmpty() || !QFileInfo(executable).isExecutable())
      throw std::runtime_error("请先选择已编译的 passage_cli 可执行文件");
    work_ = std::make_unique<QTemporaryDir>(QDir::tempPath() +
                                            "/passage_native_XXXXXX");
    if (!work_->isValid())
      throw std::runtime_error("无法创建规划临时目录");
    QString project = document.writeSnapshot(work_->path());
    revision_ = document.revision;
    cancelled_ = timedOut_ = false;
    running_ = true;
    emit runningChanged(true);
    process_.setWorkingDirectory(work_->path());
    process_.start(QFileInfo(executable).absoluteFilePath(),
                   QStringList{project, "--out", work_->filePath("result.json"),
                               "--debug", work_->filePath("debug")});
    timeout_.start(timeoutMs);
    return true;
  } catch (const std::exception &e) {
    emit failed(QString::fromUtf8(e.what()));
    return false;
  }
}
void PlannerClient::cancel() {
  if (running_) {
    cancelled_ = true;
    process_.kill();
  }
}
void PlannerClient::finishError(const QString &message) {
  if (!running_)
    return;
  running_ = false;
  timeout_.stop();
  emit runningChanged(false);
  emit failed(message);
}
void PlannerClient::finish(int code, QProcess::ExitStatus status) {
  if (!running_)
    return;
  if (cancelled_) {
    finishError("已取消本次规划");
    return;
  }
  if (timedOut_) {
    finishError("规划进程超时，已结束");
    return;
  }
  if (status != QProcess::NormalExit) {
    finishError("规划进程异常退出");
    return;
  }
  try {
    if (code != 0 && code != 3)
      throw std::runtime_error(
          ("规划器退出码 " + QString::number(code) + "，请查看日志")
              .toStdString());
    auto r = readObject(work_->filePath("result.json"));
    if (!r["success"].isBool() || !r["status"].isString())
      throw std::runtime_error("规划结果协议无效：缺少 success/status");
    if (r["success"].toBool()) {
      auto path = r["path"].toArray(), arcs = r["arc_lengths"].toArray();
      double previous = -1.;
      if (path.isEmpty() || path.size() != arcs.size())
        throw std::runtime_error("规划结果路径与弧长数量不匹配");
      for (int i = 0; i < path.size(); ++i) {
        auto v = path[i].toArray();
        double s = arcs[i].toDouble(-1);
        if (v.size() != 2 || !std::isfinite(v[0].toDouble()) ||
            !std::isfinite(v[1].toDouble()) || s < previous ||
            !std::isfinite(s))
          throw std::runtime_error("规划结果坐标或弧长无效");
        previous = s;
      }
    }
    running_ = false;
    timeout_.stop();
    emit runningChanged(false);
    emit completed(r, revision_);
  } catch (const std::exception &e) {
    finishError(QString::fromUtf8(e.what()));
  }
}
} // namespace passage_editor
