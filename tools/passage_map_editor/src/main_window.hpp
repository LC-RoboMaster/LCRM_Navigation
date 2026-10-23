#pragma once
#include "map_canvas.hpp"
#include "planner_client.hpp"
#include <QMainWindow>
#include <QMap>
#include <QTimer>
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QProgressBar;
class QSlider;

namespace passage_editor {
class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QString executable = {}, QWidget *parent = nullptr);
  Document doc;
  MapCanvas *canvas = nullptr;
  PlannerClient client;
  QJsonObject result;
  double currentProgress = 0.;
  void openProject(const QString &path);
  void importMap(const QString &path);
  bool exportTo(const QString &directory);
  void loadExample(int index);
  void plan();
  void seek(double s);
  void addRegion(QString kind, QJsonObject rect);
  void deleteSelection();
  void alignEndpoints();
  void changed();
  void syncAll();
  void applyProperties();
  void applySettings();
  void setBackground(int index);
  static QString findExamples();
signals:
  void planningFinished(QJsonObject result);
  void planningFailed(QString message);

protected:
  void closeEvent(QCloseEvent *) override;

private:
  bool updating_ = false, playing_ = false;
  quint64 resultRevision_ = 0;
  QTimer timer_;
  QMap<QString, QDoubleSpinBox *> fields_, settings_, mapFields_;
  QLineEdit *id_ = nullptr, *mode_ = nullptr, *cliField_ = nullptr;
  QComboBox *direction_ = nullptr, *background_ = nullptr;
  QCheckBox *enabled_ = nullptr, *clearStatic_ = nullptr, *optimize_ = nullptr,
            *fallback_ = nullptr;
  QListWidget *items_ = nullptr;
  QLabel *summary_ = nullptr, *hint_ = nullptr, *coordinate_ = nullptr,
         *propertyTitle_ = nullptr, *progressLabel_ = nullptr,
         *stateLabel_ = nullptr;
  QPlainTextEdit *log_ = nullptr;
  QPushButton *planButton_ = nullptr, *cancelButton_ = nullptr,
              *playButton_ = nullptr;
  QSlider *slider_ = nullptr;
  QProgressBar *busy_ = nullptr;
  void buildUi();
  void buildActions();
  void syncProperties();
  void selectionChanged(QString kind, int index);
  void receiveResult(QJsonObject value, quint64 revision);
  void invalidate();
  void openDialog();
  void importDialog();
  void save(bool saveAs = false);
  void newMapDialog();
  void exportDialog();
  void duplicate();
  void undo();
  void redo();
  void togglePlay();
  void logMessage(const QString &message);
  bool canDiscard();
  void showError(const QString &message);
};
} // namespace passage_editor
