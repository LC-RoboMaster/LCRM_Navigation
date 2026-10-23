#include "main_window.hpp"
#include <QApplication>
#include <QCommandLineParser>
#include <QFile>
#include <QFontDatabase>
#include <QTimer>
#include <iostream>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("HBUT");
  QCoreApplication::setApplicationName("PassageStudio");
  QCoreApplication::setApplicationVersion("2.0.0");
  QCommandLineParser parser;
  parser.setApplicationDescription("Independent C++ Qt passage map editor");
  parser.addHelpOption();
  parser.addVersionOption();
  parser.addOption({"project", "Open project JSON", "path"});
  parser.addOption({"cli", "External planner executable", "path"});
  parser.addOption({"autoplan", "Plan after opening"});
  parser.addOption(
      {"render", "Save an actual editor screenshot after planning, then exit",
       "png"});
  parser.process(app);
  QString font = qEnvironmentVariable("TDT_TEST_FONT");
  if (!font.isEmpty())
    QFontDatabase::addApplicationFont(font);
  app.setStyleSheet(QString::fromUtf8(R"CSS(
QWidget {background:#14202c;color:#dce6ef;font-family:'Noto Sans CJK SC','Microsoft YaHei','DejaVu Sans';font-size:12px;}
QMenuBar,QMenu,QToolBar {background:#1a2938;} QMenu::item:selected {background:#235575;}
QGroupBox {border:1px solid #324658;border-radius:7px;margin-top:12px;padding:10px 7px 7px;}
QGroupBox::title {subcontrol-origin:margin;left:10px;color:#9fb5c8;}
QPushButton {background:#22364a;border:1px solid #3b5268;border-radius:5px;padding:6px 9px;}
QPushButton:hover {background:#2d4862;border-color:#5c90ad;} QPushButton:checked {background:#145575;border-color:#55c2e7;}
QPushButton:disabled {color:#627586;} QPushButton#primary {background:#087f8c;border-color:#22a5b2;font-weight:600;}
QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox {background:#0f1b27;border:1px solid #354b61;border-radius:4px;padding:4px;min-height:20px;}
QListWidget,QPlainTextEdit {background:#101b27;border:1px solid #30465a;border-radius:5px;}
QListWidget::item {padding:7px 5px;} QListWidget::item:selected {background:#1c526a;color:white;}
QTabWidget::pane {border:1px solid #30465a;} QTabBar::tab {background:#1b2e40;padding:8px 10px;}
QTabBar::tab:selected {background:#28506b;} QScrollArea {border:0;}
QSlider::groove:horizontal {height:5px;background:#3b5265;border-radius:2px;}
QSlider::handle:horizontal {width:12px;background:#4fc0cf;margin:-5px 0;border-radius:5px;}
QProgressBar {border:0;background:#22364a;} QProgressBar::chunk {background:#2bb4c3;}
QStatusBar {background:#101a24;color:#9cb3c6;}
)CSS"));
  passage_editor::MainWindow window(parser.value("cli"));
  window.show();
  if (parser.isSet("project"))
    window.openProject(parser.value("project"));
  else if (!passage_editor::MainWindow::findExamples().isEmpty())
    window.loadExample(1);
  if (parser.isSet("render")) {
    QObject::connect(
        &window, &passage_editor::MainWindow::planningFinished, &app,
        [&](const QJsonObject &r) {
          if (!r["success"].toBool()) {
            std::cerr << r["message"].toString().toStdString() << '\n';
            app.exit(3);
            return;
          }
          window.setBackground(1);
          window.canvas->setSelection("passage", 0);
          auto t = r["traversals"].toArray();
          if (!t.isEmpty()) {
            auto v = t[0].toObject();
            window.seek((v["enter_s"].toDouble() + v["exit_s"].toDouble()) / 2);
          }
          QTimer::singleShot(80, &app, [&] {
            if (!window.grab().save(parser.value("render"))) {
              app.exit(4);
              return;
            }
            app.exit(0);
          });
        });
    QObject::connect(&window, &passage_editor::MainWindow::planningFailed, &app,
                     [&](const QString &s) {
                       std::cerr << s.toStdString() << '\n';
                       app.exit(3);
                     });
    QTimer::singleShot(190000, &app, [&] { app.exit(5); });
  }
  if (parser.isSet("autoplan") || parser.isSet("render"))
    QTimer::singleShot(80, &window, &passage_editor::MainWindow::plan);
  return app.exec();
}
