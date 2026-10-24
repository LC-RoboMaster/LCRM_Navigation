#include "main_window.hpp"
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace passage_editor {
namespace {
QDoubleSpinBox *spin(double value, double lo = -10000, double hi = 10000,
                     int decimals = 3) {
  auto w = new QDoubleSpinBox;
  w->setRange(lo, hi);
  w->setDecimals(decimals);
  w->setSingleStep(.05);
  w->setValue(value);
  w->setKeyboardTracking(false);
  return w;
}
QString findCli() {
  QString env = qEnvironmentVariable("TDT_PASSAGE_CLI");
  if (!env.isEmpty())
    return env;
  QString saved = QSettings().value("planner/executable").toString();
  if (QFileInfo(saved).isExecutable())
    return saved;
  QStringList bases{QDir::currentPath(),
                    QCoreApplication::applicationDirPath()};
  for (const auto &base : bases) {
    QDir d(base);
    for (int i = 0; i < 5; ++i) {
      for (const auto &suffix :
           {"passage_cli", "planner/passage_cli", "build/planner/passage_cli",
            "build/passage_cli"}) {
        QString p = d.filePath(suffix);
        if (QFileInfo(p).isExecutable())
          return QFileInfo(p).absoluteFilePath();
      }
      if (!d.cdUp())
        break;
    }
  }
  return {};
}
void copyTree(const QString &from, const QString &to) {
  if (!QDir().mkpath(to))
    throw std::runtime_error("无法创建导出目录");
  QDir source(from);
  QDirIterator it(from, QDir::Files | QDir::NoDotAndDotDot,
                  QDirIterator::Subdirectories);
  while (it.hasNext()) {
    auto path = it.next(), relative = source.relativeFilePath(path),
         dest = QDir(to).filePath(relative);
    QDir().mkpath(QFileInfo(dest).absolutePath());
    if (!QFile::copy(path, dest))
      throw std::runtime_error(("复制失败：" + relative).toStdString());
  }
}
} // namespace
MainWindow::MainWindow(QString executable, QWidget *parent)
    : QMainWindow(parent), client(this) {
  client.executable = executable.isEmpty() ? findCli() : executable;
  resize(1580, 1000);
  buildUi();
  buildActions();
  connect(canvas, &MapCanvas::changed, this, &MainWindow::changed);
  connect(canvas, &MapCanvas::selectionChanged, this,
          &MainWindow::selectionChanged);
  connect(canvas, &MapCanvas::regionCreated, this, &MainWindow::addRegion);
  connect(canvas, &MapCanvas::hintChanged, hint_, &QLabel::setText);
  connect(canvas, &MapCanvas::positionChanged, this, [this](QPointF p) {
    coordinate_->setText(QString("map: x=%1, y=%2 m")
                             .arg(p.x(), 0, 'f', 3)
                             .arg(p.y(), 0, 'f', 3));
  });
  connect(&client, &PlannerClient::log, this, &MainWindow::logMessage);
  connect(&client, &PlannerClient::completed, this, &MainWindow::receiveResult);
  connect(&client, &PlannerClient::failed, this, [this](const QString &s) {
    summary_->setText(s);
    logMessage(s);
    emit planningFailed(s);
  });
  connect(&client, &PlannerClient::runningChanged, this, [this](bool running) {
    planButton_->setEnabled(!running);
    cancelButton_->setEnabled(running);
    busy_->setRange(0, running ? 0 : 1);
    busy_->setValue(0);
  });
  timer_.setInterval(40);
  connect(&timer_, &QTimer::timeout, this, [this] {
    if (!playing_ || result.isEmpty())
      return;
    seek(std::min(result["length"].toDouble(), currentProgress + .04));
    if (currentProgress >= result["length"].toDouble()) {
      playing_ = false;
      playButton_->setText("▶ 回放");
    }
  });
  timer_.start();
  canvas->setTool("select");
  syncAll();
  QTimer::singleShot(0, canvas, &MapCanvas::fitMap);
}
void MainWindow::buildUi() {
  auto outer = new QWidget;
  setCentralWidget(outer);
  auto layout = new QVBoxLayout(outer);
  layout->setContentsMargins(10, 8, 10, 6);
  auto top = new QHBoxLayout;
  auto title = new QLabel("PASSAGE STUDIO · C++");
  title->setStyleSheet("font-size:18px;font-weight:700;color:#76d5e1");
  top->addWidget(title);
  top->addSpacing(15);
  auto button = [&](const QString &text, auto fn) {
    auto b = new QPushButton(text);
    connect(b, &QPushButton::clicked, this, fn);
    top->addWidget(b);
    return b;
  };
  button("打开工程", [this] { openDialog(); });
  button("导入地图", [this] { importDialog(); });
  button("新建地图", [this] { newMapDialog(); });
  button("保存", [this] { save(); });
  button("另存为", [this] { save(true); });
  button("适应窗口", [this] { canvas->fitMap(); });
  top->addStretch();
  planButton_ = button("▶ 计算路径 F5", [this] { plan(); });
  planButton_->setObjectName("primary");
  cancelButton_ = button("取消", [this] { client.cancel(); });
  cancelButton_->setEnabled(false);
  button("导出结果", [this] { exportDialog(); });
  layout->addLayout(top);
  busy_ = new QProgressBar;
  busy_->setRange(0, 1);
  busy_->setValue(0);
  busy_->setTextVisible(false);
  busy_->setMaximumHeight(4);
  layout->addWidget(busy_);
  auto splitter = new QSplitter;
  layout->addWidget(splitter, 1);
  auto left = new QWidget;
  auto lv = new QVBoxLayout(left);
  lv->setContentsMargins(0, 0, 7, 0);
  left->setMinimumWidth(220);
  left->setMaximumWidth(270);
  auto tools = new QGroupBox("编辑工具");
  auto grid = new QGridLayout(tools);
  auto group = new QButtonGroup(this);
  int i = 0;
  const QVector<QPair<QString, QString>> names{
      {"select", "选择 / 拖动"}, {"region", "特殊区域"},  {"start", "设置起点"},
      {"goal", "设置终点"},      {"blocker", "真实障碍"}, {"pan", "平移视图"},
      {"wall", "画静态墙"},      {"erase", "擦除静态墙"}};
  for (const auto &n : names) {
    auto b = new QPushButton(n.second);
    b->setCheckable(true);
    b->setChecked(n.first == "select");
    b->setObjectName("tool_" + n.first);
    group->addButton(b);
    connect(b, &QPushButton::clicked, this,
            [this, n] { canvas->setTool(n.first); });
    grid->addWidget(b, i / 2, i % 2);
    ++i;
  }
  lv->addWidget(tools);
  auto brush = new QHBoxLayout;
  brush->addWidget(new QLabel("画笔半径 / m"));
  auto brushSpin = spin(.12, .02, 2., 2);
  brush->addWidget(brushSpin);
  connect(brushSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          [this](double v) { canvas->brushRadius = v; });
  lv->addLayout(brush);
  lv->addWidget(new QLabel("通道与真实障碍"));
  items_ = new QListWidget;
  items_->setObjectName("regionList");
  lv->addWidget(items_, 1);
  connect(items_, &QListWidget::currentRowChanged, this, [this](int row) {
    if (updating_)
      return;
    int n = doc.data["passages"].toArray().size();
    if (row < 0)
      canvas->setSelection("none", -1);
    else
      canvas->setSelection(row < n ? "passage" : "blocker",
                           row < n ? row : row - n);
  });
  auto editRow = new QHBoxLayout;
  auto dup = new QPushButton("复制通道"), del = new QPushButton("删除");
  editRow->addWidget(dup);
  editRow->addWidget(del);
  lv->addLayout(editRow);
  connect(dup, &QPushButton::clicked, this, &MainWindow::duplicate);
  connect(del, &QPushButton::clicked, this, &MainWindow::deleteSelection);
  auto layerBox = new QGroupBox("显示图层");
  auto gl = new QVBoxLayout(layerBox);
  background_ = new QComboBox;
  background_->addItems(
      {"原始地图", "处理后可通行地图", "普通规划禁行图", "距离场"});
  gl->addWidget(background_);
  connect(background_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &MainWindow::setBackground);
  for (const auto &n :
       QVector<QPair<QString, QString>>{{"graph", "候选图连接"},
                                        {"raw", "A* 原始路径"},
                                        {"simple", "化简路径"},
                                        {"corridors", "优化路点区域"},
                                        {"final", "最终路径"},
                                        {"events", "变形 / 释放事件"}}) {
    auto cb = new QCheckBox(n.second);
    cb->setChecked(n.first == "final" || n.first == "events");
    cb->setObjectName("layer_" + n.first);
    gl->addWidget(cb);
    connect(cb, &QCheckBox::toggled, this, [this, n](bool v) {
      canvas->layers[n.first] = v;
      canvas->update();
    });
  }
  lv->addWidget(layerBox);
  auto example = new QComboBox;
  example->addItems(
      {"打开内置示例…", "两条通道选路", "旋转通道", "连续穿过两洞"});
  lv->addWidget(example);
  connect(example, qOverload<int>(&QComboBox::activated), this, [this](int n) {
    if (n && canDiscard())
      loadExample(n);
  });
  splitter->addWidget(left);
  auto middle = new QWidget;
  auto mv = new QVBoxLayout(middle);
  mv->setContentsMargins(0, 0, 0, 0);
  summary_ = new QLabel("地图编辑器独立运行；选择规划程序后按 F5 测试。");
  summary_->setWordWrap(true);
  summary_->setStyleSheet("padding:7px;background:#1b2c3d;color:#bad2e0");
  mv->addWidget(summary_);
  canvas = new MapCanvas(&doc);
  mv->addWidget(canvas, 1);
  hint_ = new QLabel;
  hint_->setWordWrap(true);
  hint_->setStyleSheet("color:#92acbe;padding:4px");
  mv->addWidget(hint_);
  splitter->addWidget(middle);
  auto tabs = new QTabWidget;
  tabs->setMinimumWidth(300);
  tabs->setMaximumWidth(350);
  auto property = new QWidget;
  auto pv = new QVBoxLayout(property);
  propertyTitle_ = new QLabel("选择一个通道或障碍");
  propertyTitle_->setStyleSheet("font-size:14px;font-weight:600;color:#f1c47b");
  pv->addWidget(propertyTitle_);
  auto form = new QFormLayout;
  id_ = new QLineEdit;
  id_->setObjectName("passageId");
  form->addRow("通道编号", id_);
  const QVector<QPair<QString, QString>> props{
      {"cx", "中心 x / m"},
      {"cy", "中心 y / m"},
      {"length", "长轴长度 / m"},
      {"width", "宽度 / m"},
      {"angle", "角度 / °"},
      {"ax", "A.x / m"},
      {"ay", "A.y / m"},
      {"bx", "B.x / m"},
      {"by", "B.y / m"},
      {"prepare_distance", "提前变形 / m"},
      {"release_distance", "出口后释放 / m"},
      {"speed", "穿洞速度 / m/s"},
      {"action_cost_seconds", "动作附加代价 / s"},
      {"fold_hold_seconds", "变形后停留 / s"}, {"restore_hold_seconds", "恢复后停留 / s"},
      {"beam_min_height", "门梁最低相对高度 / m"}, {"beam_max_height", "门梁最高相对高度 / m"}};
  for (const auto &n : props) {
    double lo = -10000.;
    if (n.first == "length" || n.first == "width")
      lo = .05;
    else if (n.first == "speed")
      lo = .01;
    else if (n.first.endsWith("distance") || n.first == "action_cost_seconds" || n.first.endsWith("seconds") || n.first.endsWith("height"))
      lo = 0.;
    auto w = spin(0., lo, n.first == "angle" ? 36000 : 10000);
    w->setObjectName("field_" + n.first);
    fields_[n.first] = w;
    form->addRow(n.second, w);
  }
  direction_ = new QComboBox;
  direction_->addItems({"双向 A ↔ B", "仅 A → B", "仅 B → A"});
  form->addRow("通行方向", direction_);
  mode_ = new QLineEdit("folded");
  form->addRow("变形模式标识", mode_);
  enabled_ = new QCheckBox("启用此通道");
  enabled_->setChecked(true);
  form->addRow(enabled_);
  clearStatic_ = new QCheckBox("清除区域内静态投影（横梁）");
  clearStatic_->setChecked(true);
  form->addRow(clearStatic_);
  clearUnknown_ = new QCheckBox("同时清除区域内静态未知栅格");
  clearUnknown_->setToolTip("仅用于已确认的门洞投影；不清除 Nav2 合并后的实时障碍。默认保留未知。");
  form->addRow(clearUnknown_);
  pv->addLayout(form);
  auto apply = new QPushButton("应用属性");
  apply->setObjectName("applyProperties");
  connect(apply, &QPushButton::clicked, this, &MainWindow::applyProperties);
  pv->addWidget(apply);
  auto align = new QPushButton("按区域自动对齐 A / B");
  connect(align, &QPushButton::clicked, this, &MainWindow::alignEndpoints);
  pv->addWidget(align);
  auto ab = new QHBoxLayout;
  for (const auto &n : QVector<QPair<QString, QString>>{
           {"pick_a", "地图点选 A"}, {"pick_b", "地图点选 B"}}) {
    auto b = new QPushButton(n.second);
    ab->addWidget(b);
    connect(b, &QPushButton::clicked, this,
            [this, n] { canvas->setTool(n.first); });
  }
  pv->addLayout(ab);
  pv->addStretch();
  auto scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(property);
  tabs->addTab(scroll, "区域属性");
  auto settings = new QWidget;
  auto sv = new QVBoxLayout(settings);
  auto sf = new QFormLayout;
  const QVector<QPair<QString, QString>> settingsNames{
      {"robot_radius", "圆形机器人半径 / m"},
      {"safety_margin", "附加边界 / m"},
      {"obstacle_weight", "远墙代价权重"},
      {"obstacle_decay", "距离代价衰减 / m"},
      {"cruise_speed", "普通路段参考速度"},
      {"max_speed", "优化速度上限"},
      {"max_acceleration", "优化加速度上限"},
      {"corridor_range", "路点区域扩张范围"},
      {"graph_budget_ms", "图搜索预算 / ms"}};
  for (const auto &n : settingsNames) {
    auto w =
        spin(1.,
             n.first == "safety_margin" || n.first == "robot_radius" ||
                     n.first == "obstacle_weight" || n.first == "corridor_range"
                 ? 0.
                 : .001,
             n.first == "graph_budget_ms" ? 600000 : 100.);
    settings_[n.first] = w;
    sf->addRow(n.second, w);
  }
  optimize_ = new QCheckBox("启用 TDT OSQP 平滑");
  fallback_ = new QCheckBox("优化失败时保留已验证 A* 路径");
  sf->addRow(optimize_);
  sf->addRow(fallback_);
  sv->addLayout(sf);
  auto mapBox = new QGroupBox("地图坐标与阈值");
  auto mf = new QFormLayout(mapBox);
  for (const auto &n :
       QVector<QPair<QString, QString>>{{"resolution", "分辨率 / m"},
                                        {"ox", "原点 x"},
                                        {"oy", "原点 y"},
                                        {"yaw", "原点角度 / °"},
                                        {"occupied_max", "障碍灰度上限"},
                                        {"free_min", "自由灰度下限"}}) {
    bool threshold = n.first.endsWith("max") || n.first.endsWith("min");
    auto w = spin(0.,
                  threshold                 ? 0.
                  : n.first == "resolution" ? .001
                                            : -36000.,
                  threshold ? 255. : 36000., threshold ? 0 : 3);
    mapFields_[n.first] = w;
    mf->addRow(n.second, w);
  }
  sv->addWidget(mapBox);
  auto applySettingsButton = new QPushButton("应用规划与地图参数");
  connect(applySettingsButton, &QPushButton::clicked, this,
          &MainWindow::applySettings);
  sv->addWidget(applySettingsButton);
  auto cliBox = new QGroupBox("外部规划程序");
  auto cv = new QVBoxLayout(cliBox);
  cliField_ = new QLineEdit(client.executable);
  cliField_->setObjectName("plannerExecutable");
  cliField_->setPlaceholderText("可不配置，独立编辑和保存地图");
  cv->addWidget(cliField_);
  auto choose = new QPushButton("选择 passage_cli…");
  cv->addWidget(choose);
  connect(choose, &QPushButton::clicked, this, [this] {
    auto p = QFileDialog::getOpenFileName(this, "选择独立规划程序",
                                          cliField_->text());
    if (!p.isEmpty())
      cliField_->setText(p);
  });
  sv->addWidget(cliBox);
  auto tip = new QLabel(
      "编辑器只交换工程 / 结果 JSON。\n回放按几何弧长推进，不连接机械控制。");
  tip->setWordWrap(true);
  sv->addWidget(tip);
  sv->addStretch();
  auto settingsScroll = new QScrollArea;
  settingsScroll->setWidgetResizable(true);
  settingsScroll->setWidget(settings);
  tabs->addTab(settingsScroll, "规划参数");
  log_ = new QPlainTextEdit;
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(2000);
  tabs->addTab(log_, "日志");
  splitter->addWidget(tabs);
  splitter->setStretchFactor(1, 1);
  auto playback = new QHBoxLayout;
  playButton_ = new QPushButton("▶ 回放");
  connect(playButton_, &QPushButton::clicked, this, &MainWindow::togglePlay);
  playback->addWidget(playButton_);
  slider_ = new QSlider(Qt::Horizontal);
  slider_->setRange(0, 10000);
  slider_->setObjectName("playbackSlider");
  playback->addWidget(slider_, 1);
  connect(slider_, &QSlider::valueChanged, this, [this](int v) {
    if (!result.isEmpty())
      seek(result["length"].toDouble() * v / 10000.);
  });
  progressLabel_ = new QLabel("s = 0.000 m");
  stateLabel_ = new QLabel("NORMAL · fold_request=0");
  stateLabel_->setMinimumWidth(315);
  playback->addWidget(progressLabel_);
  playback->addWidget(stateLabel_);
  layout->addLayout(playback);
  coordinate_ = new QLabel("map: —");
  statusBar()->addPermanentWidget(coordinate_);
}
void MainWindow::buildActions() {
  auto file = menuBar()->addMenu("文件"), edit = menuBar()->addMenu("编辑"),
       run = menuBar()->addMenu("规划");
  auto action = [&](QMenu *menu, const QString &name, const QKeySequence &key,
                    auto fn) {
    auto a = new QAction(name, this);
    a->setShortcut(key);
    connect(a, &QAction::triggered, this, fn);
    menu->addAction(a);
  };
  action(file, "打开工程", QKeySequence::Open, [this] { openDialog(); });
  action(file, "导入地图", QKeySequence("Ctrl+I"), [this] { importDialog(); });
  action(file, "保存", QKeySequence::Save, [this] { save(); });
  action(file, "另存为", QKeySequence::SaveAs, [this] { save(true); });
  action(file, "导出导航通道文件", QKeySequence("Ctrl+Shift+E"), [this] {
    QString path = QFileDialog::getSaveFileName(this, "导出导航通道", "map.passages.json", "JSON (*.json)");
    if (path.isEmpty()) return;
    try { doc.exportPassages(path); logMessage("已导出通道：" + path); } catch (const std::exception &e) { showError(e.what()); }
  });
  action(file, "导入导航通道文件", QKeySequence("Ctrl+Shift+I"), [this] {
    QString path = QFileDialog::getOpenFileName(this, "导入导航通道", {}, "JSON (*.json)");
    if (path.isEmpty()) return;
    try { doc.importPassages(path); changed(); } catch (const std::exception &e) { showError(e.what()); }
  });
  action(file, "导出结果", QKeySequence("Ctrl+E"), [this] { exportDialog(); });
  action(edit, "撤销", QKeySequence::Undo, [this] { undo(); });
  action(edit, "重做", QKeySequence("Ctrl+Y"), [this] { redo(); });
  action(edit, "删除选区", QKeySequence::Delete, [this] { deleteSelection(); });
  action(run, "计算路径", QKeySequence("F5"), [this] { plan(); });
  action(run, "播放 / 暂停", QKeySequence("Space"), [this] { togglePlay(); });
  action(run, "适应窗口", QKeySequence("F"), [this] { canvas->fitMap(); });
  auto help = menuBar()->addMenu("帮助");
  action(help, "操作说明", QKeySequence("F1"), [this] {
    QMessageBox::information(
        this, "操作说明",
        "旋转矩形：点击长轴两端，再点击宽度。\n拖中心移动、角点缩放、蓝点旋转；"
        "A/B 可独立拖动。\n中键平移，滚轮缩放，Esc 取消。\n"
        "区域只覆盖需要清除的横梁投影。真实障碍和未知格不会被清除。\n工程改动使"
        "旧规划失效；规划时可以继续编辑，旧版本结果会被丢弃。\n"
        "编辑器独立于规划器。先构建并选择 passage_cli，再按 "
        "F5。\n回放只展示状态请求，实际变形完成情况由未来控制层反馈。");
  });
}
void MainWindow::syncAll() {
  updating_ = true;
  items_->clear();
  auto a = doc.data["passages"].toArray(),
       blocks = doc.data["blockers"].toArray();
  for (const auto &v : a) {
    auto g = v.toObject();
    auto direction = g["direction"].toString();
    items_->addItem(QString(g["enabled"].toBool(true) ? "● " : "○ ") +
                    g["id"].toString() + "  " +
                    (direction == "both"     ? "↔"
                     : direction == "a_to_b" ? "→"
                                             : "←"));
  }
  for (int i = 0; i < blocks.size(); ++i)
    items_->addItem(QString("■ 真实障碍 %1").arg(i + 1));
  if (canvas->selectedGate >= a.size())
    canvas->selectedGate = -1;
  if (canvas->selectedBlocker >= blocks.size())
    canvas->selectedBlocker = -1;
  int row = canvas->selectedGate >= 0      ? canvas->selectedGate
            : canvas->selectedBlocker >= 0 ? a.size() + canvas->selectedBlocker
                                           : -1;
  items_->setCurrentRow(row);
  auto cfg = doc.data["planner"].toObject();
  for (auto i = settings_.begin(); i != settings_.end(); ++i)
    i.value()->setValue(cfg[i.key()].toDouble());
  optimize_->setChecked(cfg["optimize"].toBool(true));
  fallback_->setChecked(cfg["allow_geometric_fallback"].toBool(true));
  auto m = doc.data["map"].toObject();
  auto origin = point(m["origin"]);
  m["ox"] = origin.x();
  m["oy"] = origin.y();
  m["yaw"] = m["yaw"].toDouble() * 180 / Pi;
  for (auto i = mapFields_.begin(); i != mapFields_.end(); ++i)
    i.value()->setValue(m[i.key()].toDouble());
  syncProperties();
  canvas->update();
  updating_ = false;
  setWindowTitle((doc.dirty ? "* " : "") +
                 (doc.filename.isEmpty() ? "未命名工程"
                                         : QFileInfo(doc.filename).fileName()) +
                 " — Passage Studio C++ Qt");
}
void MainWindow::syncProperties() {
  auto s = canvas->selected();
  auto g = s.first, r = s.second;
  bool isGate = !g.isEmpty(), valid = !r.isEmpty();
  propertyTitle_->setText(
      !valid   ? "选择一个通道或障碍"
      : isGate ? g["id"].toString()
               : QString("真实障碍 %1").arg(canvas->selectedBlocker + 1));
  const QStringList geometry{"cx", "cy", "length", "width", "angle"};
  for (auto i = fields_.begin(); i != fields_.end(); ++i)
    i.value()->setEnabled(valid && (isGate || geometry.contains(i.key())));
  for (auto w :
       QVector<QWidget *>{id_, mode_, direction_, enabled_, clearStatic_})
    w->setEnabled(isGate);
  if (!valid)
    return;
  auto c = point(r["center"]);
  QMap<QString, double> vals{{"cx", c.x()},
                             {"cy", c.y()},
                             {"length", r["length"].toDouble()},
                             {"width", r["width"].toDouble()},
                             {"angle", r["yaw"].toDouble() * 180 / Pi}};
  if (isGate) {
    auto a = point(g["endpoint_a"]), b = point(g["endpoint_b"]);
    vals["ax"] = a.x();
    vals["ay"] = a.y();
    vals["bx"] = b.x();
    vals["by"] = b.y();
    for (const auto &key : {"prepare_distance", "release_distance", "speed",
                            "action_cost_seconds", "fold_hold_seconds", "restore_hold_seconds", "beam_min_height", "beam_max_height"})
      vals[key] = g[key].toDouble(QString(key) == "beam_min_height" ? .26 : QString(key) == "beam_max_height" ? 2. : QString(key).endsWith("hold_seconds") ? .3 : 0.);
    id_->setText(g["id"].toString());
    mode_->setText(g["mode"].toString("folded"));
    direction_->setCurrentIndex(QStringList{"both", "a_to_b", "b_to_a"}.indexOf(
        g["direction"].toString("both")));
    enabled_->setChecked(g["enabled"].toBool(true));
    clearStatic_->setChecked(g["clear_static"].toBool(true));
    clearUnknown_->setChecked(g["clear_unknown"].toBool(false));
  }
  for (auto i = vals.begin(); i != vals.end(); ++i)
    fields_[i.key()]->setValue(i.value());
}
void MainWindow::selectionChanged(QString kind, int index) {
  updating_ = true;
  items_->setCurrentRow(kind == "passage" ? index
                        : kind == "blocker"
                            ? doc.data["passages"].toArray().size() + index
                            : -1);
  updating_ = false;
  syncProperties();
}
void MainWindow::invalidate() {
  result = {};
  canvas->result = {};
  canvas->background = {};
  canvas->progress = -1;
  currentProgress = 0;
  playing_ = false;
  playButton_->setText("▶ 回放");
  {
    QSignalBlocker b(slider_);
    slider_->setValue(0);
  }
  progressLabel_->setText("s = 0.000 m");
  stateLabel_->setText("NORMAL · fold_request=0");
  canvas->update();
}
void MainWindow::changed() {
  invalidate();
  summary_->setText("工程已修改；按 F5 重新规划。");
  syncAll();
}
void MainWindow::addRegion(QString kind, QJsonObject r) {
  doc.checkpoint();
  if (kind == "region") {
    auto a = doc.data["passages"].toArray();
    a.append(newPassage(r, doc.data));
    doc.data["passages"] = a;
    canvas->setSelection("passage", a.size() - 1);
  } else {
    auto a = doc.data["blockers"].toArray();
    a.append(r);
    doc.data["blockers"] = a;
    canvas->setSelection("blocker", a.size() - 1);
  }
  doc.touch();
  changed();
}
void MainWindow::deleteSelection() {
  if (canvas->selected().second.isEmpty())
    return;
  doc.checkpoint();
  QString key = canvas->selectedGate >= 0 ? "passages" : "blockers";
  int i = canvas->selectedGate >= 0 ? canvas->selectedGate
                                    : canvas->selectedBlocker;
  auto a = doc.data[key].toArray();
  a.removeAt(i);
  doc.data[key] = a;
  canvas->setSelection("none", -1);
  doc.touch();
  changed();
}
void MainWindow::duplicate() {
  auto s = canvas->selected();
  if (s.first.isEmpty())
    return;
  auto g = s.first, r = s.second;
  auto center = point(r["center"]) + QPointF(.6, .6);
  r["center"] = jsonPoint(center);
  auto unique = newPassage(r, doc.data);
  g["id"] = unique["id"];
  g["region"] = r;
  autoEndpoints(g, doc.data);
  doc.checkpoint();
  auto a = doc.data["passages"].toArray();
  a.append(g);
  doc.data["passages"] = a;
  canvas->setSelection("passage", a.size() - 1);
  doc.touch();
  changed();
}
void MainWindow::alignEndpoints() {
  auto s = canvas->selected();
  if (s.first.isEmpty())
    return;
  doc.checkpoint();
  auto g = s.first;
  autoEndpoints(g, doc.data);
  canvas->updateSelected(g, s.second);
  doc.touch();
  changed();
}
void MainWindow::applyProperties() {
  auto s = canvas->selected();
  auto g = s.first, r = s.second;
  if (r.isEmpty())
    return;
  auto value = [this](const QString &key) { return fields_[key]->value(); };
  if (!g.isEmpty()) {
    if (id_->text().trimmed().isEmpty()) {
      showError("通道编号不能为空");
      return;
    }
    auto gates = doc.data["passages"].toArray();
    for (int i = 0; i < gates.size(); ++i)
      if (i != canvas->selectedGate &&
          gates[i].toObject()["id"].toString() == id_->text().trimmed()) {
        showError("通道编号重复");
        return;
      }
  }
  doc.checkpoint();
  auto old = r;
  r["center"] = QJsonArray{value("cx"), value("cy")};
  r["length"] = value("length");
  r["width"] = value("width");
  r["yaw"] = value("angle") * Pi / 180;
  if (!g.isEmpty()) {
    auto a = point(g["endpoint_a"]), b = point(g["endpoint_b"]);
    QPointF nextA(value("ax"), value("ay")), nextB(value("bx"), value("by"));
    bool endpointsEdited = norm(a - nextA) > .0008 || norm(b - nextB) > .0008;
    g["region"] = r;
    if (endpointsEdited) {
      g["endpoint_a"] = jsonPoint(nextA);
      g["endpoint_b"] = jsonPoint(nextB);
      g["auto_endpoints"] = false;
    } else if (g["auto_endpoints"].toBool(true))
      autoEndpoints(g, doc.data);
    else {
      double delta = r["yaw"].toDouble() - old["yaw"].toDouble();
      for (const auto &key : {"endpoint_a", "endpoint_b"}) {
        auto v = point(g[key]) - point(old["center"]);
        g[key] = jsonPoint(point(r["center"]) + axis(delta) * v.x() +
                           axis(delta + Pi / 2) * v.y());
      }
    }
    g["id"] = id_->text().trimmed();
    g["direction"] = QStringList{
        "both", "a_to_b", "b_to_a"}[std::max(0, direction_->currentIndex())];
    g["mode"] = mode_->text();
    g["enabled"] = enabled_->isChecked();
    g["clear_static"] = clearStatic_->isChecked();
    g["clear_unknown"] = clearUnknown_->isChecked();
    for (const auto &key : {"prepare_distance", "release_distance", "speed",
                            "action_cost_seconds", "fold_hold_seconds", "restore_hold_seconds", "beam_min_height", "beam_max_height"})
      g[key] = value(key);
  }
  canvas->updateSelected(g, r);
  doc.touch();
  changed();
}
void MainWindow::applySettings() {
  if (mapFields_["occupied_max"]->value() >= mapFields_["free_min"]->value()) {
    showError("障碍灰度上限必须小于自由灰度下限");
    return;
  }
  doc.checkpoint();
  auto cfg = doc.data["planner"].toObject(), m = doc.data["map"].toObject();
  for (auto i = settings_.begin(); i != settings_.end(); ++i)
    cfg[i.key()] = i.value()->value();
  cfg["optimize"] = optimize_->isChecked();
  cfg["allow_geometric_fallback"] = fallback_->isChecked();
  for (const auto &key : {"resolution", "occupied_max", "free_min"})
    m[key] = mapFields_[key]->value();
  m["origin"] =
      QJsonArray{mapFields_["ox"]->value(), mapFields_["oy"]->value()};
  m["yaw"] = mapFields_["yaw"]->value() * Pi / 180;
  doc.data["planner"] = cfg;
  doc.data["map"] = m;
  auto gates = doc.data["passages"].toArray();
  for (int i = 0; i < gates.size(); ++i) {
    auto g = gates[i].toObject();
    if (g["auto_endpoints"].toBool(true)) {
      autoEndpoints(g, doc.data);
      gates[i] = g;
    }
  }
  doc.data["passages"] = gates;
  doc.touch();
  changed();
}
void MainWindow::undo() {
  if (doc.undo()) {
    canvas->setSelection("none", -1);
    changed();
  }
}
void MainWindow::redo() {
  if (doc.redo()) {
    canvas->setSelection("none", -1);
    changed();
  }
}
void MainWindow::openProject(const QString &path) {
  try {
    doc.open(path);
    canvas->setSelection("none", -1);
    canvas->setTool("select");
    invalidate();
    syncAll();
    canvas->fitMap();
    summary_->setText("已打开 " + QFileInfo(path).fileName() +
                      "；按 F5 测试。");
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
void MainWindow::importMap(const QString &path) {
  try {
    doc.importMap(path);
    canvas->setSelection("none", -1);
    invalidate();
    syncAll();
    canvas->fitMap();
    summary_->setText("已导入地图，设置特殊区域后保存工程。");
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
void MainWindow::openDialog() {
  if (!canDiscard())
    return;
  auto p = QFileDialog::getOpenFileName(this, "打开工程", {}, "工程 (*.json)");
  if (!p.isEmpty())
    openProject(p);
}
void MainWindow::importDialog() {
  if (!canDiscard())
    return;
  auto p = QFileDialog::getOpenFileName(
      this, "导入地图", {},
      "地图 (*.yaml *.yml *.png *.pgm *.jpg *.jpeg *.bmp)");
  if (!p.isEmpty())
    importMap(p);
}
void MainWindow::save(bool saveAs) {
  QString path = doc.filename;
  if (path.isEmpty() || saveAs)
    path = QFileDialog::getSaveFileName(this, "保存工程",
                                        path.isEmpty() ? "project.json" : path,
                                        "工程 (*.json)");
  if (path.isEmpty())
    return;
  if (!path.endsWith(".json", Qt::CaseInsensitive))
    path += ".json";
  try {
    doc.save(path);
    syncAll();
    logMessage("已保存工程和地图：" + path);
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
bool MainWindow::canDiscard() {
  if (!doc.dirty)
    return true;
  auto choice = QMessageBox::question(
      this, "未保存的修改", "是否保存当前工程？",
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Save);
  if (choice == QMessageBox::Cancel)
    return false;
  if (choice == QMessageBox::Save) {
    save();
    return !doc.dirty;
  }
  return true;
}
void MainWindow::newMapDialog() {
  if (!canDiscard())
    return;
  QDialog dialog(this);
  dialog.setWindowTitle("新建地图");
  auto v = new QVBoxLayout(&dialog);
  auto f = new QFormLayout;
  auto w = new QSpinBox, h = new QSpinBox;
  w->setRange(32, 20000);
  h->setRange(32, 20000);
  w->setValue(400);
  h->setValue(240);
  auto res = spin(.05, .001, 10.);
  f->addRow("宽度 / 像素", w);
  f->addRow("高度 / 像素", h);
  f->addRow("分辨率 / m", res);
  v->addLayout(f);
  auto buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  v->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted)
    return;
  try {
    doc.newMap(w->value(), h->value(), res->value());
    canvas->setSelection("none", -1);
    invalidate();
    syncAll();
    canvas->fitMap();
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
  }
}
QString MainWindow::findExamples() {
  QString env = qEnvironmentVariable("TDT_EXAMPLES_DIR");
  if (QFileInfo(QDir(env).filePath("two_passages.json")).isFile())
    return env;
  for (const auto &base : QStringList{QDir::currentPath(),
                                      QCoreApplication::applicationDirPath()}) {
    QDir d(base);
    for (int i = 0; i < 6; ++i) {
      auto p = d.filePath("examples");
      if (QFileInfo(QDir(p).filePath("two_passages.json")).isFile())
        return p;
      if (!d.cdUp())
        break;
    }
  }
  return {};
}
void MainWindow::loadExample(int index) {
  QStringList names{"", "two_passages", "rotated_passage", "chain_passages"};
  auto dir = findExamples();
  if (index < 1 || index >= names.size())
    return;
  if (dir.isEmpty()) {
    showError("未找到示例目录。请直接打开工程，或设置 TDT_EXAMPLES_DIR。");
    return;
  }
  openProject(QDir(dir).filePath(names[index] + ".json"));
}
void MainWindow::plan() {
  if (client.busy())
    return;
  client.executable = cliField_->text().trimmed();
  QSettings().setValue("planner/executable", client.executable);
  invalidate();
  summary_->setText("正在调用独立规划程序…");
  client.start(doc);
}
void MainWindow::receiveResult(QJsonObject r, quint64 revision) {
  if (revision != doc.revision) {
    logMessage("规划期间工程已变化，已丢弃旧版本结果。");
    summary_->setText("工程已变化，请重新规划。");
    emit planningFailed("stale_result");
    return;
  }
  result = r;
  resultRevision_ = revision;
  canvas->result = r;
  logMessage(r["status"].toString() + ": " + r["message"].toString());
  for (const auto &v : r["warnings"].toArray())
    logMessage("warning: " + v.toString());
  if (r["success"].toBool()) {
    summary_->setText(QString("路径 %1 m   特殊通道 %2 个   计算 %3 ms")
                          .arg(r["length"].toDouble(), 0, 'f', 2)
                          .arg(r["traversals"].toArray().size())
                          .arg(r["elapsed_ms"].toDouble(), 0, 'f', 1));
    seek(0.);
  } else
    summary_->setText(r["status"].toString() + "：" + r["message"].toString());
  setBackground(background_->currentIndex());
  canvas->update();
  emit planningFinished(r);
}
void MainWindow::setBackground(int index) {
  if (background_->currentIndex() != index) {
    QSignalBlocker blocker(background_);
    background_->setCurrentIndex(index);
  }
  static const QStringList files{"", "01_passable.png", "02_ordinary.png",
                                 "03_distance.png"};
  canvas->background = {};
  if (index > 0 && index < files.size() && !result.isEmpty())
    canvas->background =
        QImage(QDir(client.runDirectory()).filePath("debug/" + files[index]));
  canvas->update();
}
void MainWindow::seek(double s) {
  if (!result["success"].toBool())
    return;
  currentProgress = std::clamp(s, 0., result["length"].toDouble());
  auto state = replayState(result, currentProgress);
  canvas->progress = currentProgress;
  canvas->update();
  {
    QSignalBlocker b(slider_);
    slider_->setValue(
        result["length"].toDouble() > 0
            ? int(currentProgress / result["length"].toDouble() * 10000)
            : 0);
  }
  progressLabel_->setText(QString("s = %1 m").arg(currentProgress, 0, 'f', 3));
  stateLabel_->setText(QString("%1 · fold_request=%2 · lock=%3")
                           .arg(state.phase)
                           .arg(state.folded ? 1 : 0)
                           .arg(state.locked ? 1 : 0));
  stateLabel_->setStyleSheet(state.folded ? "color:#f0b34f;font-weight:600"
                                          : "color:#8de2bb");
}
void MainWindow::togglePlay() {
  if (!result["success"].toBool())
    return;
  if (currentProgress >= result["length"].toDouble())
    seek(0.);
  playing_ = !playing_;
  playButton_->setText(playing_ ? "暂停" : "▶ 回放");
}
bool MainWindow::exportTo(const QString &directory) {
  if (result.isEmpty() || resultRevision_ != doc.revision || client.busy()) {
    logMessage("请先完成当前工程版本的规划");
    return false;
  }
  try {
    if (QFileInfo(directory).exists())
      throw std::runtime_error("导出目标已存在，请使用新目录");
    copyTree(client.runDirectory(), directory);
    logMessage("已导出：" + directory);
    return true;
  } catch (const std::exception &e) {
    showError(QString::fromUtf8(e.what()));
    return false;
  }
}
void MainWindow::exportDialog() {
  if (result.isEmpty()) {
    showError("请先计算路径");
    return;
  }
  auto parent = QFileDialog::getExistingDirectory(this, "选择导出父目录");
  if (parent.isEmpty())
    return;
  exportTo(QDir(parent).filePath(
      "passage_" +
      QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")));
}
void MainWindow::logMessage(const QString &s) {
  if (!s.trimmed().isEmpty())
    log_->appendPlainText(s.trimmed());
}
void MainWindow::showError(const QString &s) {
  logMessage(s);
  summary_->setText(s);
  emit planningFailed(s);
}
void MainWindow::closeEvent(QCloseEvent *e) {
  if (canDiscard()) {
    client.cancel();
    e->accept();
  } else
    e->ignore();
}
} // namespace passage_editor
