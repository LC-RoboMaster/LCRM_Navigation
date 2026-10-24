#include "main_window.hpp"
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>
#include <cmath>

using namespace passage_editor;
namespace {
QString fixture() {
  return QCoreApplication::applicationDirPath() + "/editor_protocol_fixture";
}
QString examples() { return qEnvironmentVariable("TDT_EXAMPLES_DIR"); }
QString cli() { return qEnvironmentVariable("TDT_PASSAGE_CLI"); }
QJsonObject rectangle(QPointF c = {6, 4}, double yaw = 0) {
  return {
      {"center", jsonPoint(c)}, {"length", 1.}, {"width", 1.4}, {"yaw", yaw}};
}
void moveMouse(QWidget *widget, QPoint from, QPoint to) {
  QTest::mousePress(widget, Qt::LeftButton, Qt::NoModifier, from);
  QMouseEvent event(QEvent::MouseMove, QPointF(to), Qt::NoButton,
                    Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(widget, &event);
  QTest::mouseRelease(widget, Qt::LeftButton, Qt::NoModifier, to);
  QApplication::processEvents();
}
} // namespace
class EditorTests : public QObject {
  Q_OBJECT
private slots:
  void projectRoundtripAndCoordinates() {
    Document d;
    d.newMap(320, 240, .05);
    auto m = d.data["map"].toObject();
    m["origin"] = QJsonArray{-4.27, -12.3};
    m["yaw"] = .61;
    d.data["map"] = m;
    for (auto pixel : QVector<QPointF>{{0, 0}, {100.5, 55.5}, {319.9, 239.9}})
      QVERIFY(norm(d.worldToPixel(d.pixelToWorld(pixel)) - pixel) < 1e-8);
    QTemporaryDir dir;
    d.save(dir.filePath("portable.json"));
    Document e;
    e.open(dir.filePath("portable.json"));
    QCOMPARE(e.data, d.data);
    QCOMPARE(e.image, d.image);
    QVERIFY(
        QFileInfo(d.data["map"].toObject()["image"].toString()).isRelative());
  }
  void channelExportImport() {
    QTemporaryDir dir; Document d;
    d.data["passages"] = QJsonArray{newPassage(rectangle(), d.data)};
    d.exportPassages(dir.filePath("field.passages.json"));
    const auto j = readObject(dir.filePath("field.passages.json"));
    QVERIFY(!j.contains("map")); QVERIFY(!j.contains("blockers"));
    QCOMPARE(j["map_reference"].toObject()["resolution"].toDouble(), .05);
    Document e; e.importPassages(dir.filePath("field.passages.json"));
    QCOMPARE(e.data["passages"], d.data["passages"]);
    QVERIFY(e.undo()); QVERIFY(e.data["passages"].toArray().isEmpty());
  }
  void exportRejectsOffAxisEndpointsAndImportRejectsDifferentMap() {
    QTemporaryDir dir; Document d;
    auto g = newPassage(rectangle(), d.data);
    auto a = point(g["endpoint_a"]); a.ry() += .02;
    g["endpoint_a"] = jsonPoint(a);
    d.data["passages"] = QJsonArray{g};
    QVERIFY_EXCEPTION_THROWN(d.exportPassages(dir.filePath("bad.json")), std::runtime_error);
    autoEndpoints(g,d.data); d.data["passages"] = QJsonArray{g};
    d.exportPassages(dir.filePath("valid.json"));
    Document other; auto m = other.data["map"].toObject();
    m["origin"] = QJsonArray{-3.58,-9.44}; other.data["map"] = m;
    QVERIFY_EXCEPTION_THROWN(other.importPassages(dir.filePath("valid.json")), std::runtime_error);
  }
  void yamlImportAndNegate() {
    QTemporaryDir dir;
    QImage image(30, 20, QImage::Format_Grayscale8);
    image.fill(0);
    QVERIFY(image.save(dir.filePath("map.pgm")));
    QFile file(dir.filePath("map.yaml"));
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write("image: map.pgm\nresolution: 0.05\norigin: [-2, 3, "
               "0.4]\nnegate: 1\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
    file.close();
    Document d;
    d.importMap(file.fileName());
    QCOMPARE(d.data["map"].toObject()["yaw"].toDouble(), .4);
    QVERIFY(norm(d.pixelToWorld({0, 20}) - QPointF(-2, 3)) < 1e-9);
    QCOMPARE(d.image.constScanLine(0)[0], uchar(255));
  }
  void drawRotateResizeAndManualEndpoints() {
    MainWindow w(fixture());
    w.show();
    QTest::qWait(20);
    w.canvas->fitMap();
    w.canvas->setTool("region");
    auto click = [&](QPointF p) {
      QTest::mouseClick(w.canvas, Qt::LeftButton, Qt::NoModifier,
                        w.canvas->screen(p).toPoint());
    };
    click({4, 3});
    click({6, 4});
    click({4, 5});
    QCOMPARE(w.doc.data["passages"].toArray().size(), 1);
    w.canvas->setTool("select");
    auto s = w.canvas->selected();
    double initial = s.second["yaw"].toDouble();
    auto c = point(s.second["center"]);
    moveMouse(w.canvas,
              w.canvas->screen(w.canvas->rotationHandle(s.second)).toPoint(),
              w.canvas->screen(c + QPointF(1.5, 0)).toPoint());
    s = w.canvas->selected();
    QVERIFY(std::abs(s.second["yaw"].toDouble() - initial) > .3);
    auto corner = corners(s.second)[2];
    moveMouse(w.canvas, w.canvas->screen(corner).toPoint(),
              w.canvas
                  ->screen(c + axis(s.second["yaw"].toDouble()) * 2 +
                           axis(s.second["yaw"].toDouble() + Pi / 2) * 1.2)
                  .toPoint());
    s = w.canvas->selected();
    QVERIFY(s.second["length"].toDouble() > 3.8);
    QVERIFY(s.second["width"].toDouble() > 2.2);
    w.canvas->setTool("pick_a");
    click(c + QPointF(-2, 1));
    s = w.canvas->selected();
    QVERIFY(!s.first["auto_endpoints"].toBool());
    auto saved = point(s.first["endpoint_a"]);
    QVERIFY(norm(saved - (c + QPointF(-2, 1))) < .05);
    moveMouse(w.canvas, w.canvas->screen(c).toPoint(),
              w.canvas->screen(c + QPointF(.4, .4)).toPoint());
    s = w.canvas->selected();
    QVERIFY(norm(point(s.first["endpoint_a"]) - saved - QPointF(.4, .4)) < .05);
    w.doc.dirty = false;
  }
  void paintUndoAndSave() {
    MainWindow w(fixture());
    w.show();
    QTest::qWait(15);
    w.canvas->fitMap();
    auto position = QPointF(3, 3);
    auto pixel = w.doc.worldToPixel(position).toPoint();
    w.canvas->setTool("wall");
    QTest::mouseClick(w.canvas, Qt::LeftButton, Qt::NoModifier,
                      w.canvas->screen(position).toPoint());
    QVERIFY(w.doc.image.constScanLine(pixel.y())[pixel.x()] < 10);
    QVERIFY(w.doc.undo());
    QVERIFY(w.doc.image.constScanLine(pixel.y())[pixel.x()] > 240);
    QVERIFY(w.doc.redo());
    QTemporaryDir dir;
    w.doc.save(dir.filePath("wall.json"));
    Document d;
    d.open(dir.filePath("wall.json"));
    QCOMPARE(d.image, w.doc.image);
    w.doc.dirty = false;
  }
  void propertiesDirectionsAndUndo() {
    MainWindow w(fixture());
    w.addRegion("region", rectangle());
    auto input = w.findChild<QDoubleSpinBox *>("field_angle");
    QVERIFY(input);
    input->setValue(35.);
    w.applyProperties();
    auto s = w.canvas->selected();
    QVERIFY(std::abs(s.second["yaw"].toDouble() - 35. * Pi / 180) < 1e-9);
    auto a = point(s.first["endpoint_a"]), b = point(s.first["endpoint_b"]),
         center = point(s.second["center"]);
    QVERIFY(norm((a + b) / 2 - center) < 1e-9);
    QVERIFY(w.doc.undo());
    QVERIFY(std::abs(w.doc.data["passages"]
                         .toArray()[0]
                         .toObject()["region"]
                         .toObject()["yaw"]
                         .toDouble()) < 1e-9);
    w.doc.dirty = false;
  }
  void protocolSuccessAndNoRoute() {
    Document d;
    PlannerClient client;
    client.executable = fixture();
    QSignalSpy done(&client, &PlannerClient::completed),
        failed(&client, &PlannerClient::failed);
    QVERIFY(client.start(d));
    QTRY_COMPARE(done.size(), 1);
    QVERIFY(done[0][0].toJsonObject()["success"].toBool());
    QCOMPARE(failed.size(), 0);
    d.data["fixture_mode"] = "no_route";
    QVERIFY(client.start(d));
    QTRY_COMPARE(done.size(), 2);
    QVERIFY(!done[1][0].toJsonObject()["success"].toBool());
    QCOMPARE(failed.size(), 0);
  }
  void protocolMalformedAndInvalidPath() {
    Document d;
    PlannerClient client;
    client.executable = fixture();
    QSignalSpy failed(&client, &PlannerClient::failed);
    for (auto mode : {"malformed", "invalid_path", "crash"}) {
      d.data["fixture_mode"] = mode;
      int n = failed.size();
      QVERIFY(client.start(d));
      QTRY_COMPARE(failed.size(), n + 1);
      QVERIFY(!client.busy());
    }
  }
  void cancelTimeoutAndFailedLaunch() {
    Document d;
    d.data["fixture_mode"] = "delay";
    PlannerClient client;
    client.executable = fixture();
    QSignalSpy failed(&client, &PlannerClient::failed);
    QVERIFY(client.start(d));
    QTest::qWait(30);
    client.cancel();
    QTRY_COMPARE(failed.size(), 1);
    QVERIFY(failed[0][0].toString().contains("取消"));
    client.timeoutMs = 30;
    QVERIFY(client.start(d));
    QTRY_COMPARE(failed.size(), 2);
    QVERIFY(failed[1][0].toString().contains("超时"));
    client.executable = "/path/does/not/exist";
    QVERIFY(!client.start(d));
    QCOMPARE(failed.size(), 3);
  }
  void staleResultRejected() {
    MainWindow w(fixture());
    QSignalSpy done(&w, &MainWindow::planningFinished),
        failed(&w, &MainWindow::planningFailed);
    w.plan();
    w.doc.touch();
    w.changed();
    QTRY_COMPARE(failed.size(), 1);
    QCOMPARE(failed[0][0].toString(), QString("stale_result"));
    QCOMPARE(done.size(), 0);
    QVERIFY(w.result.isEmpty());
    w.doc.dirty = false;
  }
  void replayOverlappingFoldWindows() {
    QJsonObject r{{"traversals", QJsonArray{QJsonObject{{"prepare_s", 1.},
                                                        {"enter_s", 2.},
                                                        {"exit_s", 3.},
                                                        {"release_s", 4.},
                                                        {"passage_id", "a"},
                                                        {"mode", "folded"}},
                                            QJsonObject{{"prepare_s", 3.5},
                                                        {"enter_s", 5.},
                                                        {"exit_s", 6.},
                                                        {"release_s", 7.},
                                                        {"passage_id", "b"},
                                                        {"mode", "folded"}}}}};
    for (double s = 1; s < 6.999; s += .01)
      QVERIFY(replayState(r, s).folded);
    QVERIFY(!replayState(r, 7.).folded);
    QVERIFY(replayState(r, 2.5).locked);
    QVERIFY(!replayState(r, 4.5).locked);
  }
  void realCppExamples_data() {
    QTest::addColumn<QString>("name");
    QTest::addColumn<int>("count");
    QTest::newRow("two") << "two_passages" << 1;
    QTest::newRow("rotated") << "rotated_passage" << 1;
    QTest::newRow("chain") << "chain_passages" << 2;
  }
  void realCppExamples() {
    if (cli().isEmpty() || examples().isEmpty())
      QSKIP("Set TDT_PASSAGE_CLI and TDT_EXAMPLES_DIR for integration tests");
    QFETCH(QString, name);
    QFETCH(int, count);
    MainWindow w(cli());
    w.openProject(QDir(examples()).filePath(name + ".json"));
    QSignalSpy done(&w, &MainWindow::planningFinished);
    w.plan();
    QTRY_COMPARE_WITH_TIMEOUT(done.size(), 1, 60000);
    QVERIFY(w.result["success"].toBool());
    QCOMPARE(w.result["traversals"].toArray().size(), count);
    for (const auto &v : w.result["traversals"].toArray()) {
      auto t = v.toObject();
      double enter = t["enter_s"].toDouble(), exit = t["exit_s"].toDouble(),
             release = t["release_s"].toDouble();
      auto a = positionAt(w.result, enter), b = positionAt(w.result, exit),
           d = (b - a) / norm(b - a), normal = QPointF(-d.y(), d.x());
      auto path = w.result["path"].toArray(),
           arcs = w.result["arc_lengths"].toArray();
      for (int i = 0; i < path.size(); ++i)
        if (arcs[i].toDouble() > enter + 1e-5 &&
            arcs[i].toDouble() < release - 1e-5)
          QVERIFY(std::abs(dot(point(path[i]) - a, normal)) < 1e-5);
      w.seek((enter + exit) / 2);
      QVERIFY(replayState(w.result, w.currentProgress).folded);
    }
    w.seek(w.result["length"].toDouble());
    QVERIFY(!replayState(w.result, w.currentProgress).folded);
    QTemporaryDir dir;
    auto dest = dir.filePath("exported");
    QVERIFY(w.exportTo(dest));
    QVERIFY(QFileInfo(QDir(dest).filePath("debug/path.csv")).isFile());
    QCOMPARE(QDir(QDir(dest).filePath("debug"))
                 .entryList({"*.png"}, QDir::Files)
                 .size(),
             8);
    Document reload;
    reload.open(QDir(dest).filePath("project.json"));
    QCOMPARE(reload.image, w.doc.image);
    w.setBackground(1);
    QVERIFY(!w.canvas->background.isNull());
    w.doc.touch();
    w.changed();
    QVERIFY(w.result.isEmpty());
    QVERIFY(!w.exportTo(dir.filePath("stale")));
    w.doc.dirty = false;
  }
  void realCppDirectionDisableAndBlocker() {
    if (cli().isEmpty() || examples().isEmpty())
      QSKIP("Integration CLI unavailable");
    Document d;
    d.open(QDir(examples()).filePath("two_passages.json"));
    auto a = d.data["passages"].toArray();
    for (int i = 0; i < a.size(); ++i) {
      auto g = a[i].toObject();
      g["direction"] = "b_to_a";
      a[i] = g;
    }
    d.data["passages"] = a;
    PlannerClient c;
    c.executable = cli();
    QSignalSpy done(&c, &PlannerClient::completed);
    QVERIFY(c.start(d));
    QTRY_COMPARE_WITH_TIMEOUT(done.size(), 1, 60000);
    QVERIFY(!done[0][0].toJsonObject()["success"].toBool());
    for (int i = 0; i < a.size(); ++i) {
      auto g = a[i].toObject();
      g["direction"] = "both";
      g["enabled"] = (i == 0);
      a[i] = g;
    }
    d.data["passages"] = a;
    QVERIFY(c.start(d));
    QTRY_COMPARE_WITH_TIMEOUT(done.size(), 2, 60000);
    auto r = done[1][0].toJsonObject();
    QVERIFY(r["success"].toBool());
    QCOMPARE(r["traversals"].toArray()[0].toObject()["passage_id"].toString(),
             a[0].toObject()["id"].toString());
    d.data["blockers"] = QJsonArray{a[0].toObject()["region"]};
    QVERIFY(c.start(d));
    QTRY_COMPARE_WITH_TIMEOUT(done.size(), 3, 60000);
    QVERIFY(!done[2][0].toJsonObject()["success"].toBool());
  }
};
QTEST_MAIN(EditorTests)
#include "editor_tests.moc"
