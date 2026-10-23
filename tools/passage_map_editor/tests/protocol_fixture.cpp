// Test-only process fixture. Never used for application planning.
#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>
int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  auto args = app.arguments();
  if (args.size() < 2)
    return 2;
  QFile input(args[1]);
  if (!input.open(QIODevice::ReadOnly))
    return 2;
  auto project = QJsonDocument::fromJson(input.readAll()).object();
  auto mode = project["fixture_mode"].toString();
  if (mode == "delay")
    QThread::msleep(5000);
  if (mode == "crash")
    return 7;
  int pos = args.indexOf("--out");
  if (pos < 0 || pos + 1 >= args.size())
    return 2;
  QFile out(args[pos + 1]);
  if (!out.open(QIODevice::WriteOnly))
    return 2;
  if (mode == "malformed") {
    out.write("not json");
    return 0;
  }
  if (mode == "invalid_path") {
    out.write(
        R"({"success":true,"status":"success","path":[[0,0]],"arc_lengths":[]})");
    return 0;
  }
  if (mode == "no_route") {
    out.write(
        R"({"success":false,"status":"no_route","message":"fixture blocked"})");
    return 3;
  }
  out.write(
      R"({"success":true,"status":"success","length":1,"path":[[0,0],[1,0]],"arc_lengths":[0,1],"traversals":[],"events":[]})");
  return 0;
}
