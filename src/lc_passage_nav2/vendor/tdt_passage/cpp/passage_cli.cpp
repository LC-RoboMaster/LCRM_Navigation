#include <fstream>
#include <iostream>
#include <tdt_passage/semantic_planner.hpp>

int main(int argc, char **argv) {
  try {
    if (argc < 2) {
      std::cout << "passage_cli PROJECT.json [--start x y] [--goal x y] [--out "
                   "result.json] [--debug directory]\n";
      return 2;
    }
    std::string projectFile = argv[1], output = "result.json", debug;
    std::optional<tdt_passage::Vec2> start, goal;
    for (int i = 2; i < argc; ++i) {
      std::string a = argv[i];
      if ((a == "--start" || a == "--goal") && i + 2 < argc) {
        double x = std::stod(argv[++i]), y = std::stod(argv[++i]);
        if (a == "--start")
          start = tdt_passage::Vec2(x, y);
        else
          goal = tdt_passage::Vec2(x, y);
      } else if (a == "--out" && i + 1 < argc)
        output = argv[++i];
      else if (a == "--debug" && i + 1 < argc)
        debug = argv[++i];
      else
        throw std::invalid_argument("unknown/incomplete argument: " + a);
    }
    auto project = tdt_passage::loadProject(projectFile);
    if (!start || !goal) {
      std::ifstream f(projectFile);
      tdt_passage::Json j;
      f >> j;
      auto pos = [&](const std::string &name) {
        auto v = j.at("test").at(name);
        return tdt_passage::Vec2(v[0].get<double>(), v[1].get<double>());
      };
      if (!start)
        start = pos("start");
      if (!goal)
        goal = pos("goal");
    }
    auto result = tdt_passage::Planner().plan(project, *start, *goal);
    tdt_passage::saveResult(result, output);
    if (!debug.empty())
      tdt_passage::saveDebug(result, project, debug);
    std::cout << result.status << ": " << result.message
              << "\nlength=" << result.length
              << " m  graph_cost=" << result.cost_seconds
              << " s-equivalent  elapsed=" << result.elapsed_ms
              << " ms  passages=" << result.traversals.size() << '\n';
    for (const auto &w : result.warnings)
      std::cout << "warning: " << w << '\n';
    return result.success ? 0 : 3;
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
