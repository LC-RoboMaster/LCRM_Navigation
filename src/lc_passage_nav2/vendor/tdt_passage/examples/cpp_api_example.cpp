#include <fstream>
#include <iostream>
#include <tdt_passage/semantic_planner.hpp>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: passage_api_example PROJECT.json\n";
    return 2;
  }
  auto project = tdt_passage::loadProject(argv[1]);
  std::ifstream file(argv[1]);
  tdt_passage::Json j;
  file >> j;
  auto point = [&](const std::string &key) {
    auto p = j.at("test").at(key);
    return tdt_passage::Vec2(p[0].get<double>(), p[1].get<double>());
  };
  auto result =
      tdt_passage::Planner().plan(project, point("start"), point("goal"));
  if (!result.success) {
    std::cerr << result.status << ": " << result.message << '\n';
    return 3;
  }
  tdt_passage::PassageTracker tracker;
  tracker.bind(result);
  for (double s = 0; s < result.length + .1; s += .1) {
    auto [state, events] = tracker.advance(s);
    for (const auto &event : events) {
      std::cout << "s=" << event.s << " " << event.passage_id << " "
                << event.type << "  fold_request=" << state.fold_request
                << "  mode=" << state.mode << '\n';
    }
  }
}
