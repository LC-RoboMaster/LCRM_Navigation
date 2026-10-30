#include "lc_passage_map/beam_filter.hpp"
#include <iostream>
using namespace lc_passage_common;
using lc_passage_map::isBeam;
void require(bool value, const char *message) { if(!value) throw std::runtime_error(message); }
int main() {
  try {
    PassageFile f;
    PassageConfig g;
    g.center = {3, 4};
    g.yaw = .8;
    f.passages.push_back(g);
    require(isBeam(f, {3, 4}, .5), "beam filtering");
    require(!isBeam(f, {3, 4}, .15), "low obstacle preserved");
    require(!isBeam(f, {0, 0}, .5), "outside preserved");
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; }
}
