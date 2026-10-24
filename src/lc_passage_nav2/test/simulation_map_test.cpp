// Reproduce the user's editor/Nav2 discrepancy on the committed real raster.
#include <tdt_passage/semantic_planner.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc,char **argv) {
  try {
    if (argc!=2) throw std::runtime_error("expected editor project path");
    auto p=tdt_passage::loadProject(argv[1]);
    if ((p.map.info.origin-tdt_nav::Vec2(-3.58,-9.44)).norm()>1e-8 ||
        std::abs(p.planner.robot_radius-.2)>1e-8)
      throw std::runtime_error("simulation project must use YAML coordinates and real test radius");
    // This is the Nav2 input after PassageMapNode, not the original editor image.
    auto runtime=p;
    runtime.map=tdt_passage::prepareEnvironment(p);
    for (auto &g : runtime.passages) { g.clear_static=false; g.clear_unknown=false; }
    for (size_t i=0;i<2;++i) {
      const auto &g=p.passages.at(i);
      const auto d=(g.b-g.a).normalized();
      const tdt_nav::Vec2 start=g.a-d*.3,goal=g.b+d*(g.release_distance+.3);
      for (const auto *input : {&p,&runtime}) {
        auto result=tdt_passage::Planner().plan(*input,start,goal);
        bool selected=false;
        for (const auto &t : result.traversals) if (t.passage_id==g.id) selected=true;
        if (!result.success || !selected)
          throw std::runtime_error(g.id+": "+result.status+" "+result.message);
      }
      std::cout<<"PASS "<<g.id<<" editor and live-map adapter select the same gate\n";
    }
    return 0;
  } catch (const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; }
}
