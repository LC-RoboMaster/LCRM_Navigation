#include "lc_passage_nav2/execution.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
using namespace lc_passage_nav2;
void require(bool v, const char *text) {
  if (!v)
    throw std::runtime_error(text);
}
ExecutionPlan plan(bool reverse = false) {
  ExecutionPlan p;
  p.id = reverse ? "reverse" : "forward";
  for (int i = 0; i <= 100; ++i) {
    p.points.push_back({reverse ? 10. - i * .1 : i * .1, 0});
    p.arcs.push_back(i * .1);
  }
  p.traversals.push_back({"gate", reverse ? "b_to_a" : "a_to_b", "folded", 1, 2,
                          4, 5, .3, .3, .4});
  p.validate();
  return p;
}
ExecutionOutput move(Execution &e, double x, double t) {
  return e.tick({x, 0}, 0, 0, t);
}
int main() {
  try {
    for (bool reverse : {false, true}) {
      Execution e;
      e.bind(plan(reverse));
      auto pos = [&](double s) { return reverse ? 10 - s : s; };
      auto o = move(e, pos(0), 0);
      require(o.phase == "APPROACH" && !o.locked, "approach");
      require(std::isinf(o.speed_limit) && !o.stop && o.target_limit == 1.,
              "ordinary approach must not inherit passage speed");
      // 即将到达准备点且仍在运动时，也只跟踪准备点，不加专用制动限速。
      o = e.tick({pos(.95), 0}, 2., 0, .5);
      require(o.phase == "APPROACH" && std::isinf(o.speed_limit) && !o.stop,
              "near approach must use normal navigation limits");
      o = e.tick({pos(1), 0}, 1., 0, .8);
      require(o.phase == "APPROACH" && o.stop && !o.locked,
              "must stop at preparation before requesting deformation");
      o = move(e, pos(1), 1);
      require(o.phase == "WAIT_FOLD" && o.stop && o.folded && o.locked,
              "fold stop");
      auto request = o.request_id;
      e.feedback("old", "folded", true, false);
      require(move(e, pos(1), 1.5).stop, "stale ack must not release");
      e.feedback(request, "folded", true, false);
      require(move(e, pos(1), 2.).phase == "PASSING", "fold ack");
      require(std::abs(e.output().speed_limit - .3) < 1e-9,
              "passage speed starts only after fold acknowledgement and hold");
      auto replacement = plan(reverse);
      replacement.id = "replacement";
      require(!e.bind(replacement), "locked replacement rejected");
      for (int i = 2; i <= 5; ++i)
        o = move(e, pos(i), 2. + i);
      require(o.phase == "WAIT_RESTORE" && o.stop && o.folded, "restore stop");
      e.feedback(o.request_id, "normal", true, false);
      require(move(e, pos(5), 7.5).phase == "NORMAL" && !e.output().folded &&
                  !e.output().locked,
              "released");
      require(std::isinf(e.output().speed_limit),
              "restored ordinary route must release passage speed");
    }
    {
      Execution e;
      e.bind(plan());
      move(e, 1, 0);
      require(move(e, 1, 9).fault, "action timeout");
      require(e.output().folded, "timeout keeps shape");
    }
    {
      Execution e;
      e.bind(plan());
      auto o = move(e, 1, 1);
      e.feedback(o.request_id, "folded", true, true);
      require(e.output().fault, "hardware fault");
    }
    {
      Execution e;
      e.bind(plan());
      move(e, 1, 1);
      require(move(e, 1, 0).fault, "clock regression");
    }
    {
      Execution e;
      e.bind(plan());
      move(e, 0, 0);
      require(move(e, 8, 1).progress < 2, "projection must not jump to exit");
    }
    {
      Execution e;
      auto p = plan();
      p.traversals.push_back(
          {"next", "a_to_b", "folded", 4.5, 6, 7, 8, .2, .3, .4});
      e.bind(p);
      auto o = move(e, 1, 0);
      e.feedback(o.request_id, "folded", true, false);
      move(e, 1, 1);
      for (int i = 2; i <= 5; ++i)
        o = move(e, i, i);
      require(o.phase == "PASSING" && o.request_mode == "folded",
              "overlapping windows keep folded");
      require(std::abs(o.speed_limit - .2) < 1e-9,
              "continuous passage group uses the lowest passage speed");
    }
    {
      Execution e;
      auto p = plan();
      p.traversals.push_back(
          {"next", "a_to_b", "folded", 7, 8, 8.5, 9, .15, .3, .4});
      e.bind(p);
      auto o = move(e, 1, 0);
      e.feedback(o.request_id, "folded", true, false);
      move(e, 1, 1);
      for (int i = 2; i <= 5; ++i)
        o = move(e, i, i);
      require(o.phase == "WAIT_RESTORE", "first separate passage restores");
      e.feedback(o.request_id, "normal", true, false);
      o = move(e, 5, 6);
      require(o.phase == "NORMAL" && std::isinf(o.speed_limit),
              "release must not retain the first passage speed");
      o = e.tick({5.5, 0}, 2., 0, 7);
      require(o.phase == "APPROACH" && std::isinf(o.speed_limit) &&
                  o.target_limit == 7.,
              "ordinary road between passages must not inherit either speed");
      o = e.tick({6.5, 0}, 2., 0, 8);
      require(std::isinf(o.speed_limit), "next passage approach is unrestricted");
      o = move(e, 7, 9);
      require(o.phase == "WAIT_FOLD", "second passage waits for deformation");
      e.feedback(o.request_id, "folded", true, false);
      o = move(e, 7, 10);
      require(o.phase == "PASSING" && std::abs(o.speed_limit - .15) < 1e-9,
              "second crossing uses its own passage speed");
    }
    {
      Execution e;
      auto p = plan();
      p.traversals.clear();
      e.bind(p);
      const auto o = e.tick({0, 0}, 2., 0, 0);
      require(o.phase == "NORMAL" && !o.stop && std::isinf(o.speed_limit),
              "ordinary routes without passages remain unrestricted");
    }
    {
      auto p = plan();
      p.arcs[2] = 50;
      bool threw = false;
      try {
        p.validate();
      } catch (...) {
        threw = true;
      }
      require(threw, "invalid arc rejected");
    }
    PassageFile f;
    PassageConfig g;
    g.center = {3, 4};
    g.yaw = .8;
    f.passages.push_back(g);
    require(isBeam(f, {3, 4}, .5), "beam filtering");
    require(!isBeam(f, {3, 4}, .15), "low obstacle preserved");
    require(!isBeam(f, {0, 0}, .5), "outside preserved");
    require(loadPassages("").passages.empty(), "empty annotation mapping mode");
    std::cout << "execution: forward/reverse, stop/ack/hold, timeout, fault, "
                 "lock, overlap, crossing-only speed, separate passages, "
                 "projection, schema, beam height passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
