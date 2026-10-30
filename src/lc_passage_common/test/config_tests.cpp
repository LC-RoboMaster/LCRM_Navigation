#include "lc_passage_common/passage_config.hpp"
#include <iostream>
using namespace lc_passage_common;
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  try {
    require(loadPassages("").passages.empty(), "empty annotation mapping mode");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
