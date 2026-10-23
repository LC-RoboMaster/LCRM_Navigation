#include "image_io.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <tdt_passage/semantic_planner.hpp>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr
        << "usage: passage_export_nav2_map PROJECT.json OUTPUT_DIRECTORY\n";
    return 2;
  }
  try {
    auto project = tdt_passage::loadProject(argv[1]);
    if (std::abs(project.map.info.yaw) > 1e-7)
      throw std::invalid_argument("initial Nav2 adapter requires map yaw=0; "
                                  "special region yaw is unrestricted");
    auto map = tdt_passage::prepareEnvironment(project);
    cv::Mat gray = map.free.clone();
    gray.setTo(205, map.unknown);
    cv::Mat image;
    cv::flip(gray, image, 0); // only file display uses top-down rows
    auto dir = std::filesystem::absolute(argv[2]);
    std::filesystem::create_directories(dir);
    if (!tdt_image::write((dir / "nav2_map.png").string(), image))
      throw std::runtime_error("cannot write map image");
    std::ofstream yaml(dir / "nav2_map.yaml");
    yaml.imbue(std::locale::classic());
    yaml << std::setprecision(17)
         << "image: nav2_map.png\nmode: trinary\nresolution: "
         << map.info.resolution << "\norigin: [" << map.info.origin.x() << ", "
         << map.info.origin.y()
         << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    yaml.close();
    if (!yaml)
      throw std::runtime_error("cannot write map YAML");
    std::cout << "Nav2 static map (not inflated): "
              << (dir / "nav2_map.yaml").string() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
