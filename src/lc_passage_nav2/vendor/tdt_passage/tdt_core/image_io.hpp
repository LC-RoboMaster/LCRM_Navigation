#pragma once
#include <opencv2/core.hpp>
#include <string>
// Small PNG/JPEG/PGM reader and PNG writer. Avoids pulling camera/video/GUI
// backends into a standalone planning library. Implemented with bundled stb.
namespace tdt_image {
cv::Mat read(const std::string& filename,int flags=0);
bool write(const std::string& filename,const cv::Mat& image);
}
