#include "image_io.hpp"
#include <opencv2/imgproc.hpp>
#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
namespace tdt_image {
cv::Mat read(const std::string &file, int flags) {
  int w, h, c;
  int desired = flags == 0 ? 1 : 0;
  auto *data = stbi_load(file.c_str(), &w, &h, &c, desired);
  if (!data)
    return {};
  int channels = desired ? desired : c;
  cv::Mat image(h, w, CV_MAKETYPE(CV_8U, channels), data);
  image = image.clone();
  stbi_image_free(data);
  if (channels == 3)
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  else if (channels == 4)
    cv::cvtColor(image, image, cv::COLOR_RGBA2BGRA);
  return image;
}
bool write(const std::string &file, const cv::Mat &src) {
  if (src.empty() || src.depth() != CV_8U)
    return false;
  cv::Mat image = src.clone();
  if (image.channels() == 3)
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  else if (image.channels() == 4)
    cv::cvtColor(image, image, cv::COLOR_BGRA2RGBA);
  return stbi_write_png(file.c_str(), image.cols, image.rows, image.channels(),
                        image.data, int(image.step)) != 0;
}
} // namespace tdt_image
