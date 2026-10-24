#pragma once
// No ROS headers here: snapshot conversion is tested independently of Nav2.
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <tdt_passage/semantic_planner.hpp>
#include <vector>

namespace lc_passage_nav2
{
    struct CostmapSnapshot
    {
        unsigned int width = 0, height = 0;
        double resolution = 0, origin_x = 0, origin_y = 0;
        std::vector<uint8_t> cells; // Nav2 row order: row zero is the bottom row
    };

    inline cv::Mat runtimeMask(const tdt_nav::GridMap& map,
                               const CostmapSnapshot& snapshot,
                               bool unknown_is_obstacle = true)
    {
        map.validate();
        if (snapshot.width != static_cast<unsigned int>(map.free.cols) ||
            snapshot.height != static_cast<unsigned int>(map.free.rows) ||
            snapshot.cells.size() != size_t(snapshot.width) * snapshot.height)
            throw std::invalid_argument(
                "Nav2/project map dimensions differ; load the exported nav2_map.yaml");
        auto equal = [](double a, double b)
        {
            return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) < 1e-7;
        };
        if (!equal(map.info.resolution, snapshot.resolution) ||
            !equal(map.info.origin.x(), snapshot.origin_x) ||
            !equal(map.info.origin.y(), snapshot.origin_y) ||
            !equal(map.info.yaw, 0.))
            throw std::invalid_argument("Nav2/project resolution or origin differs, or "
                "project map yaw is nonzero");
        cv::Mat mask(map.free.size(), CV_8UC1, cv::Scalar(0));
        for (unsigned int y = 0; y < snapshot.height; ++y)
            for (unsigned int x = 0; x < snapshot.width; ++x)
            {
                const auto cost = snapshot.cells[size_t(y) * snapshot.width + x];
                // 254 is physical lethal occupancy. 253 and lower inflation costs are NOT
                // promoted to obstacles: the core applies its own robot inflation once.
                if (cost == 254 || (cost == 255 && unknown_is_obstacle))
                    mask.at<uint8_t>(y, x) = 255;
            }
        return mask;
    }
} // namespace lc_passage_nav2
