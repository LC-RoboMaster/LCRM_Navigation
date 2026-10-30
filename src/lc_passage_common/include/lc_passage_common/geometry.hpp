#pragma once
#include <cmath>

namespace lc_passage_common
{
    struct Point
    {
        double x = 0, y = 0;
    };

    inline double distance(Point a, Point b)
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

}
