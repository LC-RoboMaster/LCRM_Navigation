#pragma once
#include "lc_passage_common/passage_config.hpp"

namespace lc_passage_map
{
    using lc_passage_common::Point;
    using lc_passage_common::PassageFile;
    // 仅对标注的梁高区间进行实时点云过滤，地面和低矮物体始终保留。
    inline bool isBeam(const PassageFile& f, Point p, double relative_height)
    {
        for (const auto& gate : f.passages)
            if (gate.enabled && gate.clear_static && gate.contains(p) &&
                relative_height >= gate.beam_min && relative_height <= gate.beam_max)
                return true;
        return false;
    }
}
