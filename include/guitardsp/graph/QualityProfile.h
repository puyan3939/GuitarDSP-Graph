#pragma once

#include "AudioNode.h"
#include <algorithm>

namespace guitardsp::graph {

struct QualityProfile {
    ProcessingQuality mode = ProcessingQuality::high;
    int maximumOversampling = 16;

    [[nodiscard]] int suggestedOversampling(NodeCategory category) const noexcept {
        int factor = 1;
        switch (category) {
            case NodeCategory::drive: factor = 16; break;
            case NodeCategory::amp: factor = 16; break;
            case NodeCategory::pitch: factor = 8; break;
            case NodeCategory::dynamics: factor = 2; break;
            default: factor = 1; break;
        }
        switch (mode) {
            case ProcessingQuality::eco: factor = std::min(factor, 2); break;
            case ProcessingQuality::live: factor = std::min(factor, 8); break;
            case ProcessingQuality::high: factor = std::min(factor, 16); break;
            case ProcessingQuality::studio: factor = std::min(factor, 32); break;
        }
        return std::min(factor, std::max(1, maximumOversampling));
    }
};

} // namespace guitardsp::graph
