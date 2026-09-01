#pragma once
#include "guitardsp/graph/AudioNode.h"
#include <algorithm>

namespace guitardsp::hq {

struct QualitySettings {
    int oversamplingFactor = 1;
    int resamplerTaps = 31;
    float stopbandDb = 90.0f;
};

inline QualitySettings qualityFor(graph::ProcessingQuality quality, graph::NodeCategory category) noexcept {
    using Q = graph::ProcessingQuality;
    using C = graph::NodeCategory;
    const bool nonlinear = category == C::drive || category == C::amp;
    const bool pitch = category == C::pitch;

    switch (quality) {
        case Q::eco:
            return { nonlinear ? 2 : 1, 23, 72.0f };
        case Q::live:
            return { nonlinear ? 4 : (pitch ? 2 : 1), 31, 90.0f };
        case Q::high:
            return { nonlinear ? 8 : (pitch ? 4 : 1), 47, 110.0f };
        case Q::studio:
            return { nonlinear ? 16 : (pitch ? 8 : 1), 63, 125.0f };
    }
    return {};
}

inline int clampOversamplingFactor(int factor) noexcept {
    if (factor <= 1) return 1;
    if (factor <= 2) return 2;
    if (factor <= 4) return 4;
    if (factor <= 8) return 8;
    return 16;
}

} // namespace guitardsp::hq
