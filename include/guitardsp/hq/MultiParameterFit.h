#pragma once

#include "OfflineModelEvaluator.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace guitardsp::hq {

struct FitParameterRange {
    std::size_t parameterIndex = 0;
    float minimum = 0.0f;
    float maximum = 1.0f;
    int steps = 5;
};

struct MultiParameterFitResult {
    std::vector<float> values;
    float score = std::numeric_limits<float>::infinity();
    FitMetrics metrics{};
    int evaluations = 0;
};

// Deterministic coordinate-descent grid fitter for offline calibration.
// Each pass sweeps every requested parameter while holding the remaining values
// at their current best. The implementation is intentionally simple and bounded
// so it is suitable for CI regression tests and first-pass hardware fitting.
template <typename Node>
MultiParameterFitResult fitParametersCoordinateGrid(Node& node,
                                                    std::span<const FitParameterRange> ranges,
                                                    int passes,
                                                    graph::PrepareSpec spec,
                                                    std::span<const FrequencyReferencePoint> reference,
                                                    float amplitude = 0.05f,
                                                    int blockSize = 256,
                                                    int settleBlocks = 5,
                                                    int captureBlocks = 4) {
    MultiParameterFitResult result;
    result.values.resize(ranges.size(), 0.0f);
    if (ranges.empty()) return result;

    for (std::size_t i = 0; i < ranges.size(); ++i)
        result.values[i] = node.parameterValue(ranges[i].parameterIndex);

    const auto evaluate = [&]() {
        ++result.evaluations;
        std::vector<float> frequencies(reference.size());
        for (std::size_t i = 0; i < reference.size(); ++i)
            frequencies[i] = reference[i].frequencyHz;
        const auto model = measureNodeFrequencyResponse(node, spec, frequencies,
                                                        amplitude, blockSize,
                                                        settleBlocks, captureBlocks);
        return compareFrequencyResponse(reference, model);
    };

    result.metrics = evaluate();
    result.score = result.metrics.weightedRmsError;

    passes = std::max(1, passes);
    for (int pass = 0; pass < passes; ++pass) {
        bool improved = false;
        for (std::size_t r = 0; r < ranges.size(); ++r) {
            const auto& range = ranges[r];
            const float low = std::min(range.minimum, range.maximum);
            const float high = std::max(range.minimum, range.maximum);
            const int steps = std::max(2, range.steps);

            float bestValue = result.values[r];
            float bestScore = result.score;
            FitMetrics bestMetrics = result.metrics;

            for (int step = 0; step < steps; ++step) {
                const float t = static_cast<float>(step) / static_cast<float>(steps - 1);
                const float candidate = low + t * (high - low);
                if (!node.setParameterValue(range.parameterIndex, candidate)) continue;
                const auto metrics = evaluate();
                const float score = metrics.weightedRmsError;
                if (score < bestScore) {
                    bestScore = score;
                    bestValue = candidate;
                    bestMetrics = metrics;
                }
            }

            node.setParameterValue(range.parameterIndex, bestValue);
            if (bestScore + 1.0e-7f < result.score) improved = true;
            result.values[r] = bestValue;
            result.score = bestScore;
            result.metrics = bestMetrics;
        }
        if (!improved) break;
    }

    for (std::size_t i = 0; i < ranges.size(); ++i)
        node.setParameterValue(ranges[i].parameterIndex, result.values[i]);
    return result;
}

} // namespace guitardsp::hq
