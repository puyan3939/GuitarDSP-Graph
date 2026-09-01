#pragma once

#include "MeasuredFit.h"
#include "Measurement.h"
#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/graph/AudioNode.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

namespace guitardsp::hq {

struct ParameterFitResult {
    float value = 0.0f;
    float score = std::numeric_limits<float>::infinity();
    FitMetrics metrics{};
};

// Offline-only frequency sweep. The node is reset for every frequency and is
// driven through settle blocks before capture. Input and output use the same
// single-bin estimator, so the Hann-window normalization cancels in the ratio.
template <typename Node>
std::vector<float> measureNodeFrequencyResponse(Node& node,
                                                graph::PrepareSpec spec,
                                                std::span<const float> frequenciesHz,
                                                float amplitude = 0.05f,
                                                int blockSize = 256,
                                                int settleBlocks = 5,
                                                int captureBlocks = 4) {
    blockSize = std::max(32, blockSize);
    settleBlocks = std::max(1, settleBlocks);
    captureBlocks = std::max(1, captureBlocks);
    spec.channels = 1;
    spec.maximumBlockSize = blockSize;
    node.prepare(spec);

    graph::AudioBuffer input(1, blockSize), output(1, blockSize);
    std::vector<float> capturedInput(static_cast<std::size_t>(blockSize * captureBlocks));
    std::vector<float> capturedOutput(static_cast<std::size_t>(blockSize * captureBlocks));
    std::vector<float> result;
    result.reserve(frequenciesHz.size());

    for (float frequency : frequenciesHz) {
        const double hz = std::clamp<double>(frequency, 1.0, 0.45 * spec.sampleRate);
        const double phaseIncrement = 2.0 * std::numbers::pi * hz / spec.sampleRate;
        double phase = 0.0;
        node.reset();

        const int totalBlocks = settleBlocks + captureBlocks;
        int captureOffset = 0;
        for (int block = 0; block < totalBlocks; ++block) {
            for (int i = 0; i < blockSize; ++i) {
                input.channel(0)[i] = amplitude * static_cast<float>(std::sin(phase));
                phase += phaseIncrement;
                if (phase >= 2.0 * std::numbers::pi) phase -= 2.0 * std::numbers::pi;
            }
            node.process(input, output, blockSize);

            if (block >= settleBlocks) {
                std::copy(input.channel(0), input.channel(0) + blockSize,
                          capturedInput.begin() + captureOffset);
                std::copy(output.channel(0), output.channel(0) + blockSize,
                          capturedOutput.begin() + captureOffset);
                captureOffset += blockSize;
            }
        }

        const float inputMagnitude = singleBinMagnitude(capturedInput, spec.sampleRate, hz);
        const float outputMagnitude = singleBinMagnitude(capturedOutput, spec.sampleRate, hz);
        const float ratio = outputMagnitude / std::max(1.0e-12f, inputMagnitude);
        result.push_back(20.0f * std::log10(std::max(1.0e-9f, ratio)));
    }
    return result;
}

template <typename Node>
FitMetrics fitNodeFrequencyResponse(Node& node,
                                    graph::PrepareSpec spec,
                                    std::span<const FrequencyReferencePoint> reference,
                                    float amplitude = 0.05f,
                                    int blockSize = 256,
                                    int settleBlocks = 5,
                                    int captureBlocks = 4) {
    std::vector<float> frequencies(reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i) frequencies[i] = reference[i].frequencyHz;
    const auto model = measureNodeFrequencyResponse(node, spec, frequencies, amplitude,
                                                    blockSize, settleBlocks, captureBlocks);
    return compareFrequencyResponse(reference, model);
}

// Small deterministic 1-D grid search intended for CI and initial fitting. More
// sophisticated optimizers can be layered on later without changing fit metrics.
template <typename Node>
ParameterFitResult fitParameterGrid1D(Node& node,
                                      std::size_t parameterIndex,
                                      float minimum,
                                      float maximum,
                                      int steps,
                                      graph::PrepareSpec spec,
                                      std::span<const FrequencyReferencePoint> reference,
                                      float amplitude = 0.05f,
                                      int blockSize = 256,
                                      int settleBlocks = 5,
                                      int captureBlocks = 4) {
    ParameterFitResult best;
    steps = std::max(2, steps);
    const float low = std::min(minimum, maximum);
    const float high = std::max(minimum, maximum);

    for (int i = 0; i < steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps - 1);
        const float value = low + t * (high - low);
        if (!node.setParameterValue(parameterIndex, value)) continue;
        const auto metrics = fitNodeFrequencyResponse(node, spec, reference, amplitude,
                                                      blockSize, settleBlocks, captureBlocks);
        const float score = metrics.weightedRmsError;
        if (score < best.score) {
            best.value = value;
            best.score = score;
            best.metrics = metrics;
        }
    }
    node.setParameterValue(parameterIndex, best.value);
    return best;
}

} // namespace guitardsp::hq
