#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace guitardsp::hq {

struct FrequencyReferencePoint {
    float frequencyHz = 0.0f;
    float magnitudeDb = 0.0f;
    float weight = 1.0f;
};

struct TransferReferencePoint {
    float input = 0.0f;
    float output = 0.0f;
    float weight = 1.0f;
};

struct FitMetrics {
    float rmsError = 0.0f;
    float maxAbsError = 0.0f;
    float weightedRmsError = 0.0f;
    std::size_t points = 0;
};

inline FitMetrics compareScalarSeries(std::span<const float> measured,
                                      std::span<const float> model,
                                      std::span<const float> weights = {}) noexcept {
    FitMetrics result;
    const std::size_t count = std::min(measured.size(), model.size());
    if (count == 0) return result;

    double squared = 0.0;
    double weightedSquared = 0.0;
    double totalWeight = 0.0;
    float maxAbs = 0.0f;

    for (std::size_t i = 0; i < count; ++i) {
        const float error = model[i] - measured[i];
        const float absError = std::abs(error);
        const float weight = i < weights.size() ? std::max(0.0f, weights[i]) : 1.0f;
        squared += static_cast<double>(error) * static_cast<double>(error);
        weightedSquared += static_cast<double>(weight) * static_cast<double>(error) * static_cast<double>(error);
        totalWeight += static_cast<double>(weight);
        maxAbs = std::max(maxAbs, absError);
    }

    result.rmsError = static_cast<float>(std::sqrt(squared / static_cast<double>(count)));
    result.weightedRmsError = totalWeight > 0.0
        ? static_cast<float>(std::sqrt(weightedSquared / totalWeight))
        : result.rmsError;
    result.maxAbsError = maxAbs;
    result.points = count;
    return result;
}

inline FitMetrics compareFrequencyResponse(std::span<const FrequencyReferencePoint> reference,
                                           std::span<const float> modelMagnitudeDb) {
    const std::size_t count = std::min(reference.size(), modelMagnitudeDb.size());
    std::vector<float> measured(count), model(count), weights(count);
    for (std::size_t i = 0; i < count; ++i) {
        measured[i] = reference[i].magnitudeDb;
        model[i] = modelMagnitudeDb[i];
        weights[i] = reference[i].weight;
    }
    return compareScalarSeries(measured, model, weights);
}

inline FitMetrics compareTransferCurve(std::span<const TransferReferencePoint> reference,
                                       std::span<const float> modelOutput) {
    const std::size_t count = std::min(reference.size(), modelOutput.size());
    std::vector<float> measured(count), model(count), weights(count);
    for (std::size_t i = 0; i < count; ++i) {
        measured[i] = reference[i].output;
        model[i] = modelOutput[i];
        weights[i] = reference[i].weight;
    }
    return compareScalarSeries(measured, model, weights);
}

// Aggregate score for CI/parameter searches. Frequency error is expressed in dB,
// transfer error in normalized signal units. The scale factor makes the transfer
// term roughly comparable to a few dB of response error for typical pedal data.
inline float aggregateFitScore(const FitMetrics& frequency,
                               const FitMetrics& transfer,
                               float transferScale = 12.0f) noexcept {
    return frequency.weightedRmsError + transferScale * transfer.weightedRmsError;
}

} // namespace guitardsp::hq
