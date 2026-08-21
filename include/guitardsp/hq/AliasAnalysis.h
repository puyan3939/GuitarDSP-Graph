#pragma once

#include "FFT.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <span>
#include <vector>

namespace guitardsp::hq {

struct AliasMetrics {
    float totalEnergy = 0.0f;
    float harmonicEnergy = 0.0f;
    float residualEnergy = 0.0f;
    float residualDb = -160.0f;
};

inline AliasMetrics analyzeAliasResidual(std::span<const float> samples,
                                         double sampleRate,
                                         double fundamentalHz,
                                         int maxHarmonics = 32,
                                         int binRadius = 1) {
    AliasMetrics result;
    if (samples.empty() || sampleRate <= 0.0 || fundamentalHz <= 0.0) return result;

    const std::size_t n = nextPowerOfTwo(samples.size());
    std::vector<std::complex<float>> spectrum(n, {});
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const float phase = samples.size() > 1 ? static_cast<float>(i) / static_cast<float>(samples.size() - 1) : 0.0f;
        const float window = 0.5f - 0.5f * std::cos(2.0f * std::numbers::pi_v<float> * phase);
        spectrum[i] = {samples[i] * window, 0.0f};
    }
    Radix2FFT::transform(spectrum, false);

    const std::size_t half = n / 2;
    std::vector<bool> harmonicBin(half + 1, false);
    for (int h = 1; h <= maxHarmonics; ++h) {
        const double hz = fundamentalHz * static_cast<double>(h);
        if (hz >= sampleRate * 0.5) break;
        const auto center = static_cast<long long>(std::llround(hz * static_cast<double>(n) / sampleRate));
        for (int d = -binRadius; d <= binRadius; ++d) {
            const auto k = center + d;
            if (k >= 0 && static_cast<std::size_t>(k) <= half) harmonicBin[static_cast<std::size_t>(k)] = true;
        }
    }

    double total = 0.0;
    double harmonic = 0.0;
    for (std::size_t k = 1; k <= half; ++k) {
        const double e = static_cast<double>(std::norm(spectrum[k]));
        total += e;
        if (harmonicBin[k]) harmonic += e;
    }
    const double residual = std::max(0.0, total - harmonic);
    result.totalEnergy = static_cast<float>(total);
    result.harmonicEnergy = static_cast<float>(harmonic);
    result.residualEnergy = static_cast<float>(residual);
    result.residualDb = static_cast<float>(10.0 * std::log10((residual + 1.0e-30) / (total + 1.0e-30)));
    return result;
}

} // namespace guitardsp::hq
