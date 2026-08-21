#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace guitardsp::app {

// Deterministic fallback cabinet impulse used only when no measured IR has been
// supplied by the host. It is deliberately documented as a reference/smoke-test
// response, not a measured speaker/cabinet capture.
inline std::vector<float> makeReferenceCabinetImpulse(double sampleRate, int length = 2048) {
    const double sr = std::max(8000.0, sampleRate);
    const int nSamples = std::clamp(length, 128, 16384);
    std::vector<float> impulse(static_cast<std::size_t>(nSamples), 0.0f);

    const double lowCut = 72.0;
    const double highCut = 6200.0;
    const double hpA = std::exp(-2.0 * std::numbers::pi * lowCut / sr);
    const double lpA = std::exp(-2.0 * std::numbers::pi * highCut / sr);

    double hpPrevIn = 0.0;
    double hpPrevOut = 0.0;
    double lp1 = 0.0;
    double lp2 = 0.0;

    for (int n = 0; n < nSamples; ++n) {
        const double t = static_cast<double>(n) / sr;
        double x = n == 0 ? 1.0 : 0.0;

        // Broad cabinet/body terms. These are intentionally modest so the fallback
        // stays useful for safe listening without pretending to be a captured cab.
        x += 0.18 * std::exp(-t * 48.0) * std::sin(2.0 * std::numbers::pi * 108.0 * t);
        x += 0.10 * std::exp(-t * 90.0) * std::sin(2.0 * std::numbers::pi * 680.0 * t);
        x += 0.07 * std::exp(-t * 125.0) * std::sin(2.0 * std::numbers::pi * 2350.0 * t);
        x -= 0.05 * std::exp(-t * 155.0) * std::sin(2.0 * std::numbers::pi * 4300.0 * t);

        const double hp = hpA * (hpPrevOut + x - hpPrevIn);
        hpPrevIn = x;
        hpPrevOut = hp;
        lp1 = (1.0 - lpA) * hp + lpA * lp1;
        lp2 = (1.0 - lpA) * lp1 + lpA * lp2;
        impulse[static_cast<std::size_t>(n)] = static_cast<float>(lp2);
    }

    const int reflection1 = static_cast<int>(std::round(0.00135 * sr));
    const int reflection2 = static_cast<int>(std::round(0.00310 * sr));
    for (int n = nSamples - 1; n >= 0; --n) {
        double y = impulse[static_cast<std::size_t>(n)];
        if (n >= reflection1)
            y += 0.17 * impulse[static_cast<std::size_t>(n - reflection1)];
        if (n >= reflection2)
            y -= 0.09 * impulse[static_cast<std::size_t>(n - reflection2)];
        impulse[static_cast<std::size_t>(n)] = static_cast<float>(y);
    }

    float peak = 0.0f;
    for (const float sample : impulse) peak = std::max(peak, std::abs(sample));
    if (peak > 1.0e-9f) {
        const float gain = 0.92f / peak;
        for (auto& sample : impulse) sample *= gain;
    }
    return impulse;
}

// Offline high-quality IR sample-rate conversion. This is intentionally a control-
// thread utility; it allocates and performs a windowed-sinc convolution so the audio
// callback never pays for IR rate conversion.
inline std::vector<float> resampleImpulseWindowedSinc(const std::vector<float>& input,
                                                       double sourceSampleRate,
                                                       double targetSampleRate,
                                                       int halfTaps = 32) {
    if (input.empty()) return {1.0f};
    const double sourceRate = std::max(1.0, sourceSampleRate);
    const double targetRate = std::max(1.0, targetSampleRate);
    if (std::abs(sourceRate - targetRate) < 1.0e-6) return input;

    halfTaps = std::clamp(halfTaps, 8, 128);
    const double ratio = targetRate / sourceRate;
    const std::size_t outputSize = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::llround(static_cast<double>(input.size()) * ratio)));
    std::vector<float> output(outputSize, 0.0f);

    const double cutoff = std::min(1.0, ratio);
    for (std::size_t outIndex = 0; outIndex < outputSize; ++outIndex) {
        const double sourcePosition = static_cast<double>(outIndex) / ratio;
        const auto center = static_cast<long long>(std::floor(sourcePosition));
        double sum = 0.0;
        double kernelSum = 0.0;

        for (int tap = -halfTaps; tap <= halfTaps; ++tap) {
            const auto sourceIndex = center + static_cast<long long>(tap);
            if (sourceIndex < 0 || sourceIndex >= static_cast<long long>(input.size())) continue;

            const double distance = sourcePosition - static_cast<double>(sourceIndex);
            const double x = cutoff * distance;
            const double sinc = std::abs(x) < 1.0e-12
                ? 1.0
                : std::sin(std::numbers::pi * x) / (std::numbers::pi * x);
            const double normalized = distance / static_cast<double>(halfTaps + 1);
            if (std::abs(normalized) >= 1.0) continue;
            const double window = 0.42
                + 0.50 * std::cos(std::numbers::pi * normalized)
                + 0.08 * std::cos(2.0 * std::numbers::pi * normalized);
            const double coefficient = cutoff * sinc * window;
            sum += static_cast<double>(input[static_cast<std::size_t>(sourceIndex)]) * coefficient;
            kernelSum += coefficient;
        }

        output[outIndex] = static_cast<float>(std::abs(kernelSum) > 1.0e-12 ? sum / kernelSum : 0.0);
    }
    return output;
}

} // namespace guitardsp::app
