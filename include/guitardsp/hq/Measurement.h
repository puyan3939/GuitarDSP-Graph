#pragma once
#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <vector>

namespace guitardsp::hq {

struct HarmonicMetrics {
    float fundamental = 0.0f;
    float thd = 0.0f;
    float thdDb = -160.0f;
    float highBandEnergy = 0.0f;
    // Magnitude of each harmonic actually measured (bounded by Nyquist),
    // in ascending order starting at the 2nd harmonic -- i.e.
    // harmonicMagnitudes[0] is the 2nd harmonic, [1] the 3rd, and so on.
    // Divide by `fundamental` for the per-harmonic ratio used by THD.
    std::vector<float> harmonicMagnitudes;
};

inline float singleBinMagnitude(std::span<const float> samples, double sampleRate, double frequency) noexcept {
    if (samples.empty() || sampleRate <= 0.0) return 0.0f;
    double re = 0.0, im = 0.0;
    const double w = 2.0 * std::numbers::pi * frequency / sampleRate;
    for (std::size_t n = 0; n < samples.size(); ++n) {
        const double window = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(n) / static_cast<double>(samples.size() - 1));
        const double phase = w * static_cast<double>(n);
        const double x = static_cast<double>(samples[n]) * window;
        re += x * std::cos(phase);
        im -= x * std::sin(phase);
    }
    return static_cast<float>(2.0 * std::sqrt(re * re + im * im) / static_cast<double>(samples.size()));
}

inline HarmonicMetrics analyzeHarmonics(std::span<const float> samples, double sampleRate, double fundamentalHz, int harmonicCount = 10) noexcept {
    HarmonicMetrics m;
    if (samples.size() < 8 || sampleRate <= 0.0 || fundamentalHz <= 0.0) return m;
    m.fundamental = singleBinMagnitude(samples, sampleRate, fundamentalHz);
    double harmonicPower = 0.0;
    for (int h = 2; h <= harmonicCount; ++h) {
        const double f = fundamentalHz * static_cast<double>(h);
        if (f >= 0.5 * sampleRate) break;
        const double mag = singleBinMagnitude(samples, sampleRate, f);
        harmonicPower += mag * mag;
        m.harmonicMagnitudes.push_back(static_cast<float>(mag));
    }
    m.thd = m.fundamental > 1.0e-12f ? static_cast<float>(std::sqrt(harmonicPower) / m.fundamental) : 0.0f;
    m.thdDb = 20.0f * std::log10(std::max(m.thd, 1.0e-8f));

    // Deliberately simple offline alias/noise proxy: energy in the upper 15% of Nyquist.
    // It is not a substitute for a full FFT report, but it gives CI a regression signal.
    const int probes = 32;
    double upper = 0.0;
    for (int i = 0; i < probes; ++i) {
        const double t = probes > 1 ? static_cast<double>(i) / static_cast<double>(probes - 1) : 0.0;
        const double f = sampleRate * (0.425 + 0.07 * t);
        const double mag = singleBinMagnitude(samples, sampleRate, f);
        upper += mag * mag;
    }
    m.highBandEnergy = static_cast<float>(upper / probes);
    return m;
}

} // namespace guitardsp::hq
