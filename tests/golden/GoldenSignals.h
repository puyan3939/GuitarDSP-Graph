#pragma once

// Deterministic input-signal generation for the golden reference suite (see
// docs/GOLDEN_REFERENCE.md). Every signal is generated from a closed-form
// expression -- never read from a committed audio file -- so the same
// samples are reproducible from source alone. Each signal is computed in
// float64 and rounded to float32 on the way out, matching the "generate in
// float64, store as float32" rule in the issue that introduced this file.
//
// Shared by tools/golden_gen (which writes the committed golden files) and
// tests/GoldenReferenceTests.cpp (which regenerates the same signals at test
// time to feed the circuits under test) so the two can never drift apart.

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace guitardsp::golden {

inline constexpr double kSampleRateHz = 48000.0;
inline constexpr double kPi = 3.14159265358979323846;

inline std::size_t secondsToSamples(double seconds) {
    return static_cast<std::size_t>(seconds * kSampleRateHz + 0.5);
}

// 1 kHz sine burst at the given dBFS level. Used for both the -20 dBFS
// "clean" reference and the -3 dBFS "clipping region" reference.
inline std::vector<float> generateSine1kHz(double amplitudeDbfs, double durationSeconds) {
    const std::size_t n = secondsToSamples(durationSeconds);
    const double amplitude = std::pow(10.0, amplitudeDbfs / 20.0);
    std::vector<float> out(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / kSampleRateHz;
        out[i] = static_cast<float>(amplitude * std::sin(2.0 * kPi * 1000.0 * t));
    }
    return out;
}

// 20 Hz -> 20 kHz exponential ("log") sweep at the given dBFS level.
inline std::vector<float> generateLogSweep20HzTo20kHz(double amplitudeDbfs, double durationSeconds) {
    const std::size_t n = secondsToSamples(durationSeconds);
    const double amplitude = std::pow(10.0, amplitudeDbfs / 20.0);
    const double startHz = 20.0;
    const double endHz = 20000.0;
    const double k = std::log(endHz / startHz) / durationSeconds;
    std::vector<float> out(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / kSampleRateHz;
        const double phase = 2.0 * kPi * startHz * (std::expm1(k * t)) / k;
        out[i] = static_cast<float>(amplitude * std::sin(phase));
    }
    return out;
}

// Unit impulse followed by silence, long enough to observe each circuit's
// impulse response settle.
inline std::vector<float> generateUnitImpulse(double durationSeconds) {
    std::vector<float> out(secondsToSamples(durationSeconds), 0.0f);
    if (!out.empty()) out[0] = 1.0f;
    return out;
}

inline std::vector<float> generateSilence(double durationSeconds) {
    return std::vector<float>(secondsToSamples(durationSeconds), 0.0f);
}

// One entry of the fixed 5-signal golden input matrix (see
// docs/GOLDEN_REFERENCE.md section 2).
struct GoldenSignal {
    const char* name;
    double durationSeconds;
};

inline const std::vector<GoldenSignal>& goldenSignalList() {
    static const std::vector<GoldenSignal> signals = {
        {"sine20", 0.5},    // 1 kHz / -20 dBFS / 0.5 s
        {"sine3", 0.5},     // 1 kHz / -3 dBFS / 0.5 s (clipping region)
        {"sweep", 2.0},     // 20 Hz -> 20 kHz log sweep / -12 dBFS / 2 s
        {"impulse", 0.5},   // unit impulse
        {"silence", 0.5},   // silence (DS-1 noise-floor fixed point)
    };
    return signals;
}

inline std::vector<float> generateGoldenSignal(const std::string& name) {
    if (name == "sine20") return generateSine1kHz(-20.0, 0.5);
    if (name == "sine3") return generateSine1kHz(-3.0, 0.5);
    if (name == "sweep") return generateLogSweep20HzTo20kHz(-12.0, 2.0);
    if (name == "impulse") return generateUnitImpulse(0.5);
    if (name == "silence") return generateSilence(0.5);
    return {};
}

} // namespace guitardsp::golden
