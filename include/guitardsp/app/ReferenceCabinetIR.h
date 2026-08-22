#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <vector>

namespace guitardsp::app {

struct CabinetImpulseAnalysis {
    float samplePeak = 0.0f;
    double midbandGainDb = -120.0;
    double maximumGainDb = -120.0;
    double maximumGainFrequencyHz = 0.0;
    bool allFinite = true;
};

namespace detail {

inline double cabinetImpulseMagnitude(const std::vector<float>& impulse,
                                     double sampleRate,
                                     double frequency) noexcept {
    const double phase = 2.0 * std::numbers::pi * frequency / sampleRate;
    const double stepReal = std::cos(phase);
    const double stepImaginary = std::sin(phase);
    double oscillatorReal = 1.0;
    double oscillatorImaginary = 0.0;
    double real = 0.0;
    double imaginary = 0.0;
    for (const float value : impulse) {
        const double sample = std::isfinite(value) ? static_cast<double>(value) : 0.0;
        real += sample * oscillatorReal;
        imaginary -= sample * oscillatorImaginary;
        const double nextReal = oscillatorReal * stepReal
                              - oscillatorImaginary * stepImaginary;
        oscillatorImaginary = oscillatorImaginary * stepReal
                            + oscillatorReal * stepImaginary;
        oscillatorReal = nextReal;
    }
    return std::sqrt(real * real + imaginary * imaginary);
}

struct CabinetDesignBiquad {
    enum class Shape { highpass, lowpass, peak };

    static CabinetDesignBiquad make(Shape shape, double sampleRate,
                                    double frequency, double q,
                                    double gainDb = 0.0) noexcept {
        const double safeFrequency = std::clamp(frequency, 5.0, sampleRate * 0.45);
        const double omega = 2.0 * std::numbers::pi * safeFrequency / sampleRate;
        const double cosine = std::cos(omega);
        const double alpha = std::sin(omega) / (2.0 * std::max(0.1, q));
        double numerator0 = 0.0;
        double numerator1 = 0.0;
        double numerator2 = 0.0;
        double denominator0 = 1.0 + alpha;
        double denominator1 = -2.0 * cosine;
        double denominator2 = 1.0 - alpha;

        if (shape == Shape::highpass) {
            numerator0 = (1.0 + cosine) * 0.5;
            numerator1 = -(1.0 + cosine);
            numerator2 = numerator0;
        } else if (shape == Shape::lowpass) {
            numerator0 = (1.0 - cosine) * 0.5;
            numerator1 = 1.0 - cosine;
            numerator2 = numerator0;
        } else {
            const double amplitude = std::pow(10.0, gainDb / 40.0);
            numerator0 = 1.0 + alpha * amplitude;
            numerator1 = -2.0 * cosine;
            numerator2 = 1.0 - alpha * amplitude;
            denominator0 = 1.0 + alpha / amplitude;
            denominator1 = -2.0 * cosine;
            denominator2 = 1.0 - alpha / amplitude;
        }

        return CabinetDesignBiquad{
            numerator0 / denominator0,
            numerator1 / denominator0,
            numerator2 / denominator0,
            denominator1 / denominator0,
            denominator2 / denominator0,
            0.0,
            0.0
        };
    }

    double process(double input) noexcept {
        const double output = b0 * input + state1;
        state1 = b1 * input - a1 * output + state2;
        state2 = b2 * input - a2 * output;
        return output;
    }

    double b0 = 1.0;
    double b1 = 0.0;
    double b2 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double state1 = 0.0;
    double state2 = 0.0;
};

} // namespace detail

// Analyze convolution gain, not the peak of one time-domain impulse sample.
// This runs only while preparing/loading an IR on the control thread.
inline CabinetImpulseAnalysis analyzeCabinetImpulse(const std::vector<float>& impulse,
                                                    double sampleRate) noexcept {
    CabinetImpulseAnalysis result;
    if (impulse.empty()) return result;
    const double sr = std::max(8000.0, sampleRate);
    for (const float sample : impulse) {
        if (!std::isfinite(sample)) {
            result.allFinite = false;
            continue;
        }
        result.samplePeak = std::max(result.samplePeak, std::abs(sample));
    }

    constexpr double minimumMagnitude = 1.0e-12;
    double midbandDbSum = 0.0;
    constexpr int midbandPoints = 29;
    const double midbandHigh = std::min(3500.0, sr * 0.43);
    for (int index = 0; index < midbandPoints; ++index) {
        const double fraction = static_cast<double>(index)
                              / static_cast<double>(midbandPoints - 1);
        const double frequency = 280.0 * std::pow(midbandHigh / 280.0, fraction);
        const double magnitude = detail::cabinetImpulseMagnitude(impulse, sr, frequency);
        midbandDbSum += 20.0 * std::log10(std::max(minimumMagnitude, magnitude));
    }
    result.midbandGainDb = midbandDbSum / static_cast<double>(midbandPoints);

    double maximumMagnitude = 0.0;
    constexpr int responsePoints = 91;
    const double highestFrequency = std::min(10500.0, sr * 0.45);
    for (int index = 0; index < responsePoints; ++index) {
        const double fraction = static_cast<double>(index)
                              / static_cast<double>(responsePoints - 1);
        const double frequency = 40.0 * std::pow(highestFrequency / 40.0, fraction);
        const double magnitude = detail::cabinetImpulseMagnitude(impulse, sr, frequency);
        if (magnitude > maximumMagnitude) {
            maximumMagnitude = magnitude;
            result.maximumGainFrequencyHz = frequency;
        }
    }
    result.maximumGainDb = 20.0 * std::log10(
        std::max(minimumMagnitude, maximumMagnitude));
    return result;
}

struct CalibratedCabinetImpulse {
    std::vector<float> impulse;
    CabinetImpulseAnalysis before;
    CabinetImpulseAnalysis after;
    double appliedGainDb = 0.0;
};

// Preserve the shape of a measured IR. Remove inaudible DC and apply one
// broadband gain; never EQ, flatten, or limit the measured speaker response.
inline CalibratedCabinetImpulse calibrateMeasuredCabinetImpulse(
        const std::vector<float>& input, double sampleRate,
        double targetMidbandDb = -1.0, double maximumResponseDb = 4.0) {
    CalibratedCabinetImpulse result;
    if (input.empty()) return result;
    const double sr = std::max(8000.0, sampleRate);
    result.before = analyzeCabinetImpulse(input, sr);
    result.impulse.resize(input.size(), 0.0f);

    const double highpass = std::exp(-2.0 * std::numbers::pi * 18.0 / sr);
    double previousInput = 0.0;
    double previousOutput = 0.0;
    for (std::size_t index = 0; index < input.size(); ++index) {
        const double sample = std::isfinite(input[index])
            ? static_cast<double>(input[index]) : 0.0;
        const double output = highpass * (previousOutput + sample - previousInput);
        previousInput = sample;
        previousOutput = output;
        result.impulse[index] = static_cast<float>(output);
    }

    const auto cleaned = analyzeCabinetImpulse(result.impulse, sr);
    if (cleaned.samplePeak <= 1.0e-12f) {
        result.after = cleaned;
        return result;
    }
    const double midbandGain = std::pow(10.0,
        (targetMidbandDb - cleaned.midbandGainDb) / 20.0);
    const double maximumGain = std::pow(10.0,
        (maximumResponseDb - cleaned.maximumGainDb) / 20.0);
    const double gain = std::clamp(std::min(midbandGain, maximumGain),
                                   1.0e-6, 1.0e6);
    for (float& sample : result.impulse)
        sample = static_cast<float>(static_cast<double>(sample) * gain);
    result.appliedGainDb = 20.0 * std::log10(gain);
    result.after = analyzeCabinetImpulse(result.impulse, sr);
    return result;
}

// Deterministic fallback cabinet impulse used only when no measured IR has been
// supplied by the host. It is deliberately documented as a reference/smoke-test
// response, not a measured speaker/cabinet capture.
inline std::vector<float> makeReferenceCabinetImpulse(double sampleRate, int length = 2048) {
    const double sr = std::max(8000.0, sampleRate);
    const int nSamples = std::clamp(length, 128, 16384);
    std::vector<float> impulse(static_cast<std::size_t>(nSamples), 0.0f);

    auto highpass = detail::CabinetDesignBiquad::make(
        detail::CabinetDesignBiquad::Shape::highpass, sr, 58.0, 0.72);
    auto body = detail::CabinetDesignBiquad::make(
        detail::CabinetDesignBiquad::Shape::peak, sr, 112.0, 0.82, 2.6);
    auto lowMid = detail::CabinetDesignBiquad::make(
        detail::CabinetDesignBiquad::Shape::peak, sr, 720.0, 0.70, -1.1);
    auto presence = detail::CabinetDesignBiquad::make(
        detail::CabinetDesignBiquad::Shape::peak, sr, 2250.0, 0.78, 1.35);
    auto lowpass = detail::CabinetDesignBiquad::make(
        detail::CabinetDesignBiquad::Shape::lowpass, sr, 7600.0, 0.67);

    for (int n = 0; n < nSamples; ++n) {
        double sample = n == 0 ? 1.0 : 0.0;
        sample = highpass.process(sample);
        sample = body.process(sample);
        sample = lowMid.process(sample);
        sample = presence.process(sample);
        sample = lowpass.process(sample);
        impulse[static_cast<std::size_t>(n)] = static_cast<float>(sample);
    }

    const int reflection1 = static_cast<int>(std::round(0.00072 * sr));
    const int reflection2 = static_cast<int>(std::round(0.00164 * sr));
    for (int n = nSamples - 1; n >= 0; --n) {
        double y = impulse[static_cast<std::size_t>(n)];
        if (n >= reflection1)
            y += 0.075 * impulse[static_cast<std::size_t>(n - reflection1)];
        if (n >= reflection2)
            y -= 0.035 * impulse[static_cast<std::size_t>(n - reflection2)];
        impulse[static_cast<std::size_t>(n)] = static_cast<float>(y);
    }

    const auto analysis = analyzeCabinetImpulse(impulse, sr);
    if (analysis.samplePeak > 1.0e-12f) {
        const double midbandGain = std::pow(10.0,
            (-0.75 - analysis.midbandGainDb) / 20.0);
        const double boundedPeak = std::pow(10.0,
            (3.25 - analysis.maximumGainDb) / 20.0);
        const float gain = static_cast<float>(std::min(midbandGain, boundedPeak));
        for (float& sample : impulse) sample *= gain;
    }
    return impulse;
}

// Bass-specific fallback: lower cutoff and a gentler top end than the guitar
// response. Like the guitar fallback this is explicitly synthetic, not measured.
inline std::vector<float> makeReferenceBassCabinetImpulse(double sampleRate, int length = 1024) {
    const double sr = std::max(8000.0, sampleRate);
    const int count = std::clamp(length, 128, 16384);
    std::vector<float> impulse(static_cast<std::size_t>(count), 0.0f);
    const double hp = std::exp(-2.0 * std::numbers::pi * 38.0 / sr);
    const double lp = std::exp(-2.0 * std::numbers::pi * 4300.0 / sr);
    double previousInput = 0.0;
    double previousHighpass = 0.0;
    double lowpass = 0.0;

    for (int index = 0; index < count; ++index) {
        const double time = static_cast<double>(index) / sr;
        double sample = index == 0 ? 1.0 : 0.0;
        sample += 0.14 * std::exp(-time * 34.0)
            * std::sin(2.0 * std::numbers::pi * 82.0 * time);
        sample += 0.06 * std::exp(-time * 75.0)
            * std::sin(2.0 * std::numbers::pi * 410.0 * time);
        const double highpassed = hp * (previousHighpass + sample - previousInput);
        previousInput = sample;
        previousHighpass = highpassed;
        lowpass = (1.0 - lp) * highpassed + lp * lowpass;
        impulse[static_cast<std::size_t>(index)] = static_cast<float>(lowpass);
    }

    float peak = 0.0f;
    for (const float sample : impulse) peak = std::max(peak, std::abs(sample));
    double maximumResponse = 0.0;
    for (const double frequency : {45.0, 55.0, 70.0, 82.0, 98.0, 110.0,
                                   140.0, 165.0, 220.0, 330.0, 440.0,
                                   660.0, 1000.0, 2000.0, 4000.0}) {
        double real = 0.0;
        double imaginary = 0.0;
        for (int index = 0; index < count; ++index) {
            const double phase = 2.0 * std::numbers::pi * frequency
                * static_cast<double>(index) / sr;
            const double sample = impulse[static_cast<std::size_t>(index)];
            real += sample * std::cos(phase);
            imaginary -= sample * std::sin(phase);
        }
        maximumResponse = std::max(maximumResponse,
                                   std::sqrt(real * real + imaginary * imaginary));
    }
    if (peak > 1.0e-9f && maximumResponse > 1.0e-12) {
        // Peak-normalizing a long low-frequency resonance can hide 30-40 dB
        // of steady-state convolution gain and trip the output safety limiter.
        const float gain = std::min(0.82f / peak,
            static_cast<float>(1.15 / maximumResponse));
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
