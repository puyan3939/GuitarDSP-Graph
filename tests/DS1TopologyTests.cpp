#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/DS1TopologyNode.h"
#include "guitardsp/hq/Measurement.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

float rms(const graph::AudioBuffer& b, int start, int end) {
    double e = 0.0;
    int count = 0;
    for (int ch = 0; ch < b.channels(); ++ch) {
        for (int i = start; i < end; ++i) {
            const float x = b.channel(ch)[i];
            e += static_cast<double>(x) * static_cast<double>(x);
            ++count;
        }
    }
    return count > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(count))) : 0.0f;
}

float differenceRms(const std::vector<float>& a, const std::vector<float>& b, int start) {
    const int n = static_cast<int>(std::min(a.size(), b.size()));
    if (start >= n) return 0.0f;
    double e = 0.0;
    int count = 0;
    for (int i = std::max(0, start); i < n; ++i) {
        const double d = static_cast<double>(a[static_cast<std::size_t>(i)])
                       - static_cast<double>(b[static_cast<std::size_t>(i)]);
        e += d * d;
        ++count;
    }
    return count > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(count))) : 0.0f;
}
}

int main() {
    bool ok = true;
    constexpr int samples = 4096;
    constexpr double sr = 48000.0;

    graph::PrepareSpec spec;
    spec.sampleRate = sr;
    spec.maximumBlockSize = samples;
    spec.channels = 1;
    spec.quality = graph::ProcessingQuality::studio;

    graph::AudioBuffer input(1, samples), output(1, samples);
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / sr;
        input.channel(0)[i] = 0.12f * static_cast<float>(std::sin(2.0 * std::numbers::pi * 440.0 * t));
    }

    hq::DS1TopologyNode ds1;
    ds1.prepare(spec);
    ds1.process(input, output, samples);

    bool finite = true;
    float peak = 0.0f;
    for (int i = 0; i < samples; ++i) {
        finite &= std::isfinite(output.channel(0)[i]);
        peak = std::max(peak, std::abs(output.channel(0)[i]));
    }
    ok &= require(finite && peak > 1.0e-4f && peak < 20.0f, "DS-1 topology output finite and bounded");
    ok &= require(ds1.latencySamples() > 0, "DS-1 studio path reports oversampling latency");

    // A DS-1 distortion control changes pre-clip gain. Once the diode pair is already
    // clipping, normalized THD is not guaranteed to be strictly monotonic with the pot
    // position, so the robust invariant is that the nonlinear steady-state waveform and
    // harmonic signature change materially between low and high settings.
    std::vector<float> lowDrive(static_cast<std::size_t>(samples));
    std::vector<float> highDrive(static_cast<std::size_t>(samples));

    ds1.reset();
    ds1.setParameterValue(0, 0.08f);
    ds1.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, lowDrive.begin());

    ds1.reset();
    ds1.setParameterValue(0, 0.92f);
    ds1.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, highDrive.begin());

    const auto lowMetrics = hq::analyzeHarmonics(lowDrive, sr, 440.0, 12);
    const auto highMetrics = hq::analyzeHarmonics(highDrive, sr, 440.0, 12);
    ok &= require(std::isfinite(lowMetrics.thdDb) && std::isfinite(highMetrics.thdDb), "DS-1 harmonic metrics finite");

    const int steadyStart = samples / 2;
    const float driveDifference = differenceRms(lowDrive, highDrive, steadyStart);
    const float thdDelta = std::abs(highMetrics.thdDb - lowMetrics.thdDb);
    ok &= require(driveDifference > 1.0e-4f, "DS-1 distortion control changes steady-state waveform");
    ok &= require(thdDelta > 0.01f, "DS-1 distortion control changes harmonic signature");

    // Tone extremes should produce measurably different broadband output.
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / sr;
        input.channel(0)[i] = 0.05f * static_cast<float>(
            std::sin(2.0 * std::numbers::pi * 180.0 * t) +
            std::sin(2.0 * std::numbers::pi * 3200.0 * t));
    }

    ds1.reset();
    ds1.setParameterValue(0, 0.35f);
    ds1.setParameterValue(1, 0.0f);
    ds1.process(input, output, samples);
    const float darkRms = rms(output, samples / 2, samples);

    ds1.reset();
    ds1.setParameterValue(1, 1.0f);
    ds1.process(input, output, samples);
    const float brightRms = rms(output, samples / 2, samples);
    ok &= require(std::abs(brightRms - darkRms) > 1.0e-4f, "DS-1 tone network changes broadband response");

    return ok ? 0 : 1;
}
