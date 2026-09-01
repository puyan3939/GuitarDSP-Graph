#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/Measurement.h"
#include "guitardsp/hq/TS808TopologyNode.h"

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

float rms(const std::vector<float>& x, int start) {
    double e = 0.0;
    int count = 0;
    for (int i = std::max(0, start); i < static_cast<int>(x.size()); ++i) {
        e += static_cast<double>(x[static_cast<std::size_t>(i)]) * x[static_cast<std::size_t>(i)];
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
        input.channel(0)[i] = 0.08f * static_cast<float>(std::sin(2.0 * std::numbers::pi * 500.0 * t));
    }

    hq::TS808TopologyNode ts;
    ts.prepare(spec);
    ts.process(input, output, samples);

    bool finite = true;
    float peak = 0.0f;
    for (int i = 0; i < samples; ++i) {
        finite &= std::isfinite(output.channel(0)[i]);
        peak = std::max(peak, std::abs(output.channel(0)[i]));
    }
    ok &= require(finite && peak > 1.0e-5f && peak < 30.0f, "TS808 topology output finite and bounded");
    ok &= require(ts.latencySamples() > 0, "TS808 studio path reports oversampling latency");

    std::vector<float> lowDrive(static_cast<std::size_t>(samples));
    std::vector<float> highDrive(static_cast<std::size_t>(samples));

    ts.reset();
    ts.setParameterValue(0, 0.05f);
    ts.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, lowDrive.begin());

    ts.reset();
    ts.setParameterValue(0, 0.95f);
    ts.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, highDrive.begin());

    const auto lowMetrics = hq::analyzeHarmonics(lowDrive, sr, 500.0, 12);
    const auto highMetrics = hq::analyzeHarmonics(highDrive, sr, 500.0, 12);
    ok &= require(std::isfinite(lowMetrics.thdDb) && std::isfinite(highMetrics.thdDb), "TS808 harmonic metrics finite");
    ok &= require(std::abs(highMetrics.thdDb - lowMetrics.thdDb) > 0.01f, "TS808 drive changes harmonic signature");
    ok &= require(std::abs(rms(highDrive, samples / 2) - rms(lowDrive, samples / 2)) > 1.0e-4f, "TS808 drive changes steady-state level/shape");

    // Tone control must change a mixed low/high-frequency stimulus.
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / sr;
        input.channel(0)[i] = 0.04f * static_cast<float>(
            std::sin(2.0 * std::numbers::pi * 220.0 * t) +
            std::sin(2.0 * std::numbers::pi * 3600.0 * t));
    }

    std::vector<float> dark(static_cast<std::size_t>(samples));
    std::vector<float> bright(static_cast<std::size_t>(samples));
    ts.reset();
    ts.setParameterValue(0, 0.35f);
    ts.setParameterValue(1, 0.0f);
    ts.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, dark.begin());

    ts.reset();
    ts.setParameterValue(1, 1.0f);
    ts.process(input, output, samples);
    std::copy(output.channel(0), output.channel(0) + samples, bright.begin());
    ok &= require(std::abs(rms(bright, samples / 2) - rms(dark, samples / 2)) > 1.0e-4f, "TS808 tone changes broadband response");

    return ok ? 0 : 1;
}
