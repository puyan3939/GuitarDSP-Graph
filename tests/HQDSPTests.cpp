#include "guitardsp/hq/ADAA.h"
#include "guitardsp/hq/DiodeClipper.h"
#include "guitardsp/hq/HQDriveNode.h"
#include "guitardsp/hq/Measurement.h"
#include "guitardsp/hq/QualityPolicy.h"
#include <cmath>
#include <iostream>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool c, const char* n) { std::cout << (c ? "PASS " : "FAIL ") << n << '\n'; return c; }
bool finiteBuffer(const graph::AudioBuffer& b) {
    for (int ch=0; ch<b.channels(); ++ch) for (int i=0; i<b.samples(); ++i) if (!std::isfinite(b.channel(ch)[i])) return false;
    return true;
}
}

int main() {
    bool ok = true;

    const auto live = hq::qualityFor(graph::ProcessingQuality::live, graph::NodeCategory::drive);
    const auto studio = hq::qualityFor(graph::ProcessingQuality::studio, graph::NodeCategory::drive);
    ok &= require(live.oversamplingFactor == 4, "live drive uses 4x oversampling");
    ok &= require(studio.oversamplingFactor == 16, "studio drive uses 16x oversampling");
    ok &= require(studio.resamplerTaps > live.resamplerTaps, "studio resampler is longer");

    hq::ImplicitDiodeClipper diode;
    diode.reset();
    float prev = -100.0f;
    bool monotonic = true;
    for (int i=0; i<=400; ++i) {
        const float x = -2.0f + 4.0f * static_cast<float>(i) / 400.0f;
        const float y = diode.process(x);
        monotonic &= std::isfinite(y) && y >= prev - 1.0e-4f;
        prev = y;
    }
    ok &= require(monotonic, "implicit diode solver remains finite and monotonic");

    hq::ADAATanh adaa;
    adaa.reset();
    bool adaaFinite = true;
    for (int i=0; i<2000; ++i) {
        const float x = 4.0f * std::sin(0.077f * static_cast<float>(i));
        adaaFinite &= std::isfinite(adaa.process(x));
    }
    ok &= require(adaaFinite, "ADAA tanh is finite under hard drive");

    constexpr int block = 1024;
    graph::AudioBuffer input(2, block), output(2, block);
    for (int i=0; i<block; ++i) {
        const float s = 0.22f * std::sin(2.0 * 3.14159265358979323846 * 1000.0 * static_cast<double>(i) / 48000.0);
        input.channel(0)[i] = s; input.channel(1)[i] = s;
    }

    hq::HQDriveNode drive;
    graph::PrepareSpec spec; spec.sampleRate=48000.0; spec.maximumBlockSize=block; spec.channels=2; spec.quality=graph::ProcessingQuality::studio;
    drive.prepare(spec);
    drive.setParameterValue(0, 0.72f);
    drive.process(input, output, block);
    ok &= require(finiteBuffer(output), "HQ drive output finite");
    ok &= require(drive.latencySamples() >= 0, "HQ drive reports resampler latency");

    std::vector<float> left(static_cast<std::size_t>(block));
    for (int i=0; i<block; ++i) left[static_cast<std::size_t>(i)] = output.channel(0)[i];
    const auto metrics = hq::analyzeHarmonics(left, 48000.0, 1000.0, 10);
    ok &= require(metrics.fundamental > 1.0e-5f, "harmonic analyzer sees fundamental");
    ok &= require(metrics.thd > 1.0e-4f, "nonlinear drive creates measurable harmonics");
    ok &= require(std::isfinite(metrics.thdDb) && std::isfinite(metrics.highBandEnergy), "measurement metrics finite");

    return ok ? 0 : 1;
}
