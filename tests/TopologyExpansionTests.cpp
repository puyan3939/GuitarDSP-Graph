#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/BD2TopologyNode.h"
#include "guitardsp/hq/MeasuredFit.h"
#include "guitardsp/hq/ReferenceAmpTopologyNode.h"
#include "guitardsp/hq/TwoTransistorFuzzNode.h"

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

bool finiteAndNonSilent(const graph::AudioBuffer& b, int samples, float& peak) {
    peak = 0.0f;
    for (int ch = 0; ch < b.channels(); ++ch) {
        for (int i = 0; i < samples; ++i) {
            const float x = b.channel(ch)[i];
            if (!std::isfinite(x)) return false;
            peak = std::max(peak, std::abs(x));
        }
    }
    return peak > 1.0e-8f;
}

float differenceRms(const std::vector<float>& a, const std::vector<float>& b, int start) {
    const std::size_t count = std::min(a.size(), b.size());
    if (count == 0 || start >= static_cast<int>(count)) return 0.0f;
    double e = 0.0;
    int n = 0;
    for (std::size_t i = static_cast<std::size_t>(std::max(0, start)); i < count; ++i) {
        const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        e += d * d;
        ++n;
    }
    return n > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(n))) : 0.0f;
}

void fillSine(graph::AudioBuffer& b, int samples, double sr, double hz, float amplitude) {
    for (int ch = 0; ch < b.channels(); ++ch) {
        for (int i = 0; i < samples; ++i) {
            const double t = static_cast<double>(i) / sr;
            b.channel(ch)[i] = amplitude * static_cast<float>(std::sin(2.0 * std::numbers::pi * hz * t));
        }
    }
}
}

int main() {
    bool ok = true;

    {
        const std::vector<hq::FrequencyReferencePoint> reference {
            {100.0f, -3.0f, 0.5f}, {1000.0f, 1.0f, 1.0f}, {5000.0f, -6.0f, 0.8f}
        };
        const std::vector<float> exact {-3.0f, 1.0f, -6.0f};
        const std::vector<float> shifted {-1.0f, 3.0f, -4.0f};
        const auto zero = hq::compareFrequencyResponse(reference, exact);
        const auto error = hq::compareFrequencyResponse(reference, shifted);
        ok &= require(zero.rmsError < 1.0e-7f && zero.points == 3, "measured fit exact response scores zero");
        ok &= require(error.rmsError > 1.9f && error.maxAbsError >= 2.0f, "measured fit reports response error");
    }

    constexpr int samples = 4096;
    constexpr double sr = 48000.0;
    graph::PrepareSpec spec;
    spec.sampleRate = sr;
    spec.maximumBlockSize = samples;
    spec.channels = 1;
    spec.quality = graph::ProcessingQuality::studio;

    graph::AudioBuffer input(1, samples), output(1, samples);
    fillSine(input, samples, sr, 330.0, 0.10f);

    {
        hq::BD2TopologyNode node;
        node.prepare(spec);
        node.process(input, output, samples);
        float peak = 0.0f;
        ok &= require(finiteAndNonSilent(output, samples, peak) && peak < 30.0f, "BD-2 topology finite and bounded");
        ok &= require(node.latencySamples() > 0, "BD-2 reports studio oversampling latency");

        std::vector<float> low(samples), high(samples);
        node.reset(); node.setParameterValue(0, 0.10f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, low.begin());
        node.reset(); node.setParameterValue(0, 0.90f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, high.begin());
        ok &= require(differenceRms(low, high, samples / 2) > 1.0e-6f, "BD-2 drive changes steady waveform");
    }

    {
        hq::TwoTransistorFuzzNode node;
        node.prepare(spec);
        node.process(input, output, samples);
        float peak = 0.0f;
        ok &= require(finiteAndNonSilent(output, samples, peak) && peak < 30.0f, "two-transistor fuzz finite and bounded");
        ok &= require(node.latencySamples() > 0, "two-transistor fuzz reports studio oversampling latency");

        std::vector<float> normal(samples), starved(samples);
        node.reset(); node.setParameterValue(1, 0.50f); node.setParameterValue(2, 0.0f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, normal.begin());
        node.reset(); node.setParameterValue(1, 0.72f); node.setParameterValue(2, 0.85f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, starved.begin());
        ok &= require(differenceRms(normal, starved, samples / 2) > 1.0e-6f, "fuzz bias/starve changes steady waveform");
    }

    {
        hq::ReferenceAmpTopologyNode node;
        node.prepare(spec);
        node.process(input, output, samples);
        float peak = 0.0f;
        ok &= require(finiteAndNonSilent(output, samples, peak) && peak < 50.0f, "reference amp topology finite and bounded");
        ok &= require(node.latencySamples() > 0, "reference amp reports studio oversampling latency");

        std::vector<float> clean(samples), driven(samples);
        node.reset(); node.setParameterValue(0, 0.12f); node.setParameterValue(4, 0.25f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, clean.begin());
        node.reset(); node.setParameterValue(0, 0.82f); node.setParameterValue(4, 0.75f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, driven.begin());
        ok &= require(differenceRms(clean, driven, samples / 2) > 1.0e-8f, "amp gain/master changes steady waveform");

        std::vector<float> el34(samples), sixl6(samples), kt88(samples);
        node.setParameterValue(0, 0.55f);
        node.setParameterValue(4, 0.78f);
        node.reset(); node.setParameterValue(7, 0.0f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, el34.begin());
        node.reset(); node.setParameterValue(7, 1.0f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, sixl6.begin());
        node.reset(); node.setParameterValue(7, 2.0f); node.process(input, output, samples);
        std::copy(output.channel(0), output.channel(0) + samples, kt88.begin());
        ok &= require(differenceRms(el34, sixl6, samples / 2) > 1.0e-9f,
                      "reference amp EL34 and 6L6 voicings differ");
        ok &= require(differenceRms(sixl6, kt88, samples / 2) > 1.0e-9f,
                      "reference amp 6L6 and KT88 voicings differ");
    }

    return ok ? 0 : 1;
}
