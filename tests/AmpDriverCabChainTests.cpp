#include "guitardsp/hq/AmpFamilyNodes.h"
#include "guitardsp/hq/CabinetChainNode.h"
#include "guitardsp/hq/ReferenceAmpTopologyNode.h"

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

void fillSine(graph::AudioBuffer& b, int samples, double sr, double hz, float amplitude) {
    for (int ch = 0; ch < b.channels(); ++ch)
        for (int i = 0; i < samples; ++i)
            b.channel(ch)[i] = amplitude * static_cast<float>(std::sin(2.0 * std::numbers::pi * hz * static_cast<double>(i) / sr));
}

float differenceRms(const graph::AudioBuffer& a, const graph::AudioBuffer& b, int start, int samples) {
    double e = 0.0;
    int n = 0;
    for (int ch = 0; ch < std::min(a.channels(), b.channels()); ++ch) {
        for (int i = start; i < samples; ++i) {
            const double d = static_cast<double>(a.channel(ch)[i]) - static_cast<double>(b.channel(ch)[i]);
            e += d * d;
            ++n;
        }
    }
    return n > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(n))) : 0.0f;
}

bool finiteBuffer(const graph::AudioBuffer& b, int samples) {
    for (int ch = 0; ch < b.channels(); ++ch)
        for (int i = 0; i < samples; ++i)
            if (!std::isfinite(b.channel(ch)[i])) return false;
    return true;
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
    spec.quality = graph::ProcessingQuality::high;

    graph::AudioBuffer input(1, samples), a(1, samples), b(1, samples);
    fillSine(input, samples, sr, 440.0, 0.08f);

    {
        hq::ReferenceAmpTopologyNode amp;
        amp.prepare(spec);
        amp.setParameterValue(0, 0.55f);
        amp.setParameterValue(4, 0.65f);
        amp.setParameterValue(8, 1.0f);
        amp.setParameterValue(7, 0.0f);
        amp.setParameterValue(9, 1.0f);
        amp.setParameterValue(10, 1.0f);
        amp.reset();
        amp.process(input, a, samples);

        amp.setParameterValue(9, 2.0f);
        amp.setParameterValue(10, 2.0f);
        amp.reset();
        amp.process(input, b, samples);

        ok &= require(finiteBuffer(a, samples) && finiteBuffer(b, samples), "amp driver/feedback paths remain finite");
        ok &= require(differenceRms(a, b, samples / 2, samples) > 1.0e-7f,
                      "cathode-follower/British and plate/American paths differ");
    }

    {
        hq::BritishPlexiFamilyNode british;
        hq::AmericanCleanFamilyNode american;
        british.prepare(spec);
        american.prepare(spec);
        british.setParameterValue(0, 0.50f);
        american.setParameterValue(0, 0.50f);
        british.process(input, a, samples);
        american.process(input, b, samples);
        ok &= require(finiteBuffer(a, samples) && finiteBuffer(b, samples), "family amp paths remain finite");
        ok &= require(differenceRms(a, b, samples / 2, samples) > 1.0e-7f,
                      "British and American family topology outputs differ");
    }

    {
        hq::CabinetChainNode cab;
        cab.setPartitionSize(64);
        cab.setImpulseResponse({1.0f, 0.35f, -0.15f, 0.07f, -0.03f});
        cab.prepare(spec);
        cab.process(input, a, samples);
        ok &= require(finiteBuffer(a, samples), "cabinet chain output remains finite");
        ok &= require(cab.latencySamples() == 64, "cabinet chain reports partition latency");

        cab.reset();
        cab.setParameterValue(0, 0.0f);
        cab.process(input, a, samples);
        cab.reset();
        cab.setParameterValue(0, 0.85f);
        cab.process(input, b, samples);
        ok &= require(differenceRms(a, b, 512, samples) > 1.0e-8f,
                      "speaker compression changes cabinet-chain output");

        graph::AudioBuffer impulse(1, samples);
        impulse.clear();
        impulse.channel(0)[0] = 0.2f;
        cab.setImpulseResponse({1.0f});
        cab.prepare(spec);
        cab.setParameterValue(0, 0.0f);
        cab.setParameterValue(1, 0.0f);
        cab.setParameterValue(2, 0.0f);

        cab.setParameterValue(4, 0.0f);
        cab.reset();
        cab.process(impulse, a, samples);
        cab.setParameterValue(4, 1.0f);
        cab.reset();
        cab.process(impulse, b, samples);

        const int latency = cab.latencySamples();
        ok &= require(std::abs(a.channel(0)[0]) < 1.0e-8f
                          && std::abs(a.channel(0)[latency]) > 0.05f,
                      "dry cabinet mix retains convolution latency alignment");
        const float dryArrival = a.channel(0)[latency];
        const float wetArrival = b.channel(0)[latency];
        ok &= require(std::abs(b.channel(0)[0]) < 1.0e-8f
                          && wetArrival > 0.001f,
                      "filtered dry and wet cabinet paths share the same impulse arrival");

        cab.setParameterValue(4, 0.5f);
        cab.reset();
        cab.process(impulse, a, samples);
        ok &= require(std::abs(a.channel(0)[0]) < 1.0e-8f
                          && std::abs(a.channel(0)[latency]
                                      - 0.5f * (dryArrival + wetArrival)) < 1.0e-5f,
                      "50 percent cabinet IR mix recombines aligned dry and filtered wet paths");
    }

    return ok ? 0 : 1;
}
