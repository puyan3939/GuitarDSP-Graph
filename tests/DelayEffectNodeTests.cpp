#include "guitardsp/dsp/DelayEffectNode.h"
#include "guitardsp/graph/NodeRegistry.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

using guitardsp::dsp::DelayEffectNode;
using guitardsp::dsp::DigitalDelayEngine;
using guitardsp::graph::AudioBuffer;
using guitardsp::graph::AudioNode;
using guitardsp::graph::NodeRegistry;
using guitardsp::graph::PrepareSpec;
using guitardsp::graph::ProcessingQuality;

namespace {

bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

// Runs `node` over `input` (mono) block-by-block and returns the mono output,
// preserving the node's internal state across the whole call (no reset).
std::vector<float> runMono(AudioNode& node, const std::vector<float>& input, int block) {
    std::vector<float> output(input.size(), 0.0f);
    AudioBuffer in(1, block), out(1, block);
    std::size_t pos = 0;
    while (pos < input.size()) {
        const int n = static_cast<int>(std::min<std::size_t>(static_cast<std::size_t>(block), input.size() - pos));
        in.clear();
        for (int i = 0; i < n; ++i) in.channel(0)[i] = input[pos + static_cast<std::size_t>(i)];
        node.process(in, out, n);
        for (int i = 0; i < n; ++i) output[pos + static_cast<std::size_t>(i)] = out.channel(0)[i];
        pos += static_cast<std::size_t>(n);
    }
    return output;
}

float rms(const std::vector<float>& v, std::size_t begin, std::size_t end) {
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t i = begin; i < end && i < v.size(); ++i) { sum += static_cast<double>(v[i]) * v[i]; ++count; }
    return count ? static_cast<float>(std::sqrt(sum / static_cast<double>(count))) : 0.0f;
}

} // namespace

int main() {
    bool ok = true;
    constexpr double sr = 48000.0;
    constexpr int block = 64;

    // --- Registry ---
    {
        auto registry = NodeRegistry::createBuiltins();
        ok &= require(registry.create("time.digital_delay") != nullptr, "registry creates digital delay");
    }

    // --- Delay-time accuracy: a single tap lands within +/-1 sample of Time ---
    {
        DelayEffectNode node;
        PrepareSpec spec{sr, block, 1, ProcessingQuality::high};
        node.prepare(spec);
        node.setParameterValue(0, 100.0f); // time = 100 ms -> exactly 4800 samples at 48 kHz
        node.setParameterValue(1, 0.0f);   // feedback = 0: isolate a single echo
        node.setParameterValue(3, 1.0f);   // mix = 1: fully wet

        // Prime for 500 ms so the delay-time smoothing fully settles before measuring.
        std::vector<float> priming(static_cast<std::size_t>(0.5 * sr), 0.0f);
        runMono(node, priming, block);

        constexpr int expectedOffset = 4800;
        std::vector<float> probe(static_cast<std::size_t>(expectedOffset) + 500, 0.0f);
        probe[0] = 1.0f;
        const std::vector<float> out = runMono(node, probe, block);

        std::size_t peakIndex = 0;
        float peakValue = 0.0f;
        for (std::size_t i = 0; i < out.size(); ++i) {
            if (std::abs(out[i]) > peakValue) { peakValue = std::abs(out[i]); peakIndex = i; }
        }
        ok &= require(std::llabs(static_cast<long long>(peakIndex) - expectedOffset) <= 1,
                      "delay tap lands within 1 sample of the configured Time");
        ok &= require(peakValue > 0.99f, "delay tap preserves impulse amplitude at feedback=0, mix=1");

        // No energy should show up well before or well after the single tap.
        const float before = rms(out, 0, static_cast<std::size_t>(expectedOffset) - 50);
        const float after = rms(out, static_cast<std::size_t>(expectedOffset) + 50, out.size());
        ok &= require(before < 1.0e-5f, "no pre-echo before the configured delay time");
        ok &= require(after < 1.0e-5f, "no extra repeats when feedback is zero");
    }

    // --- Feedback stability: bounded and decaying, never diverges ---
    {
        DelayEffectNode node;
        PrepareSpec spec{sr, block, 1, ProcessingQuality::high};
        node.prepare(spec);
        node.setParameterValue(0, 50.0f);                          // time = 50 ms
        node.setParameterValue(1, DigitalDelayEngine::kMaxFeedback); // max feedback the node allows
        node.setParameterValue(2, 0.5f);                            // tone = middle
        node.setParameterValue(3, 1.0f);                            // mix = 1, wet only

        std::vector<float> priming(static_cast<std::size_t>(0.2 * sr), 0.0f);
        runMono(node, priming, block);

        std::vector<float> probe(static_cast<std::size_t>(5.0 * sr), 0.0f);
        probe[0] = 1.0f;
        const std::vector<float> out = runMono(node, probe, block);

        bool finite = true;
        float peakAbs = 0.0f;
        for (float v : out) {
            if (!std::isfinite(v)) finite = false;
            peakAbs = std::max(peakAbs, std::abs(v));
        }
        ok &= require(finite, "feedback delay output stays finite at max feedback over 5 seconds");
        ok &= require(peakAbs < 2.0f, "feedback delay output stays bounded at max feedback");

        const float earlyRms = rms(out, 0, static_cast<std::size_t>(0.5 * sr));
        const float lateRms = rms(out, out.size() - static_cast<std::size_t>(0.5 * sr), out.size());
        ok &= require(earlyRms > 1.0e-4f, "feedback delay produces audible early repeats");
        ok &= require(lateRms < 0.05f * earlyRms, "feedback delay repeats decay well below the initial level");
    }

    // --- No self-oscillation on silence, even at max feedback ---
    {
        DelayEffectNode node;
        PrepareSpec spec{sr, block, 1, ProcessingQuality::high};
        node.prepare(spec);
        node.setParameterValue(0, 100.0f);
        node.setParameterValue(1, DigitalDelayEngine::kMaxFeedback);
        node.setParameterValue(2, 0.5f);
        node.setParameterValue(3, 1.0f);

        std::vector<float> silence(static_cast<std::size_t>(2.0 * sr), 0.0f);
        const std::vector<float> out = runMono(node, silence, block);
        const float silenceRms = rms(out, 0, out.size());
        ok &= require(silenceRms < 1.0e-9f, "silence in produces silence out, no self-oscillation at max feedback");
    }

    // --- Mix = 0 is a transparent dry pass-through ---
    {
        DelayEffectNode node;
        PrepareSpec spec{sr, block, 1, ProcessingQuality::high};
        node.prepare(spec);
        node.setParameterValue(0, 300.0f);
        node.setParameterValue(1, 0.6f);
        node.setParameterValue(3, 0.0f); // mix = 0: dry only

        std::vector<float> in(static_cast<std::size_t>(0.05 * sr));
        for (std::size_t i = 0; i < in.size(); ++i)
            in[i] = 0.2f * std::sin(2.0 * 3.14159265358979323846 * 220.0 * static_cast<double>(i) / sr);
        const std::vector<float> out = runMono(node, in, block);

        float maxDiff = 0.0f;
        for (std::size_t i = 0; i < in.size(); ++i) maxDiff = std::max(maxDiff, std::abs(out[i] - in[i]));
        ok &= require(maxDiff < 1.0e-6f, "mix=0 reproduces the dry signal exactly");
    }

    // --- Parameters clamp to their descriptor ranges ---
    {
        DelayEffectNode node;
        node.setParameterValue(1, 5.0f); // way above max
        ok &= require(node.parameterValue(1) <= DigitalDelayEngine::kMaxFeedback,
                      "feedback parameter clamps to the descriptor maximum (< 1.0 for stability)");
    }

    return ok ? 0 : 1;
}
