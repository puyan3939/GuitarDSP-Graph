#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/DS1TopologyNode.h"
#include "guitardsp/hq/AliasAnalysis.h"
#include "guitardsp/hq/MultiParameterFit.h"
#include "guitardsp/hq/PowerTubeModels.h"
#include "guitardsp/hq/SpeakerDynamicsNode.h"

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
    int n = 0;
    for (int ch = 0; ch < b.channels(); ++ch)
        for (int i = start; i < end; ++i) {
            const double x = b.channel(ch)[i];
            e += x * x;
            ++n;
        }
    return n > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(n))) : 0.0f;
}
}

int main() {
    bool ok = true;

    graph::PrepareSpec spec;
    spec.sampleRate = 48000.0;
    spec.maximumBlockSize = 128;
    spec.channels = 1;
    spec.quality = graph::ProcessingQuality::high;

    // Synthetic two-parameter calibration: tone and output level are both known
    // and deliberately included in the bounded coordinate grids.
    {
        hq::DS1TopologyNode node;
        node.setParameterValue(0, 0.45f);
        node.setParameterValue(1, 0.75f);
        node.setParameterValue(2, -6.0f);
        const std::vector<float> frequencies {160.0f, 450.0f, 1100.0f, 2600.0f, 5200.0f};
        constexpr float amplitude = 0.12f;
        constexpr int settle = 5;
        constexpr int capture = 5;
        const auto target = hq::measureNodeFrequencyResponse(node, spec, frequencies,
                                                             amplitude, 128, settle, capture);
        std::vector<hq::FrequencyReferencePoint> reference;
        for (std::size_t i = 0; i < frequencies.size(); ++i)
            reference.push_back({frequencies[i], target[i], 1.0f});

        node.setParameterValue(1, 0.0f);
        node.setParameterValue(2, -12.0f);
        const std::vector<hq::FitParameterRange> ranges {
            {1, 0.0f, 1.0f, 5},
            {2, -12.0f, 0.0f, 3}
        };
        const auto fit = hq::fitParametersCoordinateGrid(node, ranges, 3, spec, reference,
                                                          amplitude, 128, settle, capture);
        ok &= require(fit.values.size() == 2, "multi-parameter fit returns all parameters");
        ok &= require(std::abs(fit.values[0] - 0.75f) < 1.0e-6f,
                      "multi-parameter fit recovers DS-1 tone");
        ok &= require(std::abs(fit.values[1] + 6.0f) < 1.0e-6f,
                      "multi-parameter fit recovers DS-1 level");
        ok &= require(fit.score < 0.20f && fit.evaluations > 1,
                      "multi-parameter fit reaches low bounded error");
    }

    // Device-family primitives should not collapse to one identical transfer.
    {
        const auto el34 = hq::PowerTubeModel::forType(hq::PowerTubeType::el34);
        const auto sixl6 = hq::PowerTubeModel::forType(hq::PowerTubeType::sixL6GC);
        const auto kt88 = hq::PowerTubeModel::forType(hq::PowerTubeType::kt88);
        const float a = el34.transfer(0.8f, 0.85f);
        const float b = sixl6.transfer(0.8f, 0.85f);
        const float c = kt88.transfer(0.8f, 0.85f);
        ok &= require(std::isfinite(a) && std::isfinite(b) && std::isfinite(c),
                      "power tube family outputs are finite");
        ok &= require(std::abs(a - b) > 1.0e-4f && std::abs(b - c) > 1.0e-4f,
                      "EL34/6L6/KT88 family transfers differ");
    }

    // Speaker dynamics must remain bounded and compress a sustained loud tone
    // more strongly when voice-coil compression is increased.
    {
        constexpr int samples = 4096;
        graph::PrepareSpec speakerSpec {48000.0, samples, 1, graph::ProcessingQuality::high};
        graph::AudioBuffer input(1, samples), output(1, samples);
        for (int i = 0; i < samples; ++i) {
            const double t = static_cast<double>(i) / speakerSpec.sampleRate;
            input.channel(0)[i] = 0.75f * static_cast<float>(
                std::sin(2.0 * std::numbers::pi * 110.0 * t));
        }

        hq::SpeakerDynamicsNode speaker;
        speaker.prepare(speakerSpec);
        speaker.setParameterValue(1, 0.30f);
        speaker.setParameterValue(0, 0.0f);
        speaker.process(input, output, samples);
        const float openRms = rms(output, samples / 2, samples);

        speaker.reset();
        speaker.setParameterValue(0, 0.90f);
        speaker.process(input, output, samples);
        const float compressedRms = rms(output, samples / 2, samples);

        bool finite = true;
        for (int i = 0; i < samples; ++i) finite &= std::isfinite(output.channel(0)[i]);
        ok &= require(finite && compressedRms > 1.0e-6f, "speaker dynamics output finite and non-silent");
        ok &= require(compressedRms < openRms, "voice-coil compression reduces sustained level");
    }

    // The default 18% cone-excursion branch must not inject nonharmonic
    // artifacts when a guitar note decays into the -60 dBFS region.
    {
        constexpr int samples = 4096;
        constexpr double frequency = 48000.0 * 19.0 / samples;
        graph::PrepareSpec quietSpec {48000.0, samples, 1, graph::ProcessingQuality::eco};
        graph::AudioBuffer input(1, samples), output(1, samples);
        for (int i = 0; i < samples; ++i) {
            input.channel(0)[i] = 0.001f * static_cast<float>(std::sin(
                2.0 * std::numbers::pi * frequency * i / quietSpec.sampleRate));
        }
        hq::SpeakerDynamicsNode speaker;
        speaker.prepare(quietSpec);
        speaker.setParameterValue(0, 0.0f);
        speaker.setParameterValue(1, 0.18f);
        speaker.setParameterValue(2, 0.0f);
        speaker.process(input, output, samples);
        const auto metrics = hq::analyzeAliasResidual(
            std::span<const float>(output.channel(0), samples),
            quietSpec.sampleRate, frequency, 48, 3);
        std::cout << "DIAG quiet-speaker nonharmonic_db=" << metrics.residualDb << '\n';
        ok &= require(metrics.residualDb < -75.0f,
                      "speaker excursion does not add audible quiet-note digital noise");
    }

    return ok ? 0 : 1;
}
