#include "guitardsp/hq/DS1TopologyNode.h"
#include "guitardsp/hq/OfflineModelEvaluator.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}
}

int main() {
    bool ok = true;

    graph::PrepareSpec spec;
    spec.sampleRate = 48000.0;
    spec.maximumBlockSize = 128;
    spec.channels = 1;
    spec.quality = graph::ProcessingQuality::high;

    hq::DS1TopologyNode node;
    node.setParameterValue(0, 0.45f);
    node.setParameterValue(1, 0.75f);
    node.setParameterValue(2, -6.0f);

    constexpr float probeAmplitude = 0.12f;
    constexpr int settleBlocks = 5;
    constexpr int captureBlocks = 5;
    const std::vector<float> frequencies {180.0f, 700.0f, 1800.0f, 4200.0f};
    const auto target = hq::measureNodeFrequencyResponse(node, spec, frequencies, probeAmplitude,
                                                        128, settleBlocks, captureBlocks);

    std::vector<hq::FrequencyReferencePoint> reference;
    for (std::size_t i = 0; i < frequencies.size(); ++i)
        reference.push_back({frequencies[i], target[i], i == 2 ? 1.5f : 1.0f});

    node.setParameterValue(1, 0.0f);
    const auto wrong = hq::fitNodeFrequencyResponse(node, spec, reference, probeAmplitude,
                                                     128, settleBlocks, captureBlocks);
    std::cout << "wrong-tone weighted RMS dB = " << wrong.weightedRmsError << '\n';
    ok &= require(wrong.weightedRmsError > 0.05f, "wrong DS-1 tone produces measurable fit error");

    const auto fit = hq::fitParameterGrid1D(node, 1, 0.0f, 1.0f, 5, spec, reference,
                                            probeAmplitude, 128, settleBlocks, captureBlocks);
    std::cout << "best tone = " << fit.value << ", score = " << fit.score << " dB\n";
    ok &= require(std::abs(fit.value - 0.75f) < 1.0e-6f, "grid fit recovers synthetic DS-1 tone target");
    ok &= require(fit.score < 0.005f, "recovered DS-1 target has low response error");

    return ok ? 0 : 1;
}
