#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/ReferenceCabinetIR.h"

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
}

int main() {
    bool ok = true;

    {
        const auto ir = app::makeReferenceCabinetImpulse(48000.0);
        float peak = 0.0f;
        float tailEnergy = 0.0f;
        for (std::size_t i = 0; i < ir.size(); ++i) {
            peak = std::max(peak, std::abs(ir[i]));
            if (i > 32U) tailEnergy += ir[i] * ir[i];
        }
        ok &= require(ir.size() >= 512U && peak > 0.5f && peak <= 0.93f,
                      "reference fallback cabinet IR is normalized and nontrivial");
        ok &= require(tailEnergy > 1.0e-5f,
                      "reference fallback cabinet IR has a real time-domain tail");

        const auto resampled = app::resampleImpulseWindowedSinc(ir, 48000.0, 96000.0);
        ok &= require(resampled.size() > ir.size(),
                      "offline cabinet IR sinc resampler follows target sample rate");
    }

    {
        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::ts808Circuit;
        settings.amp = app::AmpModel::reference;
        settings.cabinetPartitionSize = 64;

        auto rig = app::prepareLiveRig(settings, 48000.0, 64, 1);
        ok &= require(rig != nullptr, "TS808 -> amp -> speaker/cab rig prepares");
        if (rig) {
            ok &= require(rig->runtime.totalLatencySamples() >= 64,
                          "live rig reports cabinet plus oversampling latency");
            graph::AudioBuffer input(1, 64), output(1, 64);
            bool finite = true;
            float energy = 0.0f;
            for (int block = 0; block < 5; ++block) {
                for (int i = 0; i < 64; ++i) {
                    const int sample = block * 64 + i;
                    input.channel(0)[i] = 0.08f * std::sin(
                        2.0f * std::numbers::pi_v<float> * 220.0f
                        * static_cast<float>(sample) / 48000.0f);
                }
                rig->runtime.process(input, output, 64);
                for (int i = 0; i < 64; ++i) {
                    finite &= std::isfinite(output.channel(0)[i]);
                    energy += output.channel(0)[i] * output.channel(0)[i];
                }
            }
            ok &= require(finite && energy > 1.0e-8f,
                          "complete live rig produces finite audible-path output");
        }
    }

    {
        app::RealtimeAudioEngine engine;
        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::bypass;
        settings.ampEnabled = false;
        settings.cabinetEnabled = false;
        ok &= require(engine.configure(48000.0, 64, 2, settings),
                      "realtime device bridge configures a stereo pass-through rig");
        engine.setInputTrimDb(0.0f);
        engine.setOutputTrimDb(0.0f);

        float mono[64]{};
        float left[64]{};
        float right[64]{};
        for (int i = 0; i < 64; ++i) mono[i] = 0.1f * std::sin(0.1f * static_cast<float>(i));
        const float* inputs[]{mono};
        float* outputs[]{left, right};
        engine.process(inputs, 1, outputs, 2, 64);

        float difference = 0.0f;
        float outputEnergy = 0.0f;
        for (int i = 0; i < 64; ++i) {
            difference += std::abs(left[i] - right[i]);
            outputEnergy += left[i] * left[i];
        }
        ok &= require(difference < 1.0e-6f && outputEnergy > 1.0e-5f,
                      "mono guitar input is duplicated coherently to stereo outputs");
        ok &= require(engine.stats().callbacks == 1,
                      "realtime device bridge publishes callback telemetry");
    }

    return ok ? 0 : 1;
}
