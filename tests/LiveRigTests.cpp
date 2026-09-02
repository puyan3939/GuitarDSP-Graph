#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/hq/Components.h"

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
        const auto triode = hq::TriodeModel::twelveAX7();
        const float nearCutoff = triode.plateCurrent(-2.0f, 180.0f);
        const float belowCutoff = triode.plateCurrent(-3.0f, 180.0f);
        const float furtherBelowCutoff = triode.plateCurrent(-4.0f, 180.0f);
        ok &= require(nearCutoff > belowCutoff
                          && belowCutoff > furtherBelowCutoff
                          && furtherBelowCutoff > 0.0f,
                      "12AX7 retains a smooth, grid-dependent current below nominal cutoff");
    }

    {
        const auto ir = app::makeReferenceCabinetImpulse(48000.0);
        float peak = 0.0f;
        float tailEnergy = 0.0f;
        for (std::size_t i = 0; i < ir.size(); ++i) {
            peak = std::max(peak, std::abs(ir[i]));
            if (i > 32U) tailEnergy += ir[i] * ir[i];
        }
        const auto response = app::analyzeCabinetImpulse(ir, 48000.0);
        ok &= require(ir.size() >= 512U && peak > 0.05f && peak < 0.9f
                          && response.maximumGainDb <= 3.3
                          && response.midbandGainDb > -2.0,
                      "reference cabinet IR bounds convolution gain rather than sample peak");
        ok &= require(tailEnergy > 1.0e-6f,
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

    {
        // Issue #47: PreampCircuit/FullAmpCircuit moved from PedalModel to
        // AmpModel, so the amp slot alone (no separate CIRCUIT PEDAL node) is
        // what now carries a component-level amp -- confirming this also rules
        // out the "double amp in series" shape the old pedal-slot placement
        // allowed (pedal + amp both resolving to an amp-category node).
        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::bypass;
        settings.amp = app::AmpModel::fullAmpCircuit;
        settings.ampEnabled = true;
        settings.cabinetEnabled = false;

        auto rig = app::prepareLiveRig(settings, 48000.0, 64, 1);
        ok &= require(rig != nullptr, "Full Amp Circuit prepares from the AmpModel slot");
        if (rig) {
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
                          "Full Amp Circuit as AmpModel produces finite audible output");
        }
    }

    {
        constexpr int blockSize = 512;
        constexpr int blocks = 32;
        constexpr double sampleRate = 44100.0;

        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::bypass;
        settings.amp = app::AmpModel::reference;
        settings.ampEnabled = true;
        settings.cabinetEnabled = true;

        app::RealtimeAudioEngine engine;
        ok &= require(engine.configure(sampleRate, blockSize, 2, settings),
                      "44.1 kHz stereo amp/cab hardware rig prepares without a pedal");
        engine.setInputRoutingMode(app::InputRoutingMode::autoMono);
        engine.setOutputTrimDb(-12.0f);
        engine.setMuted(false);

        std::vector<float> silentLeft(static_cast<std::size_t>(blockSize), 0.0f);
        std::vector<float> activeRight(static_cast<std::size_t>(blockSize), 0.0f);
        std::vector<float> outputLeft(static_cast<std::size_t>(blockSize), 0.0f);
        std::vector<float> outputRight(static_cast<std::size_t>(blockSize), 0.0f);
        const float* inputs[]{silentLeft.data(), activeRight.data()};
        float* outputs[]{outputLeft.data(), outputRight.data()};

        float finalPeak = 0.0f;
        for (int block = 0; block < blocks; ++block) {
            for (int i = 0; i < blockSize; ++i) {
                const int sample = block * blockSize + i;
                activeRight[static_cast<std::size_t>(i)] = 0.20f * std::sin(
                    2.0f * std::numbers::pi_v<float> * 220.0f
                    * static_cast<float>(sample) / static_cast<float>(sampleRate));
            }
            engine.process(inputs, 2, outputs, 2, blockSize);
            if (block == blocks - 1) {
                for (const float sample : outputLeft)
                    finalPeak = std::max(finalPeak, std::abs(sample));
            }
        }

        ok &= require(engine.stats().selectedInputChannel == 1 && finalPeak > 1.0e-5f,
                      "right-jack guitar stays audible after tube self-bias settles");
    }

    return ok ? 0 : 1;
}
