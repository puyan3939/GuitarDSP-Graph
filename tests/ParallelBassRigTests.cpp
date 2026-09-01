#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/hq/BassAmpNode.h"
#include "guitardsp/hq/Measurement.h"
#include "guitardsp/hq/OctaveDownNode.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

bool graphContains(const graph::PreparedGraph& prepared, std::string_view type) {
    for (const auto id : prepared.graph.schedule()) {
        if (const auto* node = prepared.graph.node(id);
            node != nullptr && node->typeName() == type)
            return true;
    }
    return false;
}
} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;
    constexpr int samples = 16384;
    const graph::PrepareSpec spec{sampleRate, samples, 1, graph::ProcessingQuality::eco};

    {
        graph::AudioBuffer input(1, samples), output(1, samples);
        for (int index = 0; index < samples; ++index)
            input.channel(0)[index] = 0.20f * static_cast<float>(std::sin(
                2.0 * std::numbers::pi * 220.0 * static_cast<double>(index) / sampleRate));

        hq::OctaveDownNode octave;
        octave.prepare(spec);
        octave.setParameterValue(0, 1.0f);
        octave.process(input, output, samples);
        const auto tail = std::span<const float>(output.channel(0) + samples / 2,
                                                  static_cast<std::size_t>(samples / 2));
        const float octaveMagnitude = hq::singleBinMagnitude(tail, sampleRate, 110.0);
        const float originalMagnitude = hq::singleBinMagnitude(tail, sampleRate, 220.0);
        std::cout << "DIAG octave110=" << octaveMagnitude
                  << " original220=" << originalMagnitude << '\n';
        ok &= require(octaveMagnitude > 0.015f
                          && octaveMagnitude > 8.0f * originalMagnitude,
                      "monophonic octave divider creates a clean 110 Hz tone from 220 Hz");

        octave.reset();
        input.clear();
        octave.process(input, output, samples);
        float silencePeak = 0.0f;
        for (int index = 0; index < samples; ++index)
            silencePeak = std::max(silencePeak, std::abs(output.channel(0)[index]));
        ok &= require(silencePeak < 1.0e-8f,
                      "octave divider does not oscillate on digital silence");
    }

    {
        const auto bassImpulse = app::makeReferenceBassCabinetImpulse(sampleRate);
        ok &= require(bassImpulse.size() >= 512U,
                      "dedicated bass cabinet uses a real synthetic reference impulse");
        auto registry = graph::NodeRegistry::createBuiltins();
        ok &= require(registry.create("pitch.octave_down_mono") != nullptr
                          && registry.create("amp.bass_reference_hq") != nullptr
                          && registry.create("cab.bass_reference_hq") != nullptr,
                      "octave, bass amplifier, and bass cabinet are routable graph nodes");
    }

    {
        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::bypass;
        settings.ampEnabled = false;
        settings.cabinetEnabled = true;
        settings.signalRouting = app::SignalRouting::parallelOctaveBass;
        settings.guitarBranchLevel = 0.0f;
        settings.bassBranchLevel = 1.0f;
        settings.bassLevel = 1.0f;

        auto rig = app::prepareLiveRig(settings, sampleRate, 256, 1);
        ok &= require(rig != nullptr,
                      "parallel guitar and dedicated octave/bass/cabinet graph prepares");
        if (rig) {
            ok &= require(graphContains(*rig, "Split")
                              && graphContains(*rig, "Merge")
                              && graphContains(*rig, "Guitar Branch Level")
                              && graphContains(*rig, "Bass Branch Level"),
                          "parallel rig exposes true split, branch mixers, and merge nodes");
            ok &= require(graphContains(*rig, "Monophonic Octave Down")
                              && graphContains(*rig, "Bass Amp Reference")
                              && graphContains(*rig, "Bass Cabinet Reference"),
                          "parallel lower branch contains octave, bass amp, and bass IR");
            ok &= require(rig->runtime.totalLatencySamples() >= 64,
                          "parallel cabinet branches retain automatic delay compensation");

            graph::AudioBuffer input(1, 256), output(1, 256);
            std::vector<float> captured;
            captured.reserve(static_cast<std::size_t>(samples));
            for (int block = 0; block < samples / 256; ++block) {
                for (int index = 0; index < 256; ++index) {
                    const int position = block * 256 + index;
                    input.channel(0)[index] = 0.20f * static_cast<float>(std::sin(
                        2.0 * std::numbers::pi * 220.0
                        * static_cast<double>(position) / sampleRate));
                }
                rig->runtime.process(input, output, 256);
                for (int index = 0; index < 256; ++index)
                    captured.push_back(output.channel(0)[index]);
            }
            const auto tail = std::span<const float>(captured).subspan(captured.size() / 2U);
            const float sub = hq::singleBinMagnitude(tail, sampleRate, 110.0);
            const float source = hq::singleBinMagnitude(tail, sampleRate, 220.0);
            std::cout << "DIAG bass-branch110=" << sub << " source220=" << source << '\n';
            ok &= require(sub > 0.001f && sub > 3.0f * source,
                          "isolated parallel bass branch emits the actual octave-down note");
        }

        settings.signalRouting = app::SignalRouting::crossoverOctaveBass;
        auto crossover = app::prepareLiveRig(settings, sampleRate, 256, 1);
        ok &= require(crossover != nullptr && graphContains(*crossover, "CrossoverSplit"),
                      "crossover mode routes low and high bands to independent branches");
    }

    {
        app::LiveRigSettings settings;
        settings.quality = graph::ProcessingQuality::eco;
        settings.pedal = app::PedalModel::bypass;
        settings.ampEnabled = true;
        settings.cabinetEnabled = false;
        settings.bassCabinetEnabled = false;
        settings.signalRouting = app::SignalRouting::parallelOctaveBass;
        app::RealtimeAudioEngine engine;
        ok &= require(engine.configure(sampleRate, 128, 1, settings),
                      "dual-amp realtime engine prepares independent parameter targets");
        ok &= require(engine.setNodeTypeParameter("Bass Amp Reference", 0, 0.86f)
                          && engine.setNodeTypeParameter("Reference Amp Topology", 0, 0.21f)
                          && engine.setNodeTypeParameter("Bass Branch Level", 0, 0.35f)
                          && !engine.setNodeTypeParameter("missing node", 0, 0.50f),
                      "guitar and bass amplifier controls remain independently addressable");
    }

    return ok ? 0 : 1;
}
