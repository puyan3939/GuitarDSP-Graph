#include "guitardsp/app/RealtimeAudioEngine.h"

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {

using guitardsp::app::AmpModel;
using guitardsp::app::LiveRigSettings;
using guitardsp::app::PedalModel;
using guitardsp::app::RealtimeAudioEngine;
using guitardsp::app::SignalRouting;

constexpr double sampleRate = 48000.0;
constexpr int blockSize = 512;
constexpr int warmupBlocks = 12;
constexpr int measuredBlocks = 128;

bool run(std::string_view name, const LiveRigSettings& settings,
         float amplitude = 0.10f) {
    RealtimeAudioEngine engine;
    if (!engine.configure(sampleRate, blockSize, 1, settings)) {
        std::cerr << name << ": failed to prepare rig\n";
        return false;
    }
    engine.setInputRoutingMode(guitardsp::app::InputRoutingMode::input2);
    engine.setInputTrimDb(0.0f);
    engine.setOutputTrimDb(-12.0f);

    std::array<float, blockSize> input1{};
    std::array<float, blockSize> input2{};
    std::array<float, blockSize> output1{};
    std::array<float, blockSize> output2{};
    const float* inputs[]{input1.data(), input2.data()};
    float* outputs[]{output1.data(), output2.data()};

    int sampleOffset = 0;
    const auto processBlock = [&]() {
        for (int sample = 0; sample < blockSize; ++sample) {
            const float phase = static_cast<float>(sampleOffset + sample)
                * 0.0287979326f;
            input2[static_cast<std::size_t>(sample)] = amplitude
                * (std::sin(phase) + 0.13f * std::sin(phase * 3.01f));
        }
        engine.process(inputs, 2, outputs, 2, blockSize);
        sampleOffset += blockSize;
    };

    for (int block = 0; block < warmupBlocks; ++block) processBlock();
    engine.resetDiagnostics();
    const auto started = std::chrono::steady_clock::now();
    for (int block = 0; block < measuredBlocks; ++block) processBlock();
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    const double budget = static_cast<double>(measuredBlocks * blockSize) / sampleRate;
    const auto stats = engine.stats();

    std::cout << std::left << std::setw(25) << name
              << " avg=" << std::right << std::setw(6)
              << 100.0 * static_cast<double>(stats.performance.averageLoad)
              << "% p99=" << std::setw(6)
              << 100.0 * static_cast<double>(stats.performance.percentile99Load)
              << "% peak=" << std::setw(6)
              << 100.0 * static_cast<double>(stats.performance.peakLoad)
              << "% wall=" << std::setw(6) << 100.0 * elapsed / budget
              << "% misses=" << stats.performance.deadlineMisses << '\n';
    return true;
}

} // namespace

int main() {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "48 kHz / 512 samples / mono DSP -> stereo output / Eco 2x\n";

    LiveRigSettings settings;
    settings.quality = guitardsp::graph::ProcessingQuality::eco;
    settings.pedal = PedalModel::bypass;
    settings.ampEnabled = false;
    settings.cabinetEnabled = false;

    bool ok = run("Dry bypass", settings);
    settings.cabinetEnabled = true;
    ok &= run("Cabinet IR only", settings);
    settings.cabinetEnabled = false;
    settings.ampEnabled = true;
    settings.amp = AmpModel::reference;
    ok &= run("Reference amp only", settings);
    settings.amp = AmpModel::britishPlexiFamily;
    ok &= run("British Plexi only", settings);
    settings.ampEnabled = false;
    settings.pedal = PedalModel::ts808Circuit;
    ok &= run("TS808 circuit only", settings);
    ok &= run("TS808 quiet input", settings, 0.0035f);
    settings.ampEnabled = true;
    settings.cabinetEnabled = false;
    ok &= run("TS808 + British Plexi", settings);
    settings.cabinetEnabled = true;
    ok &= run("TS808 + amp + cabinet", settings);
    settings.signalRouting = SignalRouting::parallelOctaveBass;
    ok &= run("Full parallel bass rig", settings);
    settings.signalRouting = SignalRouting::serialGuitar;
    settings.pedal = PedalModel::ds1Circuit;
    ok &= run("DS-1 + amp + cabinet", settings);

    return ok ? 0 : 1;
}
