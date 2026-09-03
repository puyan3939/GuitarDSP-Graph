#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/MnaCircuitEngine.h"
#include "guitardsp/circuit/TS808Circuit.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

using namespace guitardsp;

namespace {
hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec spec{};
    spec.resistanceOhms = ohms;
    return spec;
}

hq::CapacitorSpec capacitor(float farads) {
    hq::CapacitorSpec spec{};
    spec.capacitanceFarads = farads;
    spec.leakageResistanceOhms = 1.0e12f;
    spec.esrOhms = 0.0f;
    return spec;
}

void runLinearLadder() {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto source = c.addVoltageSource(input, circuit::ground, 0.0f);
    auto previous = input;
    for (int i = 0; i < 32; ++i) {
        const auto node = c.addNode();
        c.addResistor(previous, node, resistor(1000.0f + 20.0f * static_cast<float>(i)));
        c.addCapacitor(node, circuit::ground, capacitor(100.0e-9f + 2.0e-9f * static_cast<float>(i)));
        previous = node;
    }
    c.addResistor(previous, circuit::ground, resistor(10000.0f));
    if (!c.prepare(48000.0)) return;
    c.resetPerformanceStats();

    constexpr std::size_t samples = 200000;
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < samples; ++i) {
        const float phase = static_cast<float>(i % 480U) / 480.0f;
        c.setVoltageSource(source, std::sin(phase * 6.28318530718f));
        c.processSample();
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(end - begin).count();
    const auto stats = c.performanceStats();
    std::cout << "linear_32_stage samples=" << samples
              << " seconds=" << seconds
              << " samples_per_second=" << static_cast<double>(samples) / seconds
              << " cache_rebuilds=" << stats.staticCacheRebuilds
              << " factorizations=" << stats.fullFactorizations
              << " cached_solves=" << stats.cachedLinearSolves << '\n';
}

void runNonlinearLadder() {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto source = c.addVoltageSource(input, circuit::ground, 0.0f);
    auto previous = input;
    for (int i = 0; i < 8; ++i) {
        const auto node = c.addNode();
        c.addResistor(previous, node, resistor(2200.0f));
        c.addCapacitor(node, circuit::ground, capacitor(47.0e-9f));
        c.addDiode(node, circuit::ground, hq::component_presets::oneN4148());
        previous = node;
    }
    c.addResistor(previous, circuit::ground, resistor(10000.0f));
    if (!c.prepare(48000.0)) return;
    c.resetPerformanceStats();

    constexpr std::size_t samples = 20000;
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < samples; ++i) {
        const float phase = static_cast<float>(i % 480U) / 480.0f;
        c.setVoltageSource(source, 0.9f * std::sin(phase * 6.28318530718f));
        c.processSample(16, 1.0e-6f);
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(end - begin).count();
    const auto stats = c.performanceStats();
    std::cout << "nonlinear_8_stage samples=" << samples
              << " seconds=" << seconds
              << " samples_per_second=" << static_cast<double>(samples) / seconds
              << " cache_rebuilds=" << stats.staticCacheRebuilds
              << " nonlinear_assemblies=" << stats.nonlinearAssemblies
              << " general_solves=" << stats.generalLinearSolves << '\n';
}

// Component-level pedal netlists, not synthetic ladders: TS808/DS-1 at their
// default control settings, driven by a swept guitar-level sine. sampleRate is
// varied to stand in for oversampling factor (e.g. 96000 Hz ~= a 2x/"Eco"
// oversampled 48 kHz engine), since MnaCircuitEngine itself has no oversampling
// concept -- the host feeds it samples at whatever rate the oversampling stage
// upsamples to. This addresses docs/MNA_ACCELERATION.md's "Next acceleration
// stages" item 4 (benchmark representative pedal/amp netlists).
void runTs808(double sampleRate, const std::string& label) {
    circuit::TS808Circuit c;
    if (!c.prepare(sampleRate)) return;
    c.engine().resetPerformanceStats();

    const auto samples = static_cast<std::size_t>(sampleRate * 2.0);
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < samples; ++i) {
        const float phase = static_cast<float>(i % 480U) / 480.0f;
        c.processSample(0.35f * std::sin(phase * 6.28318530718f));
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(end - begin).count();
    const auto stats = c.engine().performanceStats();
    std::cout << "ts808_" << label << " sample_rate=" << sampleRate
              << " samples=" << samples
              << " seconds=" << seconds
              << " samples_per_second=" << static_cast<double>(samples) / seconds
              << " estimated_cpu_percent="
              << (sampleRate / (static_cast<double>(samples) / seconds)) * 100.0
              << " nonlinear_assemblies=" << stats.nonlinearAssemblies
              << " sparse_newton_solves=" << stats.sparseNewtonSolves
              << " sparse_fallback_solves=" << stats.sparseFallbackSolves
              << " general_solves=" << stats.generalLinearSolves << '\n';
}

void runDs1(double sampleRate, const std::string& label) {
    circuit::DS1Circuit c;
    if (!c.prepare(sampleRate)) return;
    c.engine().resetPerformanceStats();

    const auto samples = static_cast<std::size_t>(sampleRate * 2.0);
    const auto begin = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < samples; ++i) {
        const float phase = static_cast<float>(i % 480U) / 480.0f;
        c.processSample(0.35f * std::sin(phase * 6.28318530718f));
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(end - begin).count();
    const auto stats = c.engine().performanceStats();
    std::cout << "ds1_" << label << " sample_rate=" << sampleRate
              << " samples=" << samples
              << " seconds=" << seconds
              << " samples_per_second=" << static_cast<double>(samples) / seconds
              << " estimated_cpu_percent="
              << (sampleRate / (static_cast<double>(samples) / seconds)) * 100.0
              << " nonlinear_assemblies=" << stats.nonlinearAssemblies
              << " sparse_newton_solves=" << stats.sparseNewtonSolves
              << " sparse_fallback_solves=" << stats.sparseFallbackSolves
              << " general_solves=" << stats.generalLinearSolves << '\n';
}
}

int main() {
    runLinearLadder();
    runNonlinearLadder();
    runTs808(48000.0, "1x_48k");
    runTs808(96000.0, "eco2x_96k");
    runDs1(48000.0, "1x_48k");
    runDs1(96000.0, "eco2x_96k");
    return 0;
}
