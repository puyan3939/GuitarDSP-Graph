#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

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
}

int main() {
    runLinearLadder();
    runNonlinearLadder();
    return 0;
}
