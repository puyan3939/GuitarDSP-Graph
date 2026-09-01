#include "guitardsp/circuit/CircuitNetlist.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec r{};
    r.resistanceOhms = ohms;
    r.tolerancePercent = 0.0f;
    return r;
}

hq::CapacitorSpec capacitor(float farads) {
    hq::CapacitorSpec c{};
    c.capacitanceFarads = farads;
    c.tolerancePercent = 0.0f;
    c.esrOhms = 0.0f;
    c.leakageResistanceOhms = 1.0e12f;
    return c;
}
} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;
    constexpr float pi = 3.14159265358979323846f;

    circuit::CircuitNetlist netlist;
    const auto supply = netlist.addNode("+9V");
    const auto inputSource = netlist.addNode("input_source");
    const auto vref = netlist.addNode("vref");
    const auto nonInv = netlist.addNode("non_inverting");
    const auto inv = netlist.addNode("inverting");
    const auto output = netlist.addNode("output");

    netlist.addVoltageSource(supply, circuit::circuitGround, 9.0f);
    const auto inputSourceId = netlist.addVoltageSource(inputSource, circuit::circuitGround, 0.0f);
    netlist.addResistor(supply, vref, resistor(47000.0f));
    netlist.addResistor(vref, circuit::circuitGround, resistor(47000.0f));
    netlist.addCapacitor(vref, circuit::circuitGround, capacitor(1.0e-6f));
    netlist.addCapacitor(inputSource, nonInv, capacitor(22.0e-9f));
    netlist.addResistor(nonInv, vref, resistor(100000.0f));
    netlist.addResistor(inv, vref, resistor(4700.0f));
    netlist.addResistor(output, inv, resistor(51000.0f));
    netlist.addDiode(output, inv, hq::component_presets::oneN4148());
    netlist.addDiode(inv, output, hq::component_presets::oneN4148());

    auto opAmp = hq::component_presets::jrc4558();
    opAmp.inputOffsetVoltage = 0.0f;
    opAmp.inputBiasCurrentAmps = 0.0f;
    netlist.addDynamicOpAmp(output, nonInv, inv, supply,
                            circuit::circuitGround, circuit::circuitGround, opAmp);
    netlist.addResistor(output, circuit::circuitGround, resistor(100000.0f));

    circuit::CompiledCircuit compiled;
    std::string error;
    ok &= require(netlist.compile(sampleRate, compiled, &error),
                  "pedal-like nonlinear circuit compiles through CircuitNetlist");

    circuit::Node outputNode{};
    ok &= require(compiled.findNode(output, outputNode),
                  "pedal-like circuit preserves stable output node");
    const auto* inputBinding = compiled.binding(inputSourceId);
    ok &= require(inputBinding != nullptr &&
                  inputBinding->kind == circuit::CircuitComponentKind::voltageSource,
                  "pedal-like circuit preserves audio source binding");

    compiled.engine.setNonlinearSolverMode(
        circuit::MnaCircuitEngine::NonlinearSolverMode::denseReference);
    ok &= require(compiled.engine.sparseNonlinearSolverAvailable(),
                  "pedal-like nonlinear circuit has a prepared sparse pattern");

    const auto inputHandle = inputBinding != nullptr
        ? static_cast<circuit::SourceHandle>(inputBinding->handle)
        : circuit::SourceHandle{};

    bool healthy = true;
    int settleUnconverged = 0;
    for (int sample = 0; sample < 4096; ++sample) {
        compiled.engine.setVoltageSource(inputHandle, 0.0f);
        const auto stats = compiled.engine.processSample(40, 1.0e-5f);
        healthy &= !stats.singular && std::isfinite(compiled.engine.voltage(outputNode));
        settleUnconverged += stats.converged ? 0 : 1;
    }

    compiled.engine.resetPerformanceStats();
    float minimum = std::numeric_limits<float>::infinity();
    float maximum = -std::numeric_limits<float>::infinity();
    int drivenUnconverged = 0;
    for (int sample = 0; sample < 2048; ++sample) {
        const float phase = 2.0f * pi * 220.0f * static_cast<float>(sample) /
                            static_cast<float>(sampleRate);
        const float input = 0.22f * std::sin(phase) + 0.04f * std::sin(3.0f * phase);
        compiled.engine.setVoltageSource(inputHandle, input);
        const auto stats = compiled.engine.processSample(40, 1.0e-5f);
        healthy &= !stats.singular;
        drivenUnconverged += stats.converged ? 0 : 1;
        const float value = compiled.engine.voltage(outputNode);
        healthy &= std::isfinite(value);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }

    const auto performance = compiled.engine.performanceStats();
    std::cout << "DIAG dense-pedal settle_unconverged=" << settleUnconverged
              << " driven_unconverged=" << drivenUnconverged
              << " min=" << minimum
              << " max=" << maximum
              << " final=" << compiled.engine.voltage(outputNode)
              << " sparse_solves=" << performance.sparseNewtonSolves
              << " sparse_fallbacks=" << performance.sparseFallbackSolves
              << " dense_solves=" << performance.generalLinearSolves
              << " density=" << compiled.engine.sparseNonlinearFactorDensity() << '\n';

    ok &= require(healthy, "pedal-like dense MNA transient stays finite and nonsingular");
    ok &= require(settleUnconverged + drivenUnconverged < 64,
                  "pedal-like nonlinear network reaches and maintains Newton convergence");
    ok &= require(minimum > -0.25f && maximum < 9.25f,
                  "single-supply op amp remains inside physical supply neighborhood");
    ok &= require(maximum - minimum > 0.05f,
                  "pedal-like circuit produces a driven AC output");

    return ok ? 0 : 1;
}
