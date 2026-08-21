#include "guitardsp/circuit/DynamicOpAmpSubcircuit.h"
#include "guitardsp/circuit/SwitchRelaySubcircuit.h"
#include "guitardsp/circuit/TransformerSubcircuit.h"
#include "guitardsp/circuit/TriodeParasiticSubcircuit.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec r{};
    r.resistanceOhms = ohms;
    return r;
}
}

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;

    {
        circuit::MnaCircuitEngine c;
        const auto positiveRail = c.addNode();
        const auto negativeRail = c.addNode();
        const auto input = c.addNode();
        const auto output = c.addNode();
        c.addVoltageSource(positiveRail, circuit::ground, 9.0f);
        c.addVoltageSource(negativeRail, circuit::ground, -9.0f);
        const auto inputSource = c.addVoltageSource(input, circuit::ground, 1.0f);
        c.addResistor(output, circuit::ground, resistor(10000.0f));
        const auto opAmp = circuit::addDynamicOpAmpSubcircuit(
            c, output, input, output, positiveRail, negativeRail, circuit::ground,
            hq::component_presets::jrc4558());
        (void)opAmp;
        ok &= require(c.prepare(sampleRate), "dynamic op-amp macro prepares");
        for (int i = 0; i < 800; ++i) c.processSample(40, 1.0e-6f);
        ok &= require(c.voltage(output) > 0.95f && c.voltage(output) < 1.05f,
                      "dominant-pole op-amp closes unity-gain feedback");

        c.setVoltageSource(inputSource, 10.0f);
        circuit::MnaCircuitEngine::SolveStats opAmpStats{};
        for (int i = 0; i < 2000; ++i) opAmpStats = c.processSample(40, 1.0e-6f);
        const float saturatedOutput = c.voltage(output);
        std::cout << "INFO opamp saturated output=" << saturatedOutput
                  << " converged=" << opAmpStats.converged
                  << " singular=" << opAmpStats.singular
                  << " iterations=" << opAmpStats.iterations << '\n';
        ok &= require(std::isfinite(saturatedOutput) && saturatedOutput > 6.8f && saturatedOutput < 8.2f,
                      "op-amp output soft-clamps below positive rail");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto bPlus = c.addNode();
        const auto plate = c.addNode();
        const auto gridDrive = c.addNode();
        const auto grid = c.addNode();
        const auto cathode = c.addNode();
        c.addVoltageSource(bPlus, circuit::ground, 250.0f);
        c.addVoltageSource(gridDrive, circuit::ground, 0.0f);
        c.addResistor(bPlus, plate, resistor(100000.0f));
        c.addResistor(gridDrive, grid, resistor(68000.0f));
        c.addResistor(grid, circuit::ground, resistor(1000000.0f));
        c.addResistor(cathode, circuit::ground, resistor(1500.0f));
        const auto triode = circuit::addTriodeParasiticSubcircuit(
            c, plate, grid, cathode, hq::component_presets::twelveAX7());
        (void)triode;
        ok &= require(c.prepare(sampleRate), "triode parasitic subcircuit prepares");
        circuit::MnaCircuitEngine::SolveStats stats{};
        for (int i = 0; i < 1000; ++i) stats = c.processSample(40, 1.0e-5f);
        ok &= require(!stats.singular && std::isfinite(c.voltage(plate)) &&
                      std::isfinite(c.voltage(cathode)),
                      "triode with Cgp/Cgk/Cpk and grid-current branch stays finite");
        ok &= require(c.voltage(plate) > 20.0f && c.voltage(plate) < 249.0f &&
                      c.voltage(cathode) > 0.1f,
                      "12AX7 parasitic circuit reaches a biased operating point");
    }

    {
        hq::TransformerSpec spec{};
        spec.primaryInductanceHenries = 20.0f;
        spec.magnetizingSaturationCurrentAmps = 0.10f;
        spec.coreSaturationExponent = 2.0f;
        spec.minimumMagnetizingInductanceRatio = 0.10f;
        const float smallSignal = circuit::detail::saturatedMagnetizingInductance(spec, 0.0f);
        const float atKnee = circuit::detail::saturatedMagnetizingInductance(spec, 0.10f);
        const float deep = circuit::detail::saturatedMagnetizingInductance(spec, 1.0f);
        ok &= require(std::abs(smallSignal - 20.0f) < 1.0e-4f,
                      "transformer core keeps nominal small-signal inductance");
        ok &= require(atKnee < smallSignal && deep < atKnee && deep >= 2.0f,
                      "transformer core smoothly collapses magnetizing inductance");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto common = c.addNode();
        const auto throwA = c.addNode();
        const auto throwB = c.addNode();
        c.addVoltageSource(common, circuit::ground, 1.0f);
        c.addResistor(throwA, circuit::ground, resistor(10000.0f));
        c.addResistor(throwB, circuit::ground, resistor(10000.0f));
        auto spec = hq::component_presets::toggleSwitch();
        spec.form = hq::SwitchContactForm::spdt;
        const auto sw = circuit::addSwitchSubcircuit(c, common, throwA, throwB,
            circuit::ground, circuit::ground, circuit::ground, spec, false);
        ok &= require(c.prepare(sampleRate), "SPDT switch subcircuit prepares");
        c.processSample();
        ok &= require(c.voltage(throwA) > 0.99f && c.voltage(throwB) < 0.01f,
                      "SPDT resting state routes common to throw A");
        ok &= require(circuit::setSwitchState(c, sw, spec, true),
                      "SPDT switch changes state without topology rebuild");
        c.processSample();
        ok &= require(c.voltage(throwB) > 0.99f && c.voltage(throwA) < 0.01f,
                      "SPDT thrown state routes common to throw B");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto coilPositive = c.addNode();
        const auto common = c.addNode();
        const auto nc = c.addNode();
        const auto no = c.addNode();
        const auto coilSource = c.addVoltageSource(coilPositive, circuit::ground, 9.0f);
        c.addVoltageSource(common, circuit::ground, 1.0f);
        c.addResistor(nc, circuit::ground, resistor(10000.0f));
        c.addResistor(no, circuit::ground, resistor(10000.0f));

        auto spec = hq::component_presets::signalRelay9V();
        spec.form = hq::RelayContactForm::spdt;
        spec.coilResistanceOhms = 100.0f;
        spec.coilInductanceHenries = 1.0e-3f;
        spec.pickupVoltage = 3.0f;
        spec.dropoutVoltage = 1.0f;
        spec.operateMilliseconds = 0.20f;
        spec.releaseMilliseconds = 0.20f;
        spec.bounceMilliseconds = 0.10f;
        spec.bounceTransitions = 2;

        const auto relay = circuit::addRelaySubcircuit(c, coilPositive, circuit::ground,
            common, nc, no, circuit::ground, circuit::ground, circuit::ground, spec);
        circuit::RelayRuntimeState state{};
        ok &= require(c.prepare(sampleRate), "relay coil and contacts prepare in common MNA system");

        for (int i = 0; i < 1000; ++i) {
            c.processSample();
            circuit::updateRelayRuntime(c, relay, state, spec, sampleRate);
        }
        ok &= require(state.energized && circuit::relayCoilCurrent(c, relay) > 0.05f,
                      "relay energizes from physical coil current with operate delay");
        c.processSample();
        ok &= require(c.voltage(no) > 0.99f && c.voltage(nc) < 0.01f,
                      "energized relay transfers SPDT contact to normally-open throw");

        c.setVoltageSource(coilSource, 0.0f);
        for (int i = 0; i < 1000; ++i) {
            c.processSample();
            circuit::updateRelayRuntime(c, relay, state, spec, sampleRate);
        }
        ok &= require(!state.energized,
                      "relay releases after coil current falls below dropout threshold");
        c.processSample();
        ok &= require(c.voltage(nc) > 0.99f && c.voltage(no) < 0.01f,
                      "released relay returns to normally-closed throw");
    }

    return ok ? 0 : 1;
}
