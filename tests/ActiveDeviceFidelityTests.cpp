#include "guitardsp/circuit/ActiveDeviceParasiticSubcircuits.h"
#include "guitardsp/circuit/DynamicOpAmpSubcircuit.h"

#include <algorithm>
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
    r.tolerancePercent = 0.0f;
    return r;
}

float firstSampleBjtBase(float capacitance) {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto base = c.addNode();
    c.addVoltageSource(input, circuit::ground, 0.010f);
    c.addResistor(input, base, resistor(100000.0f));
    auto spec = hq::component_presets::twoN3904();
    circuit::BjtParasiticValues p{};
    p.baseEmitterFarads = capacitance;
    p.baseCollectorFarads = 0.0f;
    circuit::addBjtParasiticSubcircuit(c, circuit::ground, base, circuit::ground, spec, p);
    c.prepare(48000.0);
    c.processSample(24, 1.0e-7f);
    return c.voltage(base);
}

float firstSampleJfetGate(float capacitance) {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto gate = c.addNode();
    c.addVoltageSource(input, circuit::ground, 0.010f);
    c.addResistor(input, gate, resistor(100000.0f));
    auto spec = hq::component_presets::j201();
    circuit::JfetParasiticValues p{};
    p.gateSourceFarads = capacitance;
    p.gateDrainFarads = 0.0f;
    p.drainSourceFarads = 0.0f;
    circuit::addJfetParasiticSubcircuit(c, circuit::ground, gate, circuit::ground, spec, p);
    c.prepare(48000.0);
    c.processSample(24, 1.0e-7f);
    return c.voltage(gate);
}

float firstSampleMosfetGate(float capacitance) {
    circuit::MnaCircuitEngine c;
    const auto input = c.addNode();
    const auto gate = c.addNode();
    c.addVoltageSource(input, circuit::ground, 0.010f);
    c.addResistor(input, gate, resistor(100000.0f));
    auto spec = hq::component_presets::bs170();
    circuit::MosfetParasiticValues p{};
    p.gateSourceFarads = capacitance;
    p.gateDrainFarads = 0.0f;
    p.drainSourceFarads = 0.0f;
    circuit::addMosfetParasiticSubcircuit(c, circuit::ground, gate, circuit::ground, spec, p);
    c.prepare(48000.0);
    c.processSample(24, 1.0e-7f);
    return c.voltage(gate);
}
} // namespace

int main() {
    bool ok = true;

    {
        circuit::MnaCircuitEngine c;
        const auto positiveRail = c.addNode();
        const auto negativeRail = c.addNode();
        const auto input = c.addNode();
        const auto output = c.addNode();
        const auto inputSource = c.addVoltageSource(input, circuit::ground, 0.0f);
        c.addVoltageSource(positiveRail, circuit::ground, 9.0f);
        c.addVoltageSource(negativeRail, circuit::ground, -9.0f);
        c.addResistor(output, circuit::ground, resistor(10000.0f));

        auto spec = hq::component_presets::jrc4558();
        spec.inputOffsetVoltage = 0.0f;
        spec.inputBiasCurrentAmps = 0.0f;
        spec.slewRateVoltsPerSecond = 48000.0f; // 1 V/sample at 48 kHz
        spec.outputCurrentLimitAmps = 0.025f;
        spec.positiveRailHeadroomVolts = 1.5f;
        spec.negativeRailHeadroomVolts = 1.5f;
        circuit::addDynamicOpAmpSubcircuit(c, output, input, output,
                                            positiveRail, negativeRail,
                                            circuit::ground, spec);
        ok &= require(c.prepare(48000.0), "bounded dynamic op amp prepares");
        for (int i = 0; i < 64; ++i) c.processSample(40, 1.0e-5f);
        c.setVoltageSource(inputSource, 20.0f);

        float previous = c.voltage(output);
        float maxStep = 0.0f;
        bool finiteAndNonsingular = true;
        circuit::MnaCircuitEngine::SolveStats last{};
        for (int i = 0; i < 256; ++i) {
            last = c.processSample(40, 1.0e-5f);
            const float now = c.voltage(output);
            finiteAndNonsingular &= !last.singular && std::isfinite(now);
            maxStep = std::max(maxStep, std::abs(now - previous));
            previous = now;
        }
        ok &= require(finiteAndNonsingular && last.converged,
                      "bounded op amp remains finite and recovers Newton convergence after overdrive");
        ok &= require(maxStep < 1.35f,
                      "op amp output obeys configured slew-rate envelope");
        ok &= require(c.voltage(output) < 7.8f && c.voltage(output) > 6.0f,
                      "op amp positive output is bounded by rail headroom");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto positiveRail = c.addNode();
        const auto negativeRail = c.addNode();
        const auto input = c.addNode();
        const auto output = c.addNode();
        c.addVoltageSource(positiveRail, circuit::ground, 9.0f);
        c.addVoltageSource(negativeRail, circuit::ground, -9.0f);
        c.addVoltageSource(input, circuit::ground, 6.0f);
        c.addResistor(output, circuit::ground, resistor(100.0f));

        auto spec = hq::component_presets::jrc4558();
        spec.inputOffsetVoltage = 0.0f;
        spec.inputBiasCurrentAmps = 0.0f;
        spec.slewRateVoltsPerSecond = 20.0e6f;
        spec.outputCurrentLimitAmps = 5.0e-3f;
        spec.outputResistanceOhms = 1.0f;
        circuit::addDynamicOpAmpSubcircuit(c, output, input, output,
                                            positiveRail, negativeRail,
                                            circuit::ground, spec);
        ok &= require(c.prepare(48000.0), "current-limited op amp prepares");
        bool converged = true;
        for (int i = 0; i < 256; ++i) {
            const auto stats = c.processSample(40, 1.0e-5f);
            converged &= !stats.singular && stats.converged;
        }
        const float outputVoltage = c.voltage(output);
        ok &= require(converged, "current-limited op amp converges into heavy load");
        ok &= require(outputVoltage > 0.20f && outputVoltage < 0.80f,
                      "op amp output current limit bounds heavy-load voltage");
    }

    {
        const float small = firstSampleBjtBase(10.0e-12f);
        const float large = firstSampleBjtBase(10.0e-9f);
        ok &= require(large < small * 0.5f,
                      "BJT Cbe parasitic changes the actual MNA transient");
    }

    {
        const float small = firstSampleJfetGate(5.0e-12f);
        const float large = firstSampleJfetGate(10.0e-9f);
        ok &= require(large < small * 0.5f,
                      "JFET Cgs parasitic changes the actual MNA transient");
    }

    {
        const float small = firstSampleMosfetGate(30.0e-12f);
        const float large = firstSampleMosfetGate(10.0e-9f);
        ok &= require(large < small * 0.5f,
                      "MOSFET Cgs parasitic changes the actual MNA transient");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto negativeSupply = c.addNode();
        const auto drain = c.addNode();
        const auto gate = c.addNode();
        c.addVoltageSource(negativeSupply, circuit::ground, -2.0f);
        c.addVoltageSource(gate, circuit::ground, 0.0f);
        c.addResistor(negativeSupply, drain, resistor(2200.0f));
        auto spec = hq::component_presets::bs170();
        circuit::MosfetParasiticValues p{};
        circuit::addMosfetParasiticSubcircuit(c, drain, gate, circuit::ground, spec, p);
        c.prepare(48000.0);
        const auto stats = c.processSample(40, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged,
                      "MOSFET body-diode fixture converges");
        ok &= require(c.voltage(drain) > -1.2f && c.voltage(drain) < -0.25f,
                      "N-MOSFET body diode clamps reverse drain excursion");
    }

    return ok ? 0 : 1;
}
