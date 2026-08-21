#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

circuit::MnaCircuitEngine makeRc(float capacitance) {
    circuit::MnaCircuitEngine c;
    const auto in = c.addNode();
    const auto out = c.addNode();
    hq::ResistorSpec r{};
    r.resistanceOhms = 1000.0f;
    hq::CapacitorSpec cap{};
    cap.capacitanceFarads = capacitance;
    cap.leakageResistanceOhms = 1.0e12f;
    cap.esrOhms = 0.0f;
    c.addVoltageSource(in, circuit::ground, 1.0f);
    c.addResistor(in, out, r);
    c.addCapacitor(out, circuit::ground, cap);
    c.prepare(48000.0);
    return c;
}
}

int main() {
    bool ok = true;

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 1000.0f;
        const auto source = c.addVoltageSource(in, circuit::ground, 1.0f);
        c.addResistor(in, out, r);
        c.addResistor(out, circuit::ground, r);
        ok &= require(c.prepare(48000.0), "MNA prepares resistor divider");
        const auto stats = c.processSample();
        ok &= require(!stats.singular && std::abs(c.voltage(out) - 0.5f) < 1.0e-5f,
                      "MNA resistor divider solves 0.5 V");
        c.setVoltageSource(source, -0.8f);
        c.processSample();
        ok &= require(std::abs(c.voltage(out) + 0.4f) < 1.0e-5f,
                      "MNA realtime voltage source update works");
    }

    {
        auto fast = makeRc(1.0e-6f);
        auto slow = makeRc(10.0e-6f);
        fast.processSample();
        slow.processSample();
        ok &= require(fast.voltage(2) > slow.voltage(2),
                      "capacitor value changes transient response");
        for (int i = 0; i < 2400; ++i) {
            fast.processSample();
            slow.processSample();
        }
        ok &= require(fast.voltage(2) > 0.99f && slow.voltage(2) > 0.98f,
                      "trapezoidal capacitors settle to DC");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 2200.0f;
        c.addVoltageSource(in, circuit::ground, 0.8f);
        c.addResistor(in, out, r);
        c.addDiode(out, circuit::ground, hq::component_presets::oneN4148());
        c.prepare(48000.0);
        const auto stats = c.processSample(20, 1.0e-7f);
        ok &= require(!stats.singular && stats.converged, "nonlinear diode circuit converges");
        ok &= require(c.voltage(out) > 0.45f && c.voltage(out) < 0.65f,
                      "1N4148-style diode bends transfer through series resistor");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::InductorSpec l{};
        l.inductanceHenries = 10.0e-3f;
        l.seriesResistanceOhms = 2.0f;
        hq::ResistorSpec load{};
        load.resistanceOhms = 1000.0f;
        c.addVoltageSource(in, circuit::ground, 1.0f);
        c.addInductor(in, out, l);
        c.addResistor(out, circuit::ground, load);
        c.prepare(48000.0);
        for (int i = 0; i < 2000; ++i) c.processSample();
        ok &= require(c.inductorCurrent(0) > 0.0009f && c.inductorCurrent(0) < 0.0011f,
                      "trapezoidal inductor reaches expected DC current");
    }

    return ok ? 0 : 1;
}
