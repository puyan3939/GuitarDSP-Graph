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
        const auto load = c.addResistor(out, circuit::ground, r);
        ok &= require(c.prepare(48000.0), "MNA prepares resistor divider");
        const auto stats = c.processSample();
        ok &= require(!stats.singular && std::abs(c.voltage(out) - 0.5f) < 1.0e-5f,
                      "MNA resistor divider solves 0.5 V");
        c.setVoltageSource(source, -0.8f);
        c.processSample();
        ok &= require(std::abs(c.voltage(out) + 0.4f) < 1.0e-5f,
                      "MNA realtime voltage source update works");
        c.setVoltageSource(source, 1.0f);
        ok &= require(c.setResistance(load, 3000.0f),
                      "resistor handle accepts value edit without topology rebuild");
        c.processSample();
        ok &= require(std::abs(c.voltage(out) - 0.75f) < 1.0e-5f,
                      "resistor handle edit changes prepared circuit response");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto high = c.addNode();
        const auto wiper = c.addNode();
        hq::PotentiometerSpec pot{};
        pot.totalResistanceOhms = 100000.0f;
        pot.taper = hq::PotTaper::linear;
        pot.position = 0.25f;
        c.addVoltageSource(high, circuit::ground, 1.0f);
        const auto handle = c.addPotentiometer(high, wiper, circuit::ground, pot);
        ok &= require(c.prepare(48000.0), "MNA prepares three-terminal potentiometer");
        c.processSample();
        ok &= require(std::abs(c.voltage(wiper) - 0.25f) < 1.0e-4f,
                      "linear potentiometer maps mechanical position to divider voltage");
        c.setPotentiometerPosition(handle, 0.75f);
        c.processSample();
        ok &= require(std::abs(c.voltage(wiper) - 0.75f) < 1.0e-4f,
                      "potentiometer position updates without topology rebuild");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        hq::ResistorSpec load{};
        load.resistanceOhms = 1000.0f;
        c.addVoltageSource(control, circuit::ground, 1.0f);
        c.addResistor(output, circuit::ground, load);
        c.addVccs(circuit::ground, output, control, circuit::ground, 1.0e-3f);
        ok &= require(c.prepare(48000.0), "MNA prepares VCCS");
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 1.0f) < 1.0e-4f,
                      "VCCS converts control voltage into output current");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        c.addVoltageSource(control, circuit::ground, 0.2f);
        c.addVcvs(output, circuit::ground, control, circuit::ground, 5.0f);
        ok &= require(c.prepare(48000.0), "MNA prepares VCVS branch unknown");
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 1.0f) < 1.0e-4f,
                      "VCVS enforces controlled output voltage");
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
        r.resistanceOhms = 1000.0f;
        hq::CapacitorSpec cap{};
        cap.capacitanceFarads = 1.0e-6f;
        cap.leakageResistanceOhms = 1.0e12f;
        cap.esrOhms = 0.0f;
        c.addVoltageSource(in, circuit::ground, 1.0f);
        c.addResistor(in, out, r);
        const auto capHandle = c.addCapacitor(out, circuit::ground, cap);
        c.prepare(48000.0);
        c.processSample();
        const float fastFirst = c.voltage(out);
        c.reset();
        ok &= require(c.setCapacitance(capHandle, 10.0e-6f),
                      "capacitor handle accepts capacitance edit");
        c.processSample();
        ok &= require(c.voltage(out) < fastFirst,
                      "capacitance handle edit changes transient without topology rebuild");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 2200.0f;
        c.addVoltageSource(in, circuit::ground, 0.8f);
        c.addResistor(in, out, r);
        const auto diode = c.addDiode(out, circuit::ground, hq::component_presets::oneN4148());
        c.prepare(48000.0);
        const auto stats = c.processSample(24, 1.0e-7f);
        ok &= require(!stats.singular && stats.converged, "nonlinear diode circuit converges");
        const float siliconVoltage = c.voltage(out);
        ok &= require(siliconVoltage > 0.45f && siliconVoltage < 0.65f,
                      "1N4148-style diode bends transfer through series resistor");
        c.reset();
        ok &= require(c.setDiodeSpec(diode, hq::component_presets::oneN34A()),
                      "diode handle accepts device-family replacement");
        c.processSample(32, 1.0e-7f);
        ok &= require(c.voltage(out) < siliconVoltage,
                      "germanium-style diode replacement lowers clamp voltage");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vcc = c.addNode();
        const auto base = c.addNode();
        const auto collector = c.addNode();
        const auto emitter = c.addNode();
        hq::ResistorSpec rc{};
        rc.resistanceOhms = 4700.0f;
        hq::ResistorSpec re{};
        re.resistanceOhms = 680.0f;
        c.addVoltageSource(vcc, circuit::ground, 9.0f);
        c.addVoltageSource(base, circuit::ground, 0.72f);
        c.addResistor(vcc, collector, rc);
        c.addResistor(emitter, circuit::ground, re);
        const auto transistor = c.addBjt(collector, base, emitter, hq::component_presets::twoN3904());
        ok &= require(c.prepare(48000.0), "MNA prepares BJT three-terminal stamp");
        const auto stats = c.processSample(32, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "BJT nonlinear stamp converges");
        ok &= require(c.voltage(emitter) > 0.02f && c.voltage(collector) < 8.95f,
                      "BJT bias produces emitter current and collector drop");
        ok &= require(c.setBjtSpec(transistor, hq::component_presets::twoN5088()),
                      "BJT handle accepts transistor replacement");
        c.processSample(32, 1.0e-6f);
        ok &= require(std::isfinite(c.voltage(collector)),
                      "BJT replacement remains numerically finite");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vdd = c.addNode();
        const auto drain = c.addNode();
        const auto source = c.addNode();
        hq::ResistorSpec rd{};
        rd.resistanceOhms = 10000.0f;
        hq::ResistorSpec rs{};
        rs.resistanceOhms = 1500.0f;
        c.addVoltageSource(vdd, circuit::ground, 9.0f);
        c.addResistor(vdd, drain, rd);
        c.addResistor(source, circuit::ground, rs);
        c.addJfet(drain, circuit::ground, source, hq::component_presets::j201());
        ok &= require(c.prepare(48000.0), "MNA prepares JFET common-source stamp");
        const auto stats = c.processSample(32, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "JFET nonlinear stamp converges");
        ok &= require(c.voltage(source) > 0.05f && c.voltage(drain) > 1.0f && c.voltage(drain) < 8.8f,
                      "J201-style self bias changes drain and source voltages");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vdd = c.addNode();
        const auto gate = c.addNode();
        const auto drain = c.addNode();
        const auto source = c.addNode();
        hq::ResistorSpec rd{};
        rd.resistanceOhms = 4700.0f;
        hq::ResistorSpec rs{};
        rs.resistanceOhms = 470.0f;
        c.addVoltageSource(vdd, circuit::ground, 9.0f);
        c.addVoltageSource(gate, circuit::ground, 2.8f);
        c.addResistor(vdd, drain, rd);
        c.addResistor(source, circuit::ground, rs);
        c.addMosfet(drain, gate, source, hq::component_presets::bs170());
        ok &= require(c.prepare(48000.0), "MNA prepares MOSFET three-terminal stamp");
        const auto stats = c.processSample(36, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "MOSFET nonlinear stamp converges");
        ok &= require(c.voltage(source) > 0.10f && c.voltage(drain) > 0.5f && c.voltage(drain) < 8.8f,
                      "BS170-style bias produces nonlinear drain current");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto input = c.addNode();
        const auto output = c.addNode();
        auto opAmp = hq::component_presets::jrc4558();
        opAmp.inputOffsetVoltage = 0.0f;
        c.addVoltageSource(input, circuit::ground, 0.25f);
        const auto handle = c.addOpAmp(output, input, output, circuit::ground, opAmp);
        ok &= require(c.prepare(48000.0), "MNA prepares finite-gain op-amp macro stamp");
        const auto stats = c.processSample();
        ok &= require(!stats.singular && std::abs(c.voltage(output) - 0.25f) < 1.0e-4f,
                      "high-open-loop-gain op-amp closes as unity follower");
        opAmp.openLoopGainDb = 20.0f;
        ok &= require(c.setOpAmpSpec(handle, opAmp),
                      "op-amp handle accepts macro-model edit");
        c.processSample();
        const float expected = 0.25f * 10.0f / 11.0f;
        ok &= require(std::abs(c.voltage(output) - expected) < 1.0e-4f,
                      "finite op-amp gain changes closed-loop accuracy");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto supply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 100000.0f;
        c.addVoltageSource(supply, circuit::ground, 250.0f);
        c.addVoltageSource(grid, circuit::ground, -1.2f);
        c.addResistor(supply, plate, plateLoad);
        const auto tube = c.addTriode(plate, grid, circuit::ground,
                                     hq::component_presets::twelveAX7());
        ok &= require(c.prepare(48000.0), "MNA prepares triode plate-grid-cathode stamp");
        const auto stats = c.processSample(40, 1.0e-5f);
        ok &= require(!stats.singular && stats.converged, "12AX7 nonlinear MNA stamp converges");
        const float ax7Plate = c.voltage(plate);
        ok &= require(ax7Plate > 40.0f && ax7Plate < 220.0f,
                      "12AX7 plate current establishes plausible loaded operating point");
        ok &= require(c.setTriodeSpec(tube, hq::component_presets::twelveAT7()),
                      "triode handle accepts tube-family replacement");
        c.processSample(40, 1.0e-5f);
        ok &= require(std::abs(c.voltage(plate) - ax7Plate) > 1.0f,
                      "12AT7 replacement changes the same circuit operating point");
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
        const auto inductor = c.addInductor(in, out, l);
        c.addResistor(out, circuit::ground, load);
        c.prepare(48000.0);
        for (int i = 0; i < 2000; ++i) c.processSample();
        ok &= require(c.inductorCurrent(0) > 0.0009f && c.inductorCurrent(0) < 0.0011f,
                      "trapezoidal inductor reaches expected DC current");
        ok &= require(c.setInductance(inductor, 20.0e-3f),
                      "inductor handle accepts inductance edit");
    }

    return ok ? 0 : 1;
}
