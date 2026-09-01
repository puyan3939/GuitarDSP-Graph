#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <cmath>
#include <iostream>

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
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        hq::ResistorSpec controlLoad{};
        controlLoad.resistanceOhms = 1000.0f;
        hq::ResistorSpec outputLoad{};
        outputLoad.resistanceOhms = 1000.0f;

        const auto controlSource = c.addVoltageSource(control, circuit::ground, 1.0f);
        c.addResistor(control, circuit::ground, controlLoad);
        c.addResistor(output, circuit::ground, outputLoad);
        const auto cccs = c.addCccs(output, circuit::ground, controlSource, 2.0f);

        ok &= require(c.prepare(48000.0), "MNA prepares CCCS with branch-current control");
        c.processSample();
        ok &= require(std::abs(c.currentThroughVoltageSource(controlSource) + 1.0e-3f) < 1.0e-7f,
                      "voltage-source branch current can be probed");
        ok &= require(std::abs(c.voltage(output) - 2.0f) < 1.0e-4f,
                      "CCCS maps control branch current into output current");

        c.setCccsGain(cccs, 0.5f);
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 0.5f) < 1.0e-4f,
                      "CCCS gain edits without topology rebuild");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        hq::ResistorSpec controlLoad{};
        controlLoad.resistanceOhms = 1000.0f;

        const auto controlSource = c.addVoltageSource(control, circuit::ground, 1.0f);
        c.addResistor(control, circuit::ground, controlLoad);
        const auto ccvs = c.addCcvs(output, circuit::ground, controlSource, -1000.0f);

        ok &= require(c.prepare(48000.0), "MNA prepares CCVS branch unknown");
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 1.0f) < 1.0e-4f,
                      "CCVS maps sensed branch current into output voltage");

        c.setCcvsTransresistance(ccvs, -2200.0f);
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 2.2f) < 1.0e-4f,
                      "CCVS transresistance edits without topology rebuild");
    }

    return ok ? 0 : 1;
}
