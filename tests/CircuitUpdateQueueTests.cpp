#include "guitardsp/circuit/CircuitUpdateQueue.h"

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

    circuit::MnaCircuitEngine engine;
    const auto input = engine.addNode();
    const auto output = engine.addNode();
    hq::ResistorSpec resistor{};
    resistor.resistanceOhms = 1000.0f;
    engine.addVoltageSource(input, circuit::ground, 1.0f);
    engine.addResistor(input, output, resistor);
    const auto load = engine.addResistor(output, circuit::ground, resistor);
    ok &= require(engine.prepare(48000.0), "update-queue fixture prepares");
    engine.processSample();
    ok &= require(std::abs(engine.voltage(output) - 0.5f) < 1.0e-5f,
                  "update-queue fixture starts at 0.5 V");

    circuit::CircuitUpdateQueue<4> queue;
    circuit::CircuitUpdateCommand resistance{};
    resistance.kind = circuit::CircuitUpdateKind::resistance;
    resistance.handle = load;
    resistance.payload = 3000.0f;
    ok &= require(queue.tryPush(resistance), "control thread queues resistor update");
    ok &= require(queue.pendingApprox() == 1U, "queue reports pending command");
    ok &= require(circuit::applyPendingCircuitUpdates(engine, queue) == 1U,
                  "audio block boundary drains queued edit");
    engine.processSample();
    ok &= require(std::abs(engine.voltage(output) - 0.75f) < 1.0e-5f,
                  "queued resistor edit changes prepared circuit");

    circuit::CircuitUpdateCommand invalid{};
    invalid.kind = circuit::CircuitUpdateKind::capacitance;
    invalid.handle = 999U;
    invalid.payload = 10.0e-9f;
    ok &= require(queue.tryPush(invalid), "queue transports invalid-handle command without allocation");
    ok &= require(circuit::applyPendingCircuitUpdates(engine, queue) == 1U,
                  "consumer drains invalid-handle command deterministically");

    circuit::CircuitUpdateQueue<2> bounded;
    ok &= require(bounded.tryPush(resistance), "bounded queue accepts first command");
    ok &= require(bounded.tryPush(resistance), "bounded queue uses full fixed capacity");
    ok &= require(!bounded.tryPush(resistance), "bounded queue rejects overflow instead of allocating");

    return ok ? 0 : 1;
}
