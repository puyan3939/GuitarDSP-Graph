#include "guitardsp/circuit/CircuitNetlist.h"
#include "guitardsp/circuit/CircuitUpdateQueue.h"

#include <cmath>
#include <iostream>
#include <string>

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
        circuit::CircuitNetlist netlist;
        const auto input = netlist.addNode("input");
        const auto output = netlist.addNode("output");
        hq::ResistorSpec resistor{};
        resistor.resistanceOhms = 1000.0f;
        const auto sourceId = netlist.addVoltageSource(input, circuit::circuitGround, 1.0f);
        netlist.addResistor(input, output, resistor);
        const auto loadId = netlist.addResistor(output, circuit::circuitGround, resistor);

        circuit::CompiledCircuit compiled;
        std::string error;
        ok &= require(netlist.compile(48000.0, compiled, &error),
                      "stable-ID netlist compiles into prepared MNA circuit");
        compiled.engine.processSample();
        circuit::Node outputNode{};
        ok &= require(compiled.findNode(output, outputNode), "compiled circuit resolves stable node ID");
        ok &= require(std::abs(compiled.engine.voltage(outputNode) - 0.5f) < 1.0e-5f,
                      "compiled resistor-divider topology solves correctly");

        const auto* sourceBinding = compiled.binding(sourceId);
        const auto* loadBinding = compiled.binding(loadId);
        ok &= require(sourceBinding != nullptr &&
                      sourceBinding->kind == circuit::CircuitComponentKind::voltageSource,
                      "compiled netlist preserves voltage-source component binding");
        ok &= require(loadBinding != nullptr &&
                      loadBinding->kind == circuit::CircuitComponentKind::resistor,
                      "compiled netlist preserves resistor component binding");

        circuit::CircuitUpdateQueue<4> queue;
        circuit::CircuitUpdateCommand command{};
        command.kind = circuit::CircuitUpdateKind::resistance;
        command.handle = loadBinding != nullptr ? loadBinding->handle : 0U;
        command.payload = 3000.0f;
        queue.tryPush(command);
        circuit::applyPendingCircuitUpdates(compiled.engine, queue);
        compiled.engine.processSample();
        ok &= require(std::abs(compiled.engine.voltage(outputNode) - 0.75f) < 1.0e-5f,
                      "stable component ID binding drives block-boundary value edit");
    }

    {
        circuit::CircuitNetlist netlist;
        const auto primary = netlist.addNode("primary");
        const auto secondary = netlist.addNode("secondary");
        netlist.addVoltageSource(primary, circuit::circuitGround, 0.0f);
        hq::TransformerSpec transformer{};
        transformer.turnsRatio = 8.0f;
        const auto transformerId = netlist.addTransformer(primary, circuit::circuitGround,
                                                           secondary, circuit::circuitGround,
                                                           transformer);
        hq::ResistorSpec load{};
        load.resistanceOhms = 1000.0f;
        netlist.addResistor(secondary, circuit::circuitGround, load);

        circuit::CompiledCircuit compiled;
        ok &= require(netlist.compile(48000.0, compiled),
                      "netlist compiler expands transformer subcircuit");
        ok &= require(compiled.transformer(transformerId) != nullptr,
                      "compiled circuit preserves transformer subcircuit binding");
    }

    {
        circuit::CircuitNetlist netlist;
        const auto collector = netlist.addNode("collector");
        const auto base = netlist.addNode("base");
        const auto drain = netlist.addNode("drain");
        const auto gate = netlist.addNode("gate");
        const auto mosDrain = netlist.addNode("mos_drain");
        const auto mosGate = netlist.addNode("mos_gate");

        const auto bjtId = netlist.addBjt(collector, base, circuit::circuitGround,
                                         hq::component_presets::twoN3904());
        const auto jfetId = netlist.addJfet(drain, gate, circuit::circuitGround,
                                           hq::component_presets::j201());
        const auto mosfetId = netlist.addMosfet(mosDrain, mosGate, circuit::circuitGround,
                                               hq::component_presets::bs170());

        circuit::CompiledCircuit compiled;
        ok &= require(netlist.compile(48000.0, compiled),
                      "netlist compiles active devices with parasitic networks");
        ok &= require(compiled.bjtParasitic(bjtId) != nullptr,
                      "normal BJT netlist definition owns Cbe/Cbc parasitics");
        ok &= require(compiled.jfetParasitic(jfetId) != nullptr,
                      "normal JFET netlist definition owns Cgs/Cgd/Cds parasitics");
        ok &= require(compiled.mosfetParasitic(mosfetId) != nullptr,
                      "normal MOSFET netlist definition owns parasitics and body diode");
    }

    {
        circuit::CircuitNetlist invalid;
        const auto validNode = invalid.addNode();
        hq::ResistorSpec resistor{};
        invalid.addResistor(validNode, 999999U, resistor);
        circuit::CompiledCircuit compiled;
        std::string error;
        ok &= require(!invalid.compile(48000.0, compiled, &error) && !error.empty(),
                      "netlist compiler rejects missing node references");
    }

    return ok ? 0 : 1;
}
