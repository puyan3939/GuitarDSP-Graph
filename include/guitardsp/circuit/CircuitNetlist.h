#pragma once

#include "DynamicOpAmpSubcircuit.h"
#include "MnaCircuitEngine.h"
#include "SwitchRelaySubcircuit.h"
#include "TransformerSubcircuit.h"
#include "TriodeParasiticSubcircuit.h"

#include <cstdint>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace guitardsp::circuit {

using CircuitNodeId = std::uint32_t;
using CircuitComponentId = std::uint32_t;
inline constexpr CircuitNodeId circuitGround = 0;

enum class CircuitComponentKind : std::uint8_t {
    resistor,
    capacitor,
    inductor,
    potentiometer,
    voltageSource,
    vccs,
    vcvs,
    cccs,
    ccvs,
    diode,
    bjt,
    jfet,
    mosfet,
    opAmp,
    dynamicOpAmp,
    triode,
    triodeParasitic,
    transformer,
    switchComponent,
    relay
};

struct CircuitComponentBinding {
    CircuitComponentKind kind = CircuitComponentKind::resistor;
    std::uint16_t handle = 0;
};

struct ResistorDefinition { CircuitComponentId id{}; CircuitNodeId a{}, b{}; hq::ResistorSpec spec{}; };
struct CapacitorDefinition { CircuitComponentId id{}; CircuitNodeId a{}, b{}; hq::CapacitorSpec spec{}; };
struct InductorDefinition { CircuitComponentId id{}; CircuitNodeId a{}, b{}; hq::InductorSpec spec{}; };
struct PotentiometerDefinition { CircuitComponentId id{}; CircuitNodeId high{}, wiper{}, low{}; hq::PotentiometerSpec spec{}; };
struct VoltageSourceDefinition { CircuitComponentId id{}; CircuitNodeId positive{}, negative{}; float volts = 0.0f; };
struct VccsDefinition { CircuitComponentId id{}; CircuitNodeId outP{}, outN{}, ctrlP{}, ctrlN{}; float transconductance = 0.0f; };
struct VcvsDefinition { CircuitComponentId id{}; CircuitNodeId outP{}, outN{}, ctrlP{}, ctrlN{}; float gain = 1.0f; };
struct CccsDefinition { CircuitComponentId id{}; CircuitNodeId outP{}, outN{}; CircuitComponentId controlVoltageSource{}; float gain = 1.0f; };
struct CcvsDefinition { CircuitComponentId id{}; CircuitNodeId outP{}, outN{}; CircuitComponentId controlVoltageSource{}; float transresistanceOhms = 1.0f; };
struct DiodeDefinition { CircuitComponentId id{}; CircuitNodeId anode{}, cathode{}; hq::DiodeSpec spec{}; };
struct BjtDefinition { CircuitComponentId id{}; CircuitNodeId collector{}, base{}, emitter{}; hq::BJTSpec spec{}; };
struct JfetDefinition { CircuitComponentId id{}; CircuitNodeId drain{}, gate{}, source{}; hq::JFETSpec spec{}; };
struct MosfetDefinition { CircuitComponentId id{}; CircuitNodeId drain{}, gate{}, source{}; hq::MOSFETSpec spec{}; };
struct OpAmpDefinition { CircuitComponentId id{}; CircuitNodeId output{}, nonInverting{}, inverting{}, reference{}; hq::OpAmpSpec spec{}; };
struct DynamicOpAmpDefinition {
    CircuitComponentId id{};
    CircuitNodeId output{}, nonInverting{}, inverting{}, positiveRail{}, negativeRail{}, reference{};
    hq::OpAmpSpec spec{};
};
struct TriodeDefinition { CircuitComponentId id{}; CircuitNodeId plate{}, grid{}, cathode{}; hq::TriodeSpec spec{}; };
struct TriodeParasiticDefinition { CircuitComponentId id{}; CircuitNodeId plate{}, grid{}, cathode{}; hq::TriodeSpec spec{}; };
struct TransformerDefinition { CircuitComponentId id{}; CircuitNodeId primaryP{}, primaryN{}, secondaryP{}, secondaryN{}; hq::TransformerSpec spec{}; };
struct SwitchDefinition {
    CircuitComponentId id{};
    CircuitNodeId common1{}, throwA1{}, throwB1{}, common2{}, throwA2{}, throwB2{};
    hq::SwitchSpec spec{};
    bool thrown = false;
};
struct RelayDefinition {
    CircuitComponentId id{};
    CircuitNodeId coilPositive{}, coilNegative{};
    CircuitNodeId common1{}, throwA1{}, throwB1{}, common2{}, throwA2{}, throwB2{};
    hq::RelaySpec spec{};
};

using CircuitComponentDefinition = std::variant<
    ResistorDefinition,
    CapacitorDefinition,
    InductorDefinition,
    PotentiometerDefinition,
    VoltageSourceDefinition,
    VccsDefinition,
    VcvsDefinition,
    CccsDefinition,
    CcvsDefinition,
    DiodeDefinition,
    BjtDefinition,
    JfetDefinition,
    MosfetDefinition,
    OpAmpDefinition,
    DynamicOpAmpDefinition,
    TriodeDefinition,
    TriodeParasiticDefinition,
    TransformerDefinition,
    SwitchDefinition,
    RelayDefinition>;

struct CompiledCircuit {
    MnaCircuitEngine engine;
    std::unordered_map<CircuitNodeId, Node> nodes;
    std::unordered_map<CircuitComponentId, CircuitComponentBinding> bindings;
    std::unordered_map<CircuitComponentId, DynamicOpAmpSubcircuit> dynamicOpAmps;
    std::unordered_map<CircuitComponentId, TriodeParasiticSubcircuit> triodeParasitics;
    std::unordered_map<CircuitComponentId, TransformerSubcircuit> transformers;
    std::unordered_map<CircuitComponentId, SwitchSubcircuit> switches;
    std::unordered_map<CircuitComponentId, RelaySubcircuit> relays;
    std::unordered_map<CircuitComponentId, RelayRuntimeState> relayStates;

    bool findNode(CircuitNodeId id, Node& node) const noexcept {
        if (id == circuitGround) {
            node = ground;
            return true;
        }
        const auto it = nodes.find(id);
        if (it == nodes.end()) return false;
        node = it->second;
        return true;
    }

    const CircuitComponentBinding* binding(CircuitComponentId id) const noexcept {
        const auto it = bindings.find(id);
        return it == bindings.end() ? nullptr : &it->second;
    }

    const TransformerSubcircuit* transformer(CircuitComponentId id) const noexcept {
        const auto it = transformers.find(id);
        return it == transformers.end() ? nullptr : &it->second;
    }

    SwitchSubcircuit* switchSubcircuit(CircuitComponentId id) noexcept {
        const auto it = switches.find(id);
        return it == switches.end() ? nullptr : &it->second;
    }

    RelaySubcircuit* relay(CircuitComponentId id) noexcept {
        const auto it = relays.find(id);
        return it == relays.end() ? nullptr : &it->second;
    }

    RelayRuntimeState* relayState(CircuitComponentId id) noexcept {
        const auto it = relayStates.find(id);
        return it == relayStates.end() ? nullptr : &it->second;
    }
};

// Control-thread representation of a schematic-like circuit. Stable node/component
// IDs are separate from solver indexes so a future UI/JSON format can retain IDs
// while the compiled MNA layout changes.
class CircuitNetlist {
public:
    CircuitNodeId addNode(std::string name = {}) {
        const CircuitNodeId id = nextNodeId_++;
        nodes_.push_back({id, std::move(name)});
        return id;
    }

    CircuitComponentId addResistor(CircuitNodeId a, CircuitNodeId b, hq::ResistorSpec spec) {
        return addDefinition(ResistorDefinition{nextComponentId_, a, b, spec});
    }
    CircuitComponentId addCapacitor(CircuitNodeId a, CircuitNodeId b, hq::CapacitorSpec spec) {
        return addDefinition(CapacitorDefinition{nextComponentId_, a, b, spec});
    }
    CircuitComponentId addInductor(CircuitNodeId a, CircuitNodeId b, hq::InductorSpec spec) {
        return addDefinition(InductorDefinition{nextComponentId_, a, b, spec});
    }
    CircuitComponentId addPotentiometer(CircuitNodeId high, CircuitNodeId wiper, CircuitNodeId low,
                                        hq::PotentiometerSpec spec) {
        return addDefinition(PotentiometerDefinition{nextComponentId_, high, wiper, low, spec});
    }
    CircuitComponentId addVoltageSource(CircuitNodeId positive, CircuitNodeId negative, float volts) {
        return addDefinition(VoltageSourceDefinition{nextComponentId_, positive, negative, volts});
    }
    CircuitComponentId addVccs(CircuitNodeId outP, CircuitNodeId outN,
                               CircuitNodeId ctrlP, CircuitNodeId ctrlN, float gm) {
        return addDefinition(VccsDefinition{nextComponentId_, outP, outN, ctrlP, ctrlN, gm});
    }
    CircuitComponentId addVcvs(CircuitNodeId outP, CircuitNodeId outN,
                               CircuitNodeId ctrlP, CircuitNodeId ctrlN, float gain) {
        return addDefinition(VcvsDefinition{nextComponentId_, outP, outN, ctrlP, ctrlN, gain});
    }
    CircuitComponentId addCccs(CircuitNodeId outP, CircuitNodeId outN,
                               CircuitComponentId controlVoltageSource, float gain) {
        return addDefinition(CccsDefinition{nextComponentId_, outP, outN, controlVoltageSource, gain});
    }
    CircuitComponentId addCcvs(CircuitNodeId outP, CircuitNodeId outN,
                               CircuitComponentId controlVoltageSource, float transresistanceOhms) {
        return addDefinition(CcvsDefinition{nextComponentId_, outP, outN, controlVoltageSource,
                                            transresistanceOhms});
    }
    CircuitComponentId addDiode(CircuitNodeId anode, CircuitNodeId cathode, hq::DiodeSpec spec) {
        return addDefinition(DiodeDefinition{nextComponentId_, anode, cathode, spec});
    }
    CircuitComponentId addBjt(CircuitNodeId collector, CircuitNodeId base, CircuitNodeId emitter,
                              hq::BJTSpec spec) {
        return addDefinition(BjtDefinition{nextComponentId_, collector, base, emitter, spec});
    }
    CircuitComponentId addJfet(CircuitNodeId drain, CircuitNodeId gate, CircuitNodeId source,
                               hq::JFETSpec spec) {
        return addDefinition(JfetDefinition{nextComponentId_, drain, gate, source, spec});
    }
    CircuitComponentId addMosfet(CircuitNodeId drain, CircuitNodeId gate, CircuitNodeId source,
                                 hq::MOSFETSpec spec) {
        return addDefinition(MosfetDefinition{nextComponentId_, drain, gate, source, spec});
    }
    CircuitComponentId addOpAmp(CircuitNodeId output, CircuitNodeId nonInverting,
                                CircuitNodeId inverting, CircuitNodeId reference, hq::OpAmpSpec spec) {
        return addDefinition(OpAmpDefinition{nextComponentId_, output, nonInverting, inverting,
                                             reference, spec});
    }
    CircuitComponentId addDynamicOpAmp(CircuitNodeId output, CircuitNodeId nonInverting,
                                       CircuitNodeId inverting, CircuitNodeId positiveRail,
                                       CircuitNodeId negativeRail, CircuitNodeId reference,
                                       hq::OpAmpSpec spec) {
        return addDefinition(DynamicOpAmpDefinition{nextComponentId_, output, nonInverting, inverting,
                                                    positiveRail, negativeRail, reference, spec});
    }
    CircuitComponentId addTriode(CircuitNodeId plate, CircuitNodeId grid, CircuitNodeId cathode,
                                 hq::TriodeSpec spec) {
        return addDefinition(TriodeDefinition{nextComponentId_, plate, grid, cathode, spec});
    }
    CircuitComponentId addTriodeParasitic(CircuitNodeId plate, CircuitNodeId grid,
                                          CircuitNodeId cathode, hq::TriodeSpec spec) {
        return addDefinition(TriodeParasiticDefinition{nextComponentId_, plate, grid, cathode, spec});
    }
    CircuitComponentId addTransformer(CircuitNodeId primaryP, CircuitNodeId primaryN,
                                      CircuitNodeId secondaryP, CircuitNodeId secondaryN,
                                      hq::TransformerSpec spec) {
        return addDefinition(TransformerDefinition{nextComponentId_, primaryP, primaryN,
                                                    secondaryP, secondaryN, spec});
    }
    CircuitComponentId addSwitch(CircuitNodeId common1, CircuitNodeId throwA1,
                                 CircuitNodeId throwB1, CircuitNodeId common2,
                                 CircuitNodeId throwA2, CircuitNodeId throwB2,
                                 hq::SwitchSpec spec, bool thrown = false) {
        return addDefinition(SwitchDefinition{nextComponentId_, common1, throwA1, throwB1,
                                              common2, throwA2, throwB2, spec, thrown});
    }
    CircuitComponentId addRelay(CircuitNodeId coilPositive, CircuitNodeId coilNegative,
                                CircuitNodeId common1, CircuitNodeId throwA1,
                                CircuitNodeId throwB1, CircuitNodeId common2,
                                CircuitNodeId throwA2, CircuitNodeId throwB2,
                                hq::RelaySpec spec) {
        return addDefinition(RelayDefinition{nextComponentId_, coilPositive, coilNegative,
                                             common1, throwA1, throwB1,
                                             common2, throwA2, throwB2, spec});
    }

    const std::vector<CircuitComponentDefinition>& components() const noexcept { return components_; }

    bool compile(double sampleRate, CompiledCircuit& output, std::string* error = nullptr) const {
        CompiledCircuit built;
        for (const auto& node : nodes_) built.nodes.emplace(node.id, built.engine.addNode());

        const auto resolve = [&](CircuitNodeId id, Node& solverNode) -> bool {
            return built.findNode(id, solverNode);
        };
        const auto fail = [&](const char* message) -> bool {
            if (error != nullptr) *error = message;
            return false;
        };

        // Independent voltage sources are compiled first because CCCS/CCVS refer
        // to their branch currents by stable component ID.
        for (const auto& component : components_) {
            if (const auto* source = std::get_if<VoltageSourceDefinition>(&component)) {
                Node p{}, n{};
                if (!resolve(source->positive, p) || !resolve(source->negative, n))
                    return fail("voltage source references missing node");
                const auto handle = built.engine.addVoltageSource(p, n, source->volts);
                built.bindings.emplace(source->id,
                    CircuitComponentBinding{CircuitComponentKind::voltageSource, handle});
            }
        }

        for (const auto& component : components_) {
            if (std::holds_alternative<VoltageSourceDefinition>(component) ||
                std::holds_alternative<CccsDefinition>(component) ||
                std::holds_alternative<CcvsDefinition>(component)) continue;

            bool componentOk = true;
            std::visit([&](const auto& definition) {
                using T = std::decay_t<decltype(definition)>;
                Node a{}, b{}, c{}, d{}, e{}, f{}, g{}, h{};
                if constexpr (std::is_same_v<T, ResistorDefinition>) {
                    componentOk = resolve(definition.a, a) && resolve(definition.b, b);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::resistor,
                            built.engine.addResistor(a, b, definition.spec)});
                } else if constexpr (std::is_same_v<T, CapacitorDefinition>) {
                    componentOk = resolve(definition.a, a) && resolve(definition.b, b);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::capacitor,
                            built.engine.addCapacitor(a, b, definition.spec)});
                } else if constexpr (std::is_same_v<T, InductorDefinition>) {
                    componentOk = resolve(definition.a, a) && resolve(definition.b, b);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::inductor,
                            built.engine.addInductor(a, b, definition.spec)});
                } else if constexpr (std::is_same_v<T, PotentiometerDefinition>) {
                    componentOk = resolve(definition.high, a) && resolve(definition.wiper, b) &&
                                  resolve(definition.low, c);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::potentiometer,
                            built.engine.addPotentiometer(a, b, c, definition.spec)});
                } else if constexpr (std::is_same_v<T, VccsDefinition>) {
                    componentOk = resolve(definition.outP, a) && resolve(definition.outN, b) &&
                                  resolve(definition.ctrlP, c) && resolve(definition.ctrlN, d);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::vccs,
                            built.engine.addVccs(a, b, c, d, definition.transconductance)});
                } else if constexpr (std::is_same_v<T, VcvsDefinition>) {
                    componentOk = resolve(definition.outP, a) && resolve(definition.outN, b) &&
                                  resolve(definition.ctrlP, c) && resolve(definition.ctrlN, d);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::vcvs,
                            built.engine.addVcvs(a, b, c, d, definition.gain)});
                } else if constexpr (std::is_same_v<T, DiodeDefinition>) {
                    componentOk = resolve(definition.anode, a) && resolve(definition.cathode, b);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::diode,
                            built.engine.addDiode(a, b, definition.spec)});
                } else if constexpr (std::is_same_v<T, BjtDefinition>) {
                    componentOk = resolve(definition.collector, a) && resolve(definition.base, b) &&
                                  resolve(definition.emitter, c);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::bjt,
                            built.engine.addBjt(a, b, c, definition.spec)});
                } else if constexpr (std::is_same_v<T, JfetDefinition>) {
                    componentOk = resolve(definition.drain, a) && resolve(definition.gate, b) &&
                                  resolve(definition.source, c);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::jfet,
                            built.engine.addJfet(a, b, c, definition.spec)});
                } else if constexpr (std::is_same_v<T, MosfetDefinition>) {
                    componentOk = resolve(definition.drain, a) && resolve(definition.gate, b) &&
                                  resolve(definition.source, c);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::mosfet,
                            built.engine.addMosfet(a, b, c, definition.spec)});
                } else if constexpr (std::is_same_v<T, OpAmpDefinition>) {
                    componentOk = resolve(definition.output, a) && resolve(definition.nonInverting, b) &&
                                  resolve(definition.inverting, c) && resolve(definition.reference, d);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::opAmp,
                            built.engine.addOpAmp(a, b, c, d, definition.spec)});
                } else if constexpr (std::is_same_v<T, DynamicOpAmpDefinition>) {
                    componentOk = resolve(definition.output, a) && resolve(definition.nonInverting, b) &&
                                  resolve(definition.inverting, c) && resolve(definition.positiveRail, d) &&
                                  resolve(definition.negativeRail, e) && resolve(definition.reference, f);
                    if (componentOk) {
                        built.dynamicOpAmps.emplace(definition.id,
                            addDynamicOpAmpSubcircuit(built.engine, a, b, c, d, e, f, definition.spec));
                        built.bindings.emplace(definition.id,
                            CircuitComponentBinding{CircuitComponentKind::dynamicOpAmp, 0});
                    }
                } else if constexpr (std::is_same_v<T, TriodeDefinition>) {
                    componentOk = resolve(definition.plate, a) && resolve(definition.grid, b) &&
                                  resolve(definition.cathode, c);
                    if (componentOk) built.bindings.emplace(definition.id,
                        CircuitComponentBinding{CircuitComponentKind::triode,
                            built.engine.addTriode(a, b, c, definition.spec)});
                } else if constexpr (std::is_same_v<T, TriodeParasiticDefinition>) {
                    componentOk = resolve(definition.plate, a) && resolve(definition.grid, b) &&
                                  resolve(definition.cathode, c);
                    if (componentOk) {
                        built.triodeParasitics.emplace(definition.id,
                            addTriodeParasiticSubcircuit(built.engine, a, b, c, definition.spec));
                        built.bindings.emplace(definition.id,
                            CircuitComponentBinding{CircuitComponentKind::triodeParasitic, 0});
                    }
                } else if constexpr (std::is_same_v<T, TransformerDefinition>) {
                    componentOk = resolve(definition.primaryP, a) && resolve(definition.primaryN, b) &&
                                  resolve(definition.secondaryP, c) && resolve(definition.secondaryN, d);
                    if (componentOk) {
                        built.transformers.emplace(definition.id,
                            addTransformerSubcircuit(built.engine, a, b, c, d, definition.spec));
                        built.bindings.emplace(definition.id,
                            CircuitComponentBinding{CircuitComponentKind::transformer, 0});
                    }
                } else if constexpr (std::is_same_v<T, SwitchDefinition>) {
                    componentOk = resolve(definition.common1, a) && resolve(definition.throwA1, b) &&
                                  resolve(definition.throwB1, c) && resolve(definition.common2, d) &&
                                  resolve(definition.throwA2, e) && resolve(definition.throwB2, f);
                    if (componentOk) {
                        built.switches.emplace(definition.id,
                            addSwitchSubcircuit(built.engine, a, b, c, d, e, f,
                                                definition.spec, definition.thrown));
                        built.bindings.emplace(definition.id,
                            CircuitComponentBinding{CircuitComponentKind::switchComponent, 0});
                    }
                } else if constexpr (std::is_same_v<T, RelayDefinition>) {
                    componentOk = resolve(definition.coilPositive, a) && resolve(definition.coilNegative, b) &&
                                  resolve(definition.common1, c) && resolve(definition.throwA1, d) &&
                                  resolve(definition.throwB1, e) && resolve(definition.common2, f) &&
                                  resolve(definition.throwA2, g) && resolve(definition.throwB2, h);
                    if (componentOk) {
                        built.relays.emplace(definition.id,
                            addRelaySubcircuit(built.engine, a, b, c, d, e, f, g, h, definition.spec));
                        built.relayStates.emplace(definition.id, RelayRuntimeState{});
                        built.bindings.emplace(definition.id,
                            CircuitComponentBinding{CircuitComponentKind::relay, 0});
                    }
                }
            }, component);
            if (!componentOk) return fail("component references missing node");
        }

        for (const auto& component : components_) {
            if (const auto* source = std::get_if<CccsDefinition>(&component)) {
                Node p{}, n{};
                if (!resolve(source->outP, p) || !resolve(source->outN, n))
                    return fail("CCCS references missing output node");
                const auto* control = built.binding(source->controlVoltageSource);
                if (control == nullptr || control->kind != CircuitComponentKind::voltageSource)
                    return fail("CCCS control must reference an independent voltage source");
                const auto handle = built.engine.addCccs(p, n,
                    static_cast<SourceHandle>(control->handle), source->gain);
                built.bindings.emplace(source->id,
                    CircuitComponentBinding{CircuitComponentKind::cccs, handle});
            } else if (const auto* source = std::get_if<CcvsDefinition>(&component)) {
                Node p{}, n{};
                if (!resolve(source->outP, p) || !resolve(source->outN, n))
                    return fail("CCVS references missing output node");
                const auto* control = built.binding(source->controlVoltageSource);
                if (control == nullptr || control->kind != CircuitComponentKind::voltageSource)
                    return fail("CCVS control must reference an independent voltage source");
                const auto handle = built.engine.addCcvs(p, n,
                    static_cast<SourceHandle>(control->handle), source->transresistanceOhms);
                built.bindings.emplace(source->id,
                    CircuitComponentBinding{CircuitComponentKind::ccvs, handle});
            }
        }

        if (!built.engine.prepare(sampleRate)) return fail("MNA prepare failed");
        output = std::move(built);
        if (error != nullptr) error->clear();
        return true;
    }

private:
    struct NodeDefinition { CircuitNodeId id{}; std::string name; };

    template <typename T>
    CircuitComponentId addDefinition(T definition) {
        const CircuitComponentId id = nextComponentId_++;
        definition.id = id;
        components_.emplace_back(std::move(definition));
        return id;
    }

    CircuitNodeId nextNodeId_ = 1;
    CircuitComponentId nextComponentId_ = 1;
    std::vector<NodeDefinition> nodes_;
    std::vector<CircuitComponentDefinition> components_;
};

} // namespace guitardsp::circuit
