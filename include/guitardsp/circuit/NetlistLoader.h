#pragma once

// Data-driven loader for the JSON circuit netlist format described in
// docs/CIRCUIT_NETLIST_FORMAT.md.
//
// A netlist document is an ordered replay script: it lists nodes and
// components in exactly the same order a hand-written guitardsp::circuit
// class (e.g. TS808Circuit, DS1Circuit) would call MnaCircuitEngine::addNode/
// addResistor/... while assembling the same physical circuit. Replaying the
// document in document order reproduces the identical MNA node/unknown
// numbering the hand-written class would produce, which is what makes
// sample-accurate parity between the two possible (see
// tests/NetlistParityTests.cpp).
//
// Real-time contract: JsonValue parsing, string handling and node-name
// lookups in this header only ever run while a circuit is being loaded and
// prepared (control thread, off the audio callback), mirroring the existing
// *Circuit::prepare() methods this format was designed to replace. Nothing
// here is called from NetlistCircuit::processSample().

#include "BjtEbersMollSubcircuit.h"
#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"
#include "JsonValue.h"
#include "MnaCircuitEngine.h"
#include "PentodeParasiticSubcircuit.h"
#include "TransformerSubcircuit.h"
#include "TriodeParasiticSubcircuit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::circuit {

// A prepared, runnable circuit built by replaying a netlist document. This is
// the data-driven counterpart to hand-written classes like TS808Circuit and
// DS1Circuit: same MnaCircuitEngine underneath, same component-level
// subcircuit helpers, same control-smoothing/DC-priming policy, just
// assembled from JSON instead of C++ source.
class NetlistCircuit {
public:
    struct ControlBinding {
        std::string pot;
        bool invert = false;
    };

    bool loadFromJson(std::string_view json, std::string* error = nullptr) {
        try {
            document_ = parseJson(json);
        } catch (const JsonParseError& e) {
            if (error != nullptr) *error = e.what();
            return false;
        }
        if (!document_.isObject()) {
            if (error != nullptr) *error = "netlist document must be a JSON object";
            return false;
        }
        loaded_ = true;
        if (error != nullptr) error->clear();
        return true;
    }

    bool loadFromFile(const std::string& path, std::string* error = nullptr) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            if (error != nullptr) *error = "unable to open netlist file: " + path;
            return false;
        }
        std::ostringstream contents;
        contents << file.rdbuf();
        return loadFromJson(contents.str(), error);
    }

    // Builds the MNA circuit from the loaded document, primes the nonlinear
    // DC operating point via source stepping, then warms it up at silence.
    // Mirrors TS808Circuit::prepare() / DS1Circuit::prepare() step for step.
    // Off the audio thread only.
    bool prepare(double sampleRate, std::string* error = nullptr) {
        if (!loaded_) {
            if (error != nullptr) *error = "no netlist document loaded";
            return false;
        }
        const auto fail = [&](const std::string& message) {
            if (error != nullptr) *error = message;
            return false;
        };

        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};
        nodes_.clear();
        pots_.clear();
        voltageSources_.clear();
        namePool_.clear();
        potInitialPosition_.clear();
        transformers_.clear();
        nodes_.emplace("ground", ground);

        const JsonValue& ops = document_["ops"];
        if (!ops.isArray()) return fail("netlist document has no 'ops' array");
        for (const JsonValue& op : ops.items()) {
            if (!applyOp(op, error)) return false;
        }

        const JsonValue& ports = document_["ports"];
        if (!resolveSource(ports, "input", inputSource_)) return fail("ports.input is missing or unknown");
        if (!resolveNode(ports, "output", outputNode_)) return fail("ports.output is missing or unknown");
        const bool hasSupply = resolveSource(ports, "supply", supplySource_);
        const bool hasVref = resolveSource(ports, "vref", vrefSource_);

        controls_.clear();
        const JsonValue& controls = document_["controls"];
        if (controls.isObject()) {
            for (const auto& [name, binding] : controls.entries()) {
                ControlBinding cb;
                cb.pot = binding["pot"].asString();
                cb.invert = binding["invert"].asBool(false);
                if (pots_.find(cb.pot) == pots_.end())
                    return fail("control '" + name + "' references unknown pot '" + cb.pot + "'");
                const auto positionIt = potInitialPosition_.find(cb.pot);
                if (positionIt == potInitialPosition_.end())
                    return fail("control '" + name + "' references unknown pot '" + cb.pot + "'");
                controls_.emplace(name, cb);
                const float value = cb.invert ? 1.0f - positionIt->second : positionIt->second;
                targetControls_[name] = value;
                appliedControls_[name] = value;
            }
        }

        readSimulationParams();

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return fail("MNA prepare failed");

        controlUpdateCountdown_ = 0;
        lastSolve_ = {};

        // A vref rail is only needed by circuits whose active devices bias
        // around a mid-supply virtual ground (TS808/DS1's transistor/op-amp
        // stages). A self-biased triode stage returns to true ground instead,
        // so priming only requires the supply rail to be present.
        if (hasSupply) {
            if (!primeOperatingPoint(hasVref)) return fail("failed to prime DC operating point");
        }

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        engine_.setNonlinearResidualTolerance(sim_.nonlinearResidualTolerance);

        const auto warmSamples = static_cast<std::size_t>(std::clamp(
            sampleRate_ * sim_.warmupSecondsFraction,
            static_cast<double>(sim_.warmupMinSamples),
            static_cast<double>(sim_.warmupMaxSamples)));
        for (std::size_t i = 0; i < warmSamples; ++i) {
            engine_.setVoltageSource(inputSource_, 0.0f);
            lastSolve_ = engine_.processSample(sim_.newtonMaxIterations, sim_.newtonTolerance);
            if (lastSolve_.singular || !allNodesFinite()) return fail("circuit diverged during warm-up");
            updateTransformerSaturation();
        }
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        controlUpdateCountdown_ = 0;
        for (auto& [name, binding] : controls_) {
            const float value = targetControls_[name];
            appliedControls_[name] = value;
            engine_.setPotentiometerPosition(pots_[binding.pot], binding.invert ? 1.0f - value : value);
        }
        for (auto& transformer : transformers_) transformer.lastMagnetizingInductanceHenries = -1.0f;
        lastSolve_ = {};
    }

    bool setControl(std::string_view name, float normalized) noexcept {
        const auto it = controls_.find(std::string(name));
        if (it == controls_.end()) return false;
        targetControls_[it->first] = std::clamp(normalized, 0.0f, 1.0f);
        return true;
    }

    float control(std::string_view name) const noexcept {
        const auto it = targetControls_.find(std::string(name));
        return it == targetControls_.end() ? 0.0f : it->second;
    }

    float appliedControl(std::string_view name) const noexcept {
        const auto it = appliedControls_.find(std::string(name));
        return it == appliedControls_.end() ? 0.0f : it->second;
    }

    float processSample(float input) noexcept {
        applySmoothedControls();
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(sim_.newtonMaxIterations, sim_.newtonTolerance);
        updateTransformerSaturation();
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    // Voltage at any named node declared by the document (op:"node" or the
    // name of a voltage-source/potentiometer terminal), for stage-by-stage
    // parity comparisons against a hand-written reference implementation.
    float nodeVoltage(std::string_view name) const noexcept {
        const auto it = nodes_.find(std::string(name));
        return it == nodes_.end() ? std::numeric_limits<float>::quiet_NaN() : engine_.voltage(it->second);
    }

    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    struct SimulationParams {
        int sourceSteps = 128;
        int solvesPerStep = 2;
        double warmupSecondsFraction = 0.08;
        std::size_t warmupMinSamples = 512;
        std::size_t warmupMaxSamples = 8192;
        float nonlinearResidualTolerance = 2.0e-5f;
        int newtonMaxIterations = 40;
        float newtonTolerance = 2.0e-5f;
        float supplyVolts = 9.0f;
        float vrefVolts = 4.5f;
    };

    // Bookkeeping for a "transformer" op's magnetizing inductance so it can
    // be re-saturated after every sample, matching PowerAmpCircuit's own
    // per-sample updateOutputTransformerSaturation().
    struct TransformerEntry {
        TransformerSubcircuit handles{};
        hq::TransformerSpec spec{};
        float lastMagnetizingInductanceHenries = -1.0f;
    };

    bool resolveNode(const JsonValue& container, const char* key, Node& out) const {
        if (!container.has(key)) return false;
        const auto it = nodes_.find(container[key].asString());
        if (it == nodes_.end()) return false;
        out = it->second;
        return true;
    }

    bool resolveSource(const JsonValue& container, const char* key, SourceHandle& out) const {
        if (!container.has(key)) return false;
        const auto it = voltageSources_.find(container[key].asString());
        if (it == voltageSources_.end()) return false;
        out = it->second;
        return true;
    }

    std::string_view intern(std::string name) {
        namePool_.push_back(std::move(name));
        return namePool_.back();
    }

    Node requireNode(const JsonValue& op, const char* key, bool& ok) {
        const auto it = nodes_.find(op[key].asString());
        if (it == nodes_.end()) { ok = false; return ground; }
        return it->second;
    }

    static hq::CapacitorTechnology parseCapacitorTechnology(const std::string& s) noexcept {
        if (s == "ceramic") return hq::CapacitorTechnology::ceramic;
        if (s == "electrolytic") return hq::CapacitorTechnology::electrolytic;
        if (s == "tantalum") return hq::CapacitorTechnology::tantalum;
        if (s == "generic") return hq::CapacitorTechnology::generic;
        return hq::CapacitorTechnology::film;
    }

    static hq::PotTaper parsePotTaper(const std::string& s) noexcept {
        if (s == "reverseAudio") return hq::PotTaper::reverseAudio;
        if (s == "linear") return hq::PotTaper::linear;
        return hq::PotTaper::audio;
    }

    static hq::DiodeTechnology parseDiodeTechnology(const std::string& s) noexcept {
        if (s == "germanium") return hq::DiodeTechnology::germanium;
        if (s == "led") return hq::DiodeTechnology::led;
        return hq::DiodeTechnology::silicon;
    }

    static hq::TransistorPolarity parsePolarity(const std::string& s) noexcept {
        if (s == "pnp") return hq::TransistorPolarity::pnp;
        if (s == "nChannel") return hq::TransistorPolarity::nChannel;
        if (s == "pChannel") return hq::TransistorPolarity::pChannel;
        return hq::TransistorPolarity::npn;
    }

    // Standard tolerance/rating metadata for pedal-grade discrete components.
    // Matches the private helpers in TS808Circuit.h/DS1Circuit.h exactly; only
    // the primary electrical value(s) need to be supplied by a netlist.
    static hq::ResistorSpec resistorSpec(float ohms) noexcept {
        hq::ResistorSpec r{};
        r.resistanceOhms = std::max(1.0e-3f, ohms);
        r.tolerancePercent = 5.0f;
        r.powerRatingWatts = 0.25f;
        return r;
    }
    static hq::CapacitorSpec capacitorSpec(float farads, float volts,
                                          hq::CapacitorTechnology technology) noexcept {
        hq::CapacitorSpec c{};
        c.capacitanceFarads = std::max(0.0f, farads);
        c.tolerancePercent = technology == hq::CapacitorTechnology::electrolytic ? 20.0f : 10.0f;
        c.voltageRatingVolts = std::max(1.0f, volts);
        c.esrOhms = technology == hq::CapacitorTechnology::electrolytic ? 0.5f : 0.03f;
        c.leakageResistanceOhms = technology == hq::CapacitorTechnology::electrolytic ? 5.0e6f : 1.0e9f;
        c.dielectricAbsorption = 0.0f;
        c.technology = technology;
        return c;
    }
    static hq::PotentiometerSpec potSpec(float ohms, hq::PotTaper taper, float position) noexcept {
        hq::PotentiometerSpec p{};
        p.totalResistanceOhms = ohms;
        p.tolerancePercent = 20.0f;
        p.powerRatingWatts = 0.25f;
        p.taper = taper;
        p.position = std::clamp(position, 0.0f, 1.0f);
        return p;
    }

    hq::DiodeSpec diodeSpecFrom(const JsonValue& op) {
        hq::DiodeSpec spec = op.has("preset") ? presetDiode(op["preset"].asString()) : hq::DiodeSpec{};
        const JsonValue& overrides = op.has("spec") ? op["spec"] : op["overrides"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("technology")) spec.technology = parseDiodeTechnology(overrides["technology"].asString());
        if (overrides.has("nominalForwardVoltage")) spec.nominalForwardVoltage = overrides["nominalForwardVoltage"].asFloat();
        if (overrides.has("saturationCurrent")) spec.saturationCurrent = overrides["saturationCurrent"].asFloat();
        if (overrides.has("emissionCoefficient")) spec.emissionCoefficient = overrides["emissionCoefficient"].asFloat();
        if (overrides.has("thermalVoltage")) spec.thermalVoltage = overrides["thermalVoltage"].asFloat();
        if (overrides.has("seriesResistanceOhms")) spec.seriesResistanceOhms = overrides["seriesResistanceOhms"].asFloat();
        if (overrides.has("junctionCapacitanceFarads")) spec.junctionCapacitanceFarads = overrides["junctionCapacitanceFarads"].asFloat();
        if (overrides.has("reverseVoltageRating")) spec.reverseVoltageRating = overrides["reverseVoltageRating"].asFloat();
        if (overrides.has("currentRatingAmps")) spec.currentRatingAmps = overrides["currentRatingAmps"].asFloat();
        return spec;
    }

    hq::BJTSpec bjtSpecFrom(const JsonValue& op) {
        hq::BJTSpec spec = op.has("preset") ? presetBjt(op["preset"].asString()) : hq::BJTSpec{};
        const JsonValue& overrides = op.has("spec") ? op["spec"] : op["overrides"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("polarity")) spec.polarity = parsePolarity(overrides["polarity"].asString());
        if (overrides.has("beta")) spec.beta = overrides["beta"].asFloat();
        if (overrides.has("nominalVbe")) spec.nominalVbe = overrides["nominalVbe"].asFloat();
        if (overrides.has("saturationVoltage")) spec.saturationVoltage = overrides["saturationVoltage"].asFloat();
        if (overrides.has("thermalVoltage")) spec.thermalVoltage = overrides["thermalVoltage"].asFloat();
        if (overrides.has("maxCollectorVoltage")) spec.maxCollectorVoltage = overrides["maxCollectorVoltage"].asFloat();
        if (overrides.has("maxCollectorCurrentAmps")) spec.maxCollectorCurrentAmps = overrides["maxCollectorCurrentAmps"].asFloat();
        if (overrides.has("inputCapacitanceFarads")) spec.inputCapacitanceFarads = overrides["inputCapacitanceFarads"].asFloat();
        return spec;
    }

    hq::OpAmpSpec opAmpSpecFrom(const JsonValue& op) {
        hq::OpAmpSpec spec = op.has("preset") ? presetOpAmp(op["preset"].asString()) : hq::OpAmpSpec{};
        const JsonValue& overrides = op.has("spec") ? op["spec"] : op["overrides"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("openLoopGainDb")) spec.openLoopGainDb = overrides["openLoopGainDb"].asFloat();
        if (overrides.has("gainBandwidthHz")) spec.gainBandwidthHz = overrides["gainBandwidthHz"].asFloat();
        if (overrides.has("slewRateVoltsPerSecond")) spec.slewRateVoltsPerSecond = overrides["slewRateVoltsPerSecond"].asFloat();
        if (overrides.has("inputBiasCurrentAmps")) spec.inputBiasCurrentAmps = overrides["inputBiasCurrentAmps"].asFloat();
        if (overrides.has("inputOffsetVoltage")) spec.inputOffsetVoltage = overrides["inputOffsetVoltage"].asFloat();
        if (overrides.has("inputNoiseVoltsPerRootHz")) spec.inputNoiseVoltsPerRootHz = overrides["inputNoiseVoltsPerRootHz"].asFloat();
        if (overrides.has("outputCurrentLimitAmps")) spec.outputCurrentLimitAmps = overrides["outputCurrentLimitAmps"].asFloat();
        if (overrides.has("positiveRailHeadroomVolts")) spec.positiveRailHeadroomVolts = overrides["positiveRailHeadroomVolts"].asFloat();
        if (overrides.has("negativeRailHeadroomVolts")) spec.negativeRailHeadroomVolts = overrides["negativeRailHeadroomVolts"].asFloat();
        if (overrides.has("outputResistanceOhms")) spec.outputResistanceOhms = overrides["outputResistanceOhms"].asFloat();
        return spec;
    }

    hq::TriodeSpec triodeSpecFrom(const JsonValue& op) {
        hq::TriodeSpec spec = op.has("preset") ? presetTriode(op["preset"].asString()) : hq::TriodeSpec{};
        const JsonValue& overrides = op.has("spec") ? op["spec"] : op["overrides"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("heaterVoltage")) spec.heaterVoltage = overrides["heaterVoltage"].asFloat();
        if (overrides.has("nominalPlateVoltage")) spec.nominalPlateVoltage = overrides["nominalPlateVoltage"].asFloat();
        if (overrides.has("maxPlateVoltage")) spec.maxPlateVoltage = overrides["maxPlateVoltage"].asFloat();
        if (overrides.has("maxPlateDissipationWatts")) spec.maxPlateDissipationWatts = overrides["maxPlateDissipationWatts"].asFloat();
        if (overrides.has("gridPlateCapacitanceFarads")) spec.gridPlateCapacitanceFarads = overrides["gridPlateCapacitanceFarads"].asFloat();
        if (overrides.has("gridCathodeCapacitanceFarads")) spec.gridCathodeCapacitanceFarads = overrides["gridCathodeCapacitanceFarads"].asFloat();
        if (overrides.has("plateCathodeCapacitanceFarads")) spec.plateCathodeCapacitanceFarads = overrides["plateCathodeCapacitanceFarads"].asFloat();
        if (overrides.has("gridCurrentSaturationAmps")) spec.gridCurrentSaturationAmps = overrides["gridCurrentSaturationAmps"].asFloat();
        if (overrides.has("gridCurrentEmissionCoefficient")) spec.gridCurrentEmissionCoefficient = overrides["gridCurrentEmissionCoefficient"].asFloat();
        return spec;
    }

    hq::PentodeSpec pentodeSpecFrom(const JsonValue& op) {
        hq::PentodeSpec spec = op.has("preset") ? presetPentode(op["preset"].asString()) : hq::PentodeSpec{};
        const JsonValue& overrides = op.has("spec") ? op["spec"] : op["overrides"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("heaterVoltage")) spec.heaterVoltage = overrides["heaterVoltage"].asFloat();
        if (overrides.has("nominalPlateVoltage")) spec.nominalPlateVoltage = overrides["nominalPlateVoltage"].asFloat();
        if (overrides.has("maxPlateVoltage")) spec.maxPlateVoltage = overrides["maxPlateVoltage"].asFloat();
        if (overrides.has("nominalScreenVoltage")) spec.nominalScreenVoltage = overrides["nominalScreenVoltage"].asFloat();
        if (overrides.has("maxScreenVoltage")) spec.maxScreenVoltage = overrides["maxScreenVoltage"].asFloat();
        if (overrides.has("maxPlateDissipationWatts")) spec.maxPlateDissipationWatts = overrides["maxPlateDissipationWatts"].asFloat();
        if (overrides.has("maxScreenDissipationWatts")) spec.maxScreenDissipationWatts = overrides["maxScreenDissipationWatts"].asFloat();
        if (overrides.has("gridPlateCapacitanceFarads")) spec.gridPlateCapacitanceFarads = overrides["gridPlateCapacitanceFarads"].asFloat();
        if (overrides.has("gridCathodeCapacitanceFarads")) spec.gridCathodeCapacitanceFarads = overrides["gridCathodeCapacitanceFarads"].asFloat();
        if (overrides.has("plateCathodeCapacitanceFarads")) spec.plateCathodeCapacitanceFarads = overrides["plateCathodeCapacitanceFarads"].asFloat();
        if (overrides.has("screenCathodeCapacitanceFarads")) spec.screenCathodeCapacitanceFarads = overrides["screenCathodeCapacitanceFarads"].asFloat();
        if (overrides.has("gridCurrentSaturationAmps")) spec.gridCurrentSaturationAmps = overrides["gridCurrentSaturationAmps"].asFloat();
        if (overrides.has("gridCurrentEmissionCoefficient")) spec.gridCurrentEmissionCoefficient = overrides["gridCurrentEmissionCoefficient"].asFloat();
        return spec;
    }

    // Transformers have no catalog preset today (PowerAmpCircuit builds its
    // output transformer's hq::TransformerSpec inline, not from
    // component_presets), so a netlist always supplies every field via
    // "spec" rather than the preset+overrides pattern used elsewhere.
    hq::TransformerSpec transformerSpecFrom(const JsonValue& op) {
        hq::TransformerSpec spec{};
        const JsonValue& overrides = op["spec"];
        if (overrides.has("name")) spec.name = intern(overrides["name"].asString());
        if (overrides.has("primaryInductanceHenries")) spec.primaryInductanceHenries = overrides["primaryInductanceHenries"].asFloat();
        if (overrides.has("leakageInductanceHenries")) spec.leakageInductanceHenries = overrides["leakageInductanceHenries"].asFloat();
        if (overrides.has("primaryResistanceOhms")) spec.primaryResistanceOhms = overrides["primaryResistanceOhms"].asFloat();
        if (overrides.has("secondaryResistanceOhms")) spec.secondaryResistanceOhms = overrides["secondaryResistanceOhms"].asFloat();
        if (overrides.has("turnsRatio")) spec.turnsRatio = overrides["turnsRatio"].asFloat();
        if (overrides.has("interwindingCapacitanceFarads")) spec.interwindingCapacitanceFarads = overrides["interwindingCapacitanceFarads"].asFloat();
        if (overrides.has("saturationFluxNormalized")) spec.saturationFluxNormalized = overrides["saturationFluxNormalized"].asFloat();
        if (overrides.has("magnetizingSaturationCurrentAmps")) spec.magnetizingSaturationCurrentAmps = overrides["magnetizingSaturationCurrentAmps"].asFloat();
        if (overrides.has("coreSaturationExponent")) spec.coreSaturationExponent = overrides["coreSaturationExponent"].asFloat();
        if (overrides.has("minimumMagnetizingInductanceRatio")) spec.minimumMagnetizingInductanceRatio = overrides["minimumMagnetizingInductanceRatio"].asFloat();
        return spec;
    }

    static hq::DiodeSpec presetDiode(const std::string& name) noexcept {
        if (name == "1n34a") return hq::component_presets::oneN34A();
        if (name == "redLed") return hq::component_presets::redLed();
        return hq::component_presets::oneN4148();
    }
    static hq::BJTSpec presetBjt(const std::string& name) noexcept {
        if (name == "2n5088") return hq::component_presets::twoN5088();
        return hq::component_presets::twoN3904();
    }
    static hq::OpAmpSpec presetOpAmp(const std::string& name) noexcept {
        if (name == "tl072") return hq::component_presets::tl072();
        return hq::component_presets::jrc4558();
    }
    static hq::TriodeSpec presetTriode(const std::string& name) noexcept {
        if (name == "12at7") return hq::component_presets::twelveAT7();
        return hq::component_presets::twelveAX7();
    }
    static hq::PentodeSpec presetPentode(const std::string& name) noexcept {
        if (name == "6l6gc") return hq::component_presets::pentodeSixL6GC();
        if (name == "kt88") return hq::component_presets::pentodeKt88();
        return hq::component_presets::pentodeEl34();
    }

    bool applyOp(const JsonValue& op, std::string* error) {
        const auto fail = [&](const std::string& message) {
            if (error != nullptr) *error = message;
            return false;
        };
        const std::string kind = op["op"].asString();
        bool ok = true;

        if (kind == "node") {
            const std::string name = op["name"].asString();
            if (name.empty()) return fail("'node' op requires a name");
            nodes_.emplace(name, engine_.addNode());
            return true;
        }
        if (kind == "voltageSource") {
            const Node p = requireNode(op, "p", ok);
            const Node n = requireNode(op, "n", ok);
            if (!ok) return fail("voltageSource references unknown node");
            const auto handle = engine_.addVoltageSource(p, n, op["volts"].asFloat());
            const std::string name = op["name"].asString();
            if (!name.empty()) voltageSources_.emplace(name, handle);
            return true;
        }
        if (kind == "resistor") {
            const Node a = requireNode(op, "a", ok);
            const Node b = requireNode(op, "b", ok);
            if (!ok) return fail("resistor references unknown node");
            engine_.addResistor(a, b, resistorSpec(op["ohms"].asFloat()));
            return true;
        }
        if (kind == "capacitor") {
            const Node a = requireNode(op, "a", ok);
            const Node b = requireNode(op, "b", ok);
            if (!ok) return fail("capacitor references unknown node");
            const auto technology = parseCapacitorTechnology(op["technology"].asString());
            engine_.addCapacitor(a, b,
                capacitorSpec(op["farads"].asFloat(), op["voltageRating"].asFloat(50.0f), technology));
            return true;
        }
        if (kind == "potentiometer") {
            const Node high = requireNode(op, "high", ok);
            const Node wiper = requireNode(op, "wiper", ok);
            const Node low = requireNode(op, "low", ok);
            if (!ok) return fail("potentiometer references unknown node");
            const auto taper = parsePotTaper(op["taper"].asString());
            const float position = op["position"].asFloat(0.5f);
            const auto handle = engine_.addPotentiometer(high, wiper, low,
                potSpec(op["ohms"].asFloat(), taper, position));
            const std::string name = op["name"].asString();
            if (name.empty()) return fail("'potentiometer' op requires a name");
            pots_.emplace(name, handle);
            potInitialPosition_.emplace(name, std::clamp(position, 0.0f, 1.0f));
            if (nodes_.find(name) == nodes_.end()) {
                // Allow controls / stage-voltage lookups by pot name via its wiper.
                nodes_.emplace(name, wiper);
            }
            return true;
        }
        if (kind == "opAmp") {
            const Node output = requireNode(op, "output", ok);
            const Node nonInv = requireNode(op, "nonInverting", ok);
            const Node inv = requireNode(op, "inverting", ok);
            const Node ref = requireNode(op, "reference", ok);
            if (!ok) return fail("opAmp references unknown node");
            engine_.addOpAmp(output, nonInv, inv, ref, opAmpSpecFrom(op));
            return true;
        }
        if (kind == "dynamicOpAmp") {
            const Node output = requireNode(op, "output", ok);
            const Node nonInv = requireNode(op, "nonInverting", ok);
            const Node inv = requireNode(op, "inverting", ok);
            const Node posRail = requireNode(op, "positiveRail", ok);
            const Node negRail = requireNode(op, "negativeRail", ok);
            const Node ref = requireNode(op, "reference", ok);
            if (!ok) return fail("dynamicOpAmp references unknown node");
            addDynamicOpAmpSubcircuit(engine_, output, nonInv, inv, posRail, negRail, ref, opAmpSpecFrom(op));
            return true;
        }
        if (kind == "bjtEbersMoll") {
            const Node collector = requireNode(op, "collector", ok);
            const Node base = requireNode(op, "base", ok);
            const Node emitter = requireNode(op, "emitter", ok);
            if (!ok) return fail("bjtEbersMoll references unknown node");
            addBjtEbersMollSubcircuit(engine_, collector, base, emitter, bjtSpecFrom(op));
            return true;
        }
        if (kind == "diodeParasitic") {
            const Node anode = requireNode(op, "anode", ok);
            const Node cathode = requireNode(op, "cathode", ok);
            if (!ok) return fail("diodeParasitic references unknown node");
            addDiodeParasiticSubcircuit(engine_, anode, cathode, diodeSpecFrom(op));
            return true;
        }
        if (kind == "triodeParasitic") {
            const Node plate = requireNode(op, "plate", ok);
            const Node grid = requireNode(op, "grid", ok);
            const Node cathode = requireNode(op, "cathode", ok);
            if (!ok) return fail("triodeParasitic references unknown node");
            addTriodeParasiticSubcircuit(engine_, plate, grid, cathode, triodeSpecFrom(op));
            return true;
        }
        if (kind == "pentodeParasitic") {
            const Node plate = requireNode(op, "plate", ok);
            const Node grid = requireNode(op, "grid", ok);
            const Node screen = requireNode(op, "screen", ok);
            const Node cathode = requireNode(op, "cathode", ok);
            if (!ok) return fail("pentodeParasitic references unknown node");
            addPentodeParasiticSubcircuit(engine_, plate, grid, screen, cathode, pentodeSpecFrom(op));
            return true;
        }
        if (kind == "transformer") {
            const Node primaryPositive = requireNode(op, "primaryPositive", ok);
            const Node primaryNegative = requireNode(op, "primaryNegative", ok);
            const Node secondaryPositive = requireNode(op, "secondaryPositive", ok);
            const Node secondaryNegative = requireNode(op, "secondaryNegative", ok);
            if (!ok) return fail("transformer references unknown node");
            TransformerEntry entry;
            entry.spec = transformerSpecFrom(op);
            entry.handles = addTransformerSubcircuit(engine_, primaryPositive, primaryNegative,
                                                       secondaryPositive, secondaryNegative, entry.spec);
            transformers_.push_back(entry);
            return true;
        }
        return fail("unknown netlist op '" + kind + "'");
    }

    void readSimulationParams() {
        sim_ = SimulationParams{};
        const JsonValue& sim = document_["simulation"];
        if (!sim.isObject()) return;
        if (sim.has("sourceSteps")) sim_.sourceSteps = sim["sourceSteps"].asInt(sim_.sourceSteps);
        if (sim.has("solvesPerStep")) sim_.solvesPerStep = sim["solvesPerStep"].asInt(sim_.solvesPerStep);
        if (sim.has("warmupSecondsFraction")) sim_.warmupSecondsFraction = sim["warmupSecondsFraction"].asNumber(sim_.warmupSecondsFraction);
        if (sim.has("warmupMinSamples")) sim_.warmupMinSamples = static_cast<std::size_t>(sim["warmupMinSamples"].asInt(static_cast<int>(sim_.warmupMinSamples)));
        if (sim.has("warmupMaxSamples")) sim_.warmupMaxSamples = static_cast<std::size_t>(sim["warmupMaxSamples"].asInt(static_cast<int>(sim_.warmupMaxSamples)));
        if (sim.has("nonlinearResidualTolerance")) sim_.nonlinearResidualTolerance = sim["nonlinearResidualTolerance"].asFloat(sim_.nonlinearResidualTolerance);
        if (sim.has("newtonMaxIterations")) sim_.newtonMaxIterations = sim["newtonMaxIterations"].asInt(sim_.newtonMaxIterations);
        if (sim.has("newtonTolerance")) sim_.newtonTolerance = sim["newtonTolerance"].asFloat(sim_.newtonTolerance);
        if (sim.has("supplyVolts")) sim_.supplyVolts = sim["supplyVolts"].asFloat(sim_.supplyVolts);
        if (sim.has("vrefVolts")) sim_.vrefVolts = sim["vrefVolts"].asFloat(sim_.vrefVolts);
    }

    bool primeOperatingPoint(bool hasVref) noexcept {
        for (int step = 1; step <= sim_.sourceSteps; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(sim_.sourceSteps);
            engine_.setVoltageSource(supplySource_, sim_.supplyVolts * t);
            if (hasVref) engine_.setVoltageSource(vrefSource_, sim_.vrefVolts * t);
            engine_.setVoltageSource(inputSource_, 0.0f);
            for (int settle = 0; settle < sim_.solvesPerStep; ++settle) {
                lastSolve_ = engine_.processSample(40, 1.0e-6f);
                if (lastSolve_.singular || !allNodesFinite()) return false;
            }
        }
        return true;
    }

    // Mirrors PowerAmpCircuit::updateOutputTransformerSaturation(): only push
    // an updated magnetizing inductance through when it has moved by more
    // than float noise, since MnaCircuitEngine::setInductorSpec()
    // unconditionally dirties the static matrix cache and forces a full
    // rebuild on the next solve (see MnaCircuitEngineCore::
    // rebuildStaticCache()). Committing on every sample -- even at idle,
    // where the magnetizing current barely moves -- would force that rebuild
    // every single sample for no audible benefit.
    void updateTransformerSaturation() noexcept {
        for (auto& transformer : transformers_) {
            const float current = engine_.inductorCurrent(
                static_cast<std::size_t>(transformer.handles.magnetizing));
            const float inductance = detail::saturatedMagnetizingInductance(transformer.spec, current);
            const float threshold = std::max(1.0e-9f, std::abs(transformer.lastMagnetizingInductanceHenries) * 1.0e-4f);
            if (std::abs(inductance - transformer.lastMagnetizingInductanceHenries) > threshold) {
                engine_.setInductorSpec(transformer.handles.magnetizing,
                                        detail::magnetizingSpec(transformer.spec, inductance));
                transformer.lastMagnetizingInductanceHenries = inductance;
            }
        }
    }

    bool allNodesFinite() const noexcept {
        for (const auto& [name, node] : nodes_) {
            if (!std::isfinite(engine_.voltage(node))) return false;
        }
        return true;
    }

    void applySmoothedControls() noexcept {
        bool anyPending = false;
        for (const auto& [name, target] : targetControls_) {
            if (appliedControls_[name] != target) { anyPending = true; break; }
        }
        if (!anyPending) {
            controlUpdateCountdown_ = 0;
            return;
        }
        if (controlUpdateCountdown_ > 0) {
            --controlUpdateCountdown_;
            return;
        }

        // Potentiometers are control signals, not oversampled audio sources.
        // Update at no less than 24 kHz while retaining a 5 ms physical ramp,
        // matching TS808Circuit/DS1Circuit's applySmoothedControls() exactly.
        const int updateInterval = std::max(1, static_cast<int>(sampleRate_ / 24000.0));
        const float maximumStep = static_cast<float>(updateInterval) /
            static_cast<float>(std::max(1.0, sampleRate_ * 0.005));
        controlUpdateCountdown_ = updateInterval - 1;

        for (auto& [name, applied] : appliedControls_) {
            const float target = targetControls_[name];
            const float next = applied + std::clamp(target - applied, -maximumStep, maximumStep);
            if (next != applied) {
                applied = next;
                const auto& binding = controls_.at(name);
                engine_.setPotentiometerPosition(pots_.at(binding.pot),
                    binding.invert ? 1.0f - applied : applied);
            }
        }
    }

    JsonValue document_;
    bool loaded_ = false;
    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;
    SimulationParams sim_{};
    std::unordered_map<std::string, Node> nodes_;
    std::unordered_map<std::string, PotHandle> pots_;
    std::unordered_map<std::string, float> potInitialPosition_;
    std::unordered_map<std::string, SourceHandle> voltageSources_;
    std::vector<TransformerEntry> transformers_;
    std::unordered_map<std::string, ControlBinding> controls_;
    std::unordered_map<std::string, float> targetControls_;
    std::unordered_map<std::string, float> appliedControls_;
    std::deque<std::string> namePool_;
    SourceHandle inputSource_{};
    SourceHandle supplySource_{};
    SourceHandle vrefSource_{};
    Node outputNode_ = ground;
    MnaCircuitEngine::SolveStats lastSolve_{};
    int controlUpdateCountdown_ = 0;
};

} // namespace guitardsp::circuit
