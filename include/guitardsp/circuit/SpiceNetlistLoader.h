#pragma once

// Elaborator for the SPICE-subset netlist format (issue #99, Layer A step
// 1, second round): turns a guitardsp::circuit::SpiceNetlist (see
// SpiceNetlistParser.h) into a runnable MnaCircuitEngine circuit. This is
// the SPICE-format counterpart to NetlistLoader.h's NetlistCircuit, built
// as a third, independent implementation so tests/NetlistParityTests.cpp
// can assert it agrees with both TS808Circuit.h (hand-written C++) and
// NetlistLoader.h's NetlistCircuit (JSON) -- see data/circuits/spice/
// ts808.cir. NetlistLoader.h itself is untouched; the two formats exist in
// parallel, per the issue's explicit instruction not to modify the existing
// JSON netlist path.
//
// Port convention (this SPICE subset has no `ports`/`controls` object the
// way the JSON format does, so it uses reserved element/node names
// instead):
//   - A voltage source named exactly "VIN" is the audio input port.
//   - A voltage source named exactly "VSUPPLY" (optional) is the supply
//     rail, primed by source-stepping at prepare() time -- see
//     primeOperatingPoint() below, which mirrors
//     NetlistLoader.h's NetlistCircuit::primeOperatingPoint() exactly.
//   - A voltage source named exactly "VREF" (optional) is the mid-supply
//     virtual-ground rail for circuits whose active devices bias around it
//     (matches TS808Circuit/DS1Circuit's own vref rail).
//   - A node named exactly "OUT" is the audio output port.
// Matching is case-insensitive (names are canonicalized the same way node
// names are).
//
// Node/component creation order: this elaborator creates every *named* node
// in two passes -- first a scan of every element card's node references, in
// file order, creating an engine node the first time each name is seen;
// only then does a second pass walk the cards again, in the same file
// order, adding the actual resistors/capacitors/devices/macros. This is
// the SPICE-format analogue of how NetlistLoader.h's NetlistCircuit builds
// a JSON document (all "node" ops run before any component op).
//
// Unlike bit-for-bit reproduction of MnaCircuitEngine's internal node/
// unknown *numbering*, matching that exactly is not required for parity:
// MNA's solve is a function of circuit topology (which named nodes each
// component connects to) and component values, not of which arbitrary
// integer index a name happens to resolve to internally -- Kirchhoff's laws
// don't care about numbering. tests/NetlistParityTests.cpp's existing
// tolerance for every hand-written-vs-JSON pair is 5e-4 (see its `compare()`
// helper), which comfortably absorbs the sub-ULP floating-point
// non-associativity a different stamp order can introduce. So ts808.cir's
// card order only needs to reproduce the *same circuit topology and
// component values* as data/circuits/ts808.json, not the same literal ops
// order -- see the comment above each pot pair below for the one place
// where the *within-file* pairing of two specific cards (not the file's
// overall card order) does matter, for a different reason (recognizing a
// potentiometer, not node numbering).
//
// Potentiometers: a POT-typed .MODEL is expected to tag exactly two 'R'
// cards that share a wiper node (one "high-to-wiper", one "wiper-to-low").
// This elaborator does *not* stamp those two cards as independent
// resistors -- it recognizes the pair and makes one
// MnaCircuitEngine::addPotentiometer() call instead, using the pair's
// shared .PARAM name, the .MODEL's OHMS/TAPER/INVERT, and this engine's own
// guitardsp::hq::PotentiometerSpec::normalizedElectricalPosition() (invoked
// by MnaCircuitEngineCore.h's stamp, not re-derived here) for the taper
// curve. The two cards' own `{...}` arithmetic is therefore never used for
// stamping -- it exists so a real SPICE tool reading ts808.cir sees a
// physically reasonable (if taper-inaccurate) linear two-resistor
// approximation instead of two nonstandard, undocumented component values.
// See SpiceNetlistParser.h's header comment for why TAPER/OHMS/INVERT live
// on a .MODEL rather than being re-derived from that arithmetic.
//
// Real-time contract: loadFromFile/loadFromSpice/prepare() (and setParam(),
// which only ever touches a handful of already-allocated handles) are
// control-thread-only, exactly like NetlistCircuit's equivalents. Nothing
// here runs from processSample()'s audio-callback path except
// processSample() itself.

#include "BjtEbersMollSubcircuit.h"
#include "DiodeParasiticSubcircuit.h"
#include "DynamicOpAmpSubcircuit.h"
#include "MnaCircuitEngine.h"
#include "SpiceNetlistParser.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::circuit {

class SpiceCircuit {
public:
    bool loadFromSpice(std::string_view text, std::string* error = nullptr) {
        try {
            netlist_ = parseSpiceNetlist(text);
        } catch (const SpiceParseError& e) {
            if (error != nullptr) *error = e.what();
            return false;
        }
        loaded_ = true;
        if (error != nullptr) error->clear();
        return true;
    }

    bool loadFromFile(const std::string& path, std::string* error = nullptr) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            if (error != nullptr) *error = "unable to open SPICE netlist file: " + path;
            return false;
        }
        std::ostringstream contents;
        contents << file.rdbuf();
        return loadFromSpice(contents.str(), error);
    }

    // Builds the MNA circuit from the loaded document, primes the nonlinear
    // DC operating point via source stepping, then warms it up at silence.
    // Mirrors NetlistCircuit::prepare() / TS808Circuit::prepare() step for
    // step. Off the audio thread only.
    bool prepare(double sampleRate, std::string* error = nullptr) {
        if (!loaded_) {
            if (error != nullptr) *error = "no SPICE netlist loaded";
            return false;
        }
        const auto fail = [&](const std::string& message) {
            if (error != nullptr) *error = message;
            return false;
        };

        sampleRate_ = std::max(1.0, sampleRate);
        engine_ = MnaCircuitEngine{};
        nodes_.clear();
        nodes_.emplace("0", ground);
        voltageSources_.clear();
        potPairInfos_.clear();
        potPairByIndex_.clear();
        potBindings_.clear();
        paramToPotBindings_.clear();
        resistorBindings_.clear();
        paramToResistorBindings_.clear();
        capacitorBindings_.clear();
        paramToCapacitorBindings_.clear();
        targetParams_.clear();
        rawParams_.clear();
        hasInput_ = hasSupply_ = hasVref_ = false;
        paramUpdateCountdown_ = 0;
        lastSolve_ = {};

        for (const auto& [name, value] : netlist_.params) {
            targetParams_[name] = value;
            rawParams_[name] = value;
        }

        std::string localError;
        if (!buildPotPairs(&localError)) return fail(localError);

        for (const auto& el : netlist_.elements) {
            for (const auto& n : el.nodes) {
                if (nodes_.find(n) == nodes_.end()) nodes_.emplace(n, engine_.addNode());
            }
        }

        std::vector<bool> consumed(netlist_.elements.size(), false);
        for (std::size_t i = 0; i < netlist_.elements.size(); ++i) {
            if (consumed[i]) continue;
            if (!applyElement(i, consumed, &localError)) return fail(localError);
        }

        if (!hasInput_) return fail("no voltage source named 'VIN' (audio input port) found");
        const auto outputIt = nodes_.find("OUT");
        if (outputIt == nodes_.end()) return fail("no node named 'OUT' (audio output port) found");
        outputNode_ = outputIt->second;

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::denseReference);
        if (!engine_.prepare(sampleRate_)) return fail("MNA prepare failed");

        if (hasSupply_) {
            if (!primeOperatingPoint()) return fail("failed to prime DC operating point");
        }

        engine_.setNonlinearSolverMode(MnaCircuitEngine::NonlinearSolverMode::automatic);
        engine_.setNonlinearResidualTolerance(kNonlinearResidualTolerance);
        return true;
    }

    void reset() noexcept {
        engine_.reset();
        paramUpdateCountdown_ = 0;
        for (auto& [name, target] : targetParams_) {
            rawParams_[name] = target;
            pushParamUpdate(name, target);
        }
        lastSolve_ = {};
    }

    // Sets a .PARAM's target value (0..1, clamped); the actual engine
    // update happens gradually inside processSample() via
    // applySmoothedParams(), exactly mirroring NetlistCircuit::setControl()
    // / applySmoothedControls()'s 24kHz-update/5ms-ramp policy -- a knob
    // move must not be able to jump a potentiometer's conductance in a
    // single sample.
    bool setParam(std::string_view name, float value) noexcept {
        const std::string key = spice_detail::toUpperCopy(name);
        const auto it = targetParams_.find(key);
        if (it == targetParams_.end()) return false;
        it->second = std::clamp(value, 0.0f, 1.0f);
        return true;
    }

    float param(std::string_view name) const noexcept {
        const auto it = targetParams_.find(spice_detail::toUpperCopy(name));
        return it == targetParams_.end() ? 0.0f : static_cast<float>(it->second);
    }

    float appliedParam(std::string_view name) const noexcept {
        const auto it = rawParams_.find(spice_detail::toUpperCopy(name));
        return it == rawParams_.end() ? 0.0f : static_cast<float>(it->second);
    }

    float processSample(float input) noexcept {
        applySmoothedParams();
        engine_.setVoltageSource(inputSource_, input);
        lastSolve_ = engine_.processSample(kNewtonMaxIterations, kNewtonTolerance);
        const float out = engine_.voltage(outputNode_);
        if (lastSolve_.singular || !std::isfinite(out)) return 0.0f;
        return out;
    }

    // Voltage at any named node (for stage-by-stage parity comparisons
    // against TS808Circuit.h / NetlistCircuit).
    float nodeVoltage(std::string_view name) const noexcept {
        const auto it = nodes_.find(spice_detail::canonicalNode(name));
        return it == nodes_.end() ? std::numeric_limits<float>::quiet_NaN() : engine_.voltage(it->second);
    }

    MnaCircuitEngine::SolveStats lastSolveStats() const noexcept { return lastSolve_; }
    const MnaCircuitEngine& engine() const noexcept { return engine_; }
    MnaCircuitEngine& engine() noexcept { return engine_; }

private:
    // This SPICE subset has no analysis-card equivalent of the JSON
    // format's "simulation" object (out of scope, see SpiceNetlistParser.h
    // and issue #99); these mirror data/circuits/ts808.json's own
    // "simulation" block / TS808Circuit.h's hardcoded equivalents exactly,
    // since ts808.cir is required to be numerically equivalent to both.
    static constexpr int kSourceSteps = 128;
    static constexpr int kSolvesPerStep = 2;
    static constexpr float kSupplyVolts = 9.0f;
    static constexpr float kVrefVolts = 4.5f;
    static constexpr float kNonlinearResidualTolerance = 2.0e-5f;
    static constexpr int kNewtonMaxIterations = 80;
    static constexpr float kNewtonTolerance = 2.0e-5f;

    struct PotPairInfo {
        std::size_t highWiperIndex = 0;
        std::size_t wiperLowIndex = 0;
        std::string paramName;
        std::string modelName;
        std::string high, wiper, low;
    };
    struct PotBinding {
        PotHandle pot{};
        std::string paramName;
        bool invert = false;
    };
    struct ResistorBinding {
        ResistorHandle handle{};
        SpiceExprPtr expr;
    };
    struct CapacitorBinding {
        CapacitorHandle handle{};
        SpiceExprPtr expr;
        hq::CapacitorSpec baseSpec;
    };

    Node nodeOf(const std::string& name) const { return nodes_.at(name); }

    static hq::ResistorSpec resistorSpecFor(double ohms) noexcept {
        hq::ResistorSpec r{};
        r.resistanceOhms = std::max(1.0e-3f, static_cast<float>(ohms));
        r.tolerancePercent = 5.0f;
        r.powerRatingWatts = 0.25f;
        return r;
    }

    // Every CapacitorSpec field besides capacitanceFarads/leakageResistanceOhms
    // is unused by MnaCircuitEngineCore's stamp (see SpiceNetlistParser.h's
    // "CAP" .MODEL doc comment); the rest are set to plausible generic
    // defaults purely for documentation purposes.
    hq::CapacitorSpec capacitorSpecFor(double farads, const std::string& modelName, bool& ok, std::string* error) {
        ok = true;
        hq::CapacitorSpec c{};
        c.capacitanceFarads = std::max(0.0f, static_cast<float>(farads));
        c.tolerancePercent = 10.0f;
        c.voltageRatingVolts = 50.0f;
        c.esrOhms = 0.03f;
        c.leakageResistanceOhms = 1.0e9f;
        c.dielectricAbsorption = 0.0f;
        c.technology = hq::CapacitorTechnology::generic;
        if (!modelName.empty()) {
            const auto it = netlist_.models.find(modelName);
            if (it == netlist_.models.end()) {
                if (error != nullptr) *error = "capacitor references unknown model '" + modelName + "'";
                ok = false;
                return c;
            }
            if (it->second.type != "CAP") {
                if (error != nullptr) *error = "capacitor references model '" + modelName + "' which is not a CAP model";
                ok = false;
                return c;
            }
            const auto leak = it->second.params.find("LEAKAGEOHMS");
            if (leak != it->second.params.end()) c.leakageResistanceOhms = static_cast<float>(leak->second);
        }
        return c;
    }

    static hq::DiodeSpec diodeSpecFromModel(const SpiceModel& model) noexcept {
        hq::DiodeSpec s{};
        const auto getf = [&](const char* key, float& field) {
            const auto it = model.params.find(key);
            if (it != model.params.end()) field = static_cast<float>(it->second);
        };
        getf("NOMINALFORWARDVOLTAGE", s.nominalForwardVoltage);
        getf("SATURATIONCURRENT", s.saturationCurrent);
        getf("EMISSIONCOEFFICIENT", s.emissionCoefficient);
        getf("THERMALVOLTAGE", s.thermalVoltage);
        getf("SERIESRESISTANCEOHMS", s.seriesResistanceOhms);
        getf("JUNCTIONCAPACITANCEFARADS", s.junctionCapacitanceFarads);
        getf("REVERSEVOLTAGERATING", s.reverseVoltageRating);
        getf("CURRENTRATINGAMPS", s.currentRatingAmps);
        return s;
    }

    static hq::BJTSpec bjtSpecFromModel(const SpiceModel& model) noexcept {
        hq::BJTSpec s{};
        s.polarity = (model.type == "PNP") ? hq::TransistorPolarity::pnp : hq::TransistorPolarity::npn;
        const auto getf = [&](const char* key, float& field) {
            const auto it = model.params.find(key);
            if (it != model.params.end()) field = static_cast<float>(it->second);
        };
        getf("BETA", s.beta);
        getf("NOMINALVBE", s.nominalVbe);
        getf("SATURATIONVOLTAGE", s.saturationVoltage);
        getf("THERMALVOLTAGE", s.thermalVoltage);
        getf("MAXCOLLECTORVOLTAGE", s.maxCollectorVoltage);
        getf("MAXCOLLECTORCURRENTAMPS", s.maxCollectorCurrentAmps);
        getf("INPUTCAPACITANCEFARADS", s.inputCapacitanceFarads);
        return s;
    }

    static hq::OpAmpSpec opAmpSpecFromModel(const SpiceModel& model) noexcept {
        hq::OpAmpSpec s{};
        const auto getf = [&](const char* key, float& field) {
            const auto it = model.params.find(key);
            if (it != model.params.end()) field = static_cast<float>(it->second);
        };
        getf("OPENLOOPGAINDB", s.openLoopGainDb);
        getf("GAINBANDWIDTHHZ", s.gainBandwidthHz);
        getf("SLEWRATEVOLTSPERSECOND", s.slewRateVoltsPerSecond);
        getf("INPUTBIASCURRENTAMPS", s.inputBiasCurrentAmps);
        getf("INPUTOFFSETVOLTAGE", s.inputOffsetVoltage);
        getf("INPUTNOISEVOLTSPERROOTHZ", s.inputNoiseVoltsPerRootHz);
        getf("OUTPUTCURRENTLIMITAMPS", s.outputCurrentLimitAmps);
        getf("POSITIVERAILHEADROOMVOLTS", s.positiveRailHeadroomVolts);
        getf("NEGATIVERAILHEADROOMVOLTS", s.negativeRailHeadroomVolts);
        getf("OUTPUTRESISTANCEOHMS", s.outputResistanceOhms);
        return s;
    }

    static bool parseTaperFromModel(const SpiceModel& model, hq::PotTaper& taper, std::string* error) {
        const auto it = model.stringParams.find("TAPER");
        if (it == model.stringParams.end()) {
            if (error != nullptr) *error = "POT model '" + model.name + "' is missing the required TAPER parameter";
            return false;
        }
        if (it->second == "AUDIO") { taper = hq::PotTaper::audio; return true; }
        if (it->second == "LINEAR") { taper = hq::PotTaper::linear; return true; }
        if (it->second == "REVERSEAUDIO") { taper = hq::PotTaper::reverseAudio; return true; }
        if (error != nullptr) *error = "POT model '" + model.name + "' has unknown TAPER value '" + it->second + "'";
        return false;
    }

    // Finds every 'R' card tagged with a POT-typed .MODEL, groups them by
    // model name, and validates that exactly two share that model and a
    // wiper node (one high-to-wiper, one wiper-to-low). Also validates that
    // both cards' `{...}` expressions reference exactly one, shared .PARAM
    // name -- see this file's top-of-header comment for why that
    // expression is documentary only and not used for stamping.
    bool buildPotPairs(std::string* error) {
        const auto fail = [&](const std::string& message) { if (error != nullptr) *error = message; return false; };
        std::unordered_map<std::string, std::vector<std::size_t>> byModel;
        for (std::size_t i = 0; i < netlist_.elements.size(); ++i) {
            const SpiceElement& el = netlist_.elements[i];
            if (el.kind != 'R' || el.modelName.empty()) continue;
            const auto mit = netlist_.models.find(el.modelName);
            if (mit == netlist_.models.end())
                return fail("resistor '" + el.name + "' references unknown model '" + el.modelName + "'");
            if (mit->second.type != "POT")
                return fail("resistor '" + el.name + "' references model '" + el.modelName + "' which is not a POT model");
            byModel[el.modelName].push_back(i);
        }
        for (auto& [modelName, idxs] : byModel) {
            if (idxs.size() != 2) {
                return fail("POT model '" + modelName + "' must be referenced by exactly two resistor cards "
                            "(high-to-wiper and wiper-to-low), found " + std::to_string(idxs.size()));
            }
            std::size_t a = idxs[0];
            std::size_t b = idxs[1];
            const SpiceElement* ea = &netlist_.elements[a];
            const SpiceElement* eb = &netlist_.elements[b];
            if (ea->nodes[1] != eb->nodes[0]) {
                if (eb->nodes[1] == ea->nodes[0]) {
                    std::swap(a, b);
                    ea = &netlist_.elements[a];
                    eb = &netlist_.elements[b];
                } else {
                    return fail("POT model '" + modelName + "'s two resistor cards don't share a wiper node "
                                "(expected one high-to-wiper and one wiper-to-low card)");
                }
            }
            std::vector<std::string> paramsA, paramsB;
            if (!ea->values.empty() && ea->values[0].isExpression()) collectSpiceExprParams(ea->values[0].expr, paramsA);
            if (!eb->values.empty() && eb->values[0].isExpression()) collectSpiceExprParams(eb->values[0].expr, paramsB);
            if (paramsA.size() != 1 || paramsB.size() != 1 || paramsA[0] != paramsB[0]) {
                return fail("POT model '" + modelName + "'s two resistor-card expressions must each reference "
                            "exactly one .PARAM, and it must be the same one on both cards");
            }
            PotPairInfo info;
            info.highWiperIndex = a;
            info.wiperLowIndex = b;
            info.paramName = paramsA[0];
            info.modelName = modelName;
            info.high = ea->nodes[0];
            info.wiper = ea->nodes[1];
            info.low = eb->nodes[1];
            potPairInfos_.push_back(info);
            const std::size_t pairIndex = potPairInfos_.size() - 1;
            potPairByIndex_[a] = pairIndex;
            potPairByIndex_[b] = pairIndex;
        }
        return true;
    }

    bool applyElement(std::size_t index, std::vector<bool>& consumed, std::string* error) {
        const auto fail = [&](const std::string& message) { if (error != nullptr) *error = message; return false; };
        const SpiceElement& el = netlist_.elements[index];

        switch (el.kind) {
            case 'R': {
                if (!el.modelName.empty()) {
                    const auto pit = potPairByIndex_.find(index);
                    if (pit == potPairByIndex_.end()) return fail("internal error resolving POT pair for '" + el.name + "'");
                    const PotPairInfo& info = potPairInfos_[pit->second];
                    const SpiceModel& model = netlist_.models.at(info.modelName);
                    hq::PotTaper taper{};
                    std::string taperError;
                    if (!parseTaperFromModel(model, taper, &taperError)) return fail(taperError);
                    const auto invertIt = model.params.find("INVERT");
                    const bool invert = invertIt != model.params.end() && invertIt->second != 0.0;
                    const auto ohmsIt = model.params.find("OHMS");
                    if (ohmsIt == model.params.end())
                        return fail("POT model '" + info.modelName + "' is missing the required OHMS parameter");
                    const double raw = rawParams_.count(info.paramName) ? rawParams_.at(info.paramName) : 0.0;
                    const float position = invert ? 1.0f - static_cast<float>(raw) : static_cast<float>(raw);

                    hq::PotentiometerSpec spec{};
                    spec.totalResistanceOhms = static_cast<float>(ohmsIt->second);
                    spec.tolerancePercent = 20.0f;
                    spec.powerRatingWatts = 0.25f;
                    spec.taper = taper;
                    spec.position = std::clamp(position, 0.0f, 1.0f);

                    const PotHandle handle = engine_.addPotentiometer(nodeOf(info.high), nodeOf(info.wiper), nodeOf(info.low), spec);
                    potBindings_.push_back({handle, info.paramName, invert});
                    paramToPotBindings_[info.paramName].push_back(potBindings_.size() - 1);
                    consumed[info.highWiperIndex] = true;
                    consumed[info.wiperLowIndex] = true;
                } else {
                    const double ohms = el.values[0].evaluate(rawParams_);
                    const ResistorHandle handle = engine_.addResistor(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), resistorSpecFor(ohms));
                    if (el.values[0].isExpression()) {
                        resistorBindings_.push_back({handle, el.values[0].expr});
                        std::vector<std::string> deps;
                        collectSpiceExprParams(el.values[0].expr, deps);
                        for (const auto& d : deps) paramToResistorBindings_[d].push_back(resistorBindings_.size() - 1);
                    }
                    consumed[index] = true;
                }
                return true;
            }
            case 'L': {
                if (!el.modelName.empty()) return fail("inductor model references are not supported by this elaborator (not needed for TS808)");
                return fail("inductor cards are not supported by this elaborator (not needed for TS808)");
            }
            case 'C': {
                bool ok = true;
                std::string capError;
                const hq::CapacitorSpec base = capacitorSpecFor(0.0, el.modelName, ok, &capError);
                if (!ok) return fail(capError);
                hq::CapacitorSpec spec = base;
                spec.capacitanceFarads = std::max(0.0f, static_cast<float>(el.values[0].evaluate(rawParams_)));
                const CapacitorHandle handle = engine_.addCapacitor(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), spec);
                if (el.values[0].isExpression()) {
                    capacitorBindings_.push_back({handle, el.values[0].expr, base});
                    std::vector<std::string> deps;
                    collectSpiceExprParams(el.values[0].expr, deps);
                    for (const auto& d : deps) paramToCapacitorBindings_[d].push_back(capacitorBindings_.size() - 1);
                }
                consumed[index] = true;
                return true;
            }
            case 'D': {
                const auto mit = netlist_.models.find(el.modelName);
                if (mit == netlist_.models.end()) return fail("diode '" + el.name + "' references unknown model '" + el.modelName + "'");
                if (mit->second.type != "D") return fail("diode '" + el.name + "' references model '" + el.modelName + "' which is not a D model");
                addDiodeParasiticSubcircuit(engine_, nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), diodeSpecFromModel(mit->second));
                consumed[index] = true;
                return true;
            }
            case 'Q': {
                const auto mit = netlist_.models.find(el.modelName);
                if (mit == netlist_.models.end()) return fail("transistor '" + el.name + "' references unknown model '" + el.modelName + "'");
                if (mit->second.type != "NPN" && mit->second.type != "PNP")
                    return fail("transistor '" + el.name + "' references model '" + el.modelName + "' which is not an NPN/PNP model");
                addBjtEbersMollSubcircuit(engine_, nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), nodeOf(el.nodes[2]), bjtSpecFromModel(mit->second));
                consumed[index] = true;
                return true;
            }
            case 'V': {
                const float volts = static_cast<float>(el.values[0].evaluate(rawParams_));
                const SourceHandle handle = engine_.addVoltageSource(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), volts);
                const std::string upperName = spice_detail::toUpperCopy(el.name);
                voltageSources_[upperName] = handle;
                if (upperName == "VIN") { inputSource_ = handle; hasInput_ = true; }
                else if (upperName == "VSUPPLY") { supplySource_ = handle; hasSupply_ = true; }
                else if (upperName == "VREF") { vrefSource_ = handle; hasVref_ = true; }
                consumed[index] = true;
                return true;
            }
            case 'I': {
                const float amps = static_cast<float>(el.values[0].evaluate(rawParams_));
                engine_.addCurrentSource(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), amps);
                consumed[index] = true;
                return true;
            }
            case 'E': {
                const float gain = static_cast<float>(el.values[0].evaluate(rawParams_));
                engine_.addVcvs(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), nodeOf(el.nodes[2]), nodeOf(el.nodes[3]), gain);
                consumed[index] = true;
                return true;
            }
            case 'X': {
                const auto mit = netlist_.models.find(el.modelName);
                if (mit == netlist_.models.end()) return fail("'" + el.name + "' references unknown model '" + el.modelName + "'");
                if (mit->second.type != "OPAMP") return fail("'" + el.name + "' references model '" + el.modelName + "' which is not an OPAMP model");
                const hq::OpAmpSpec spec = opAmpSpecFromModel(mit->second);
                if (el.nodes.size() == 6) {
                    addDynamicOpAmpSubcircuit(engine_, nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), nodeOf(el.nodes[2]),
                                               nodeOf(el.nodes[3]), nodeOf(el.nodes[4]), nodeOf(el.nodes[5]), spec);
                } else if (el.nodes.size() == 4) {
                    engine_.addOpAmp(nodeOf(el.nodes[0]), nodeOf(el.nodes[1]), nodeOf(el.nodes[2]), nodeOf(el.nodes[3]), spec);
                } else {
                    return fail("'" + el.name + "' OPAMP X card must have exactly 4 nodes (ideal op-amp: "
                                "output,nonInverting,inverting,reference) or 6 nodes (dynamic op-amp macro: "
                                "output,nonInverting,inverting,positiveRail,negativeRail,reference), got " +
                                std::to_string(el.nodes.size()));
                }
                consumed[index] = true;
                return true;
            }
            default:
                return fail("internal error: unhandled element kind in elaborator");
        }
    }

    // Mirrors NetlistCircuit::primeOperatingPoint() exactly: each step is a
    // DC operating-point solve (capacitors open, inductors shorted), not a
    // transient step, so the source-stepping homotopy converges directly to
    // the circuit's true DC equilibrium.
    bool primeOperatingPoint() noexcept {
        for (int step = 1; step <= kSourceSteps; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(kSourceSteps);
            engine_.setVoltageSource(supplySource_, kSupplyVolts * t);
            if (hasVref_) engine_.setVoltageSource(vrefSource_, kVrefVolts * t);
            engine_.setVoltageSource(inputSource_, 0.0f);
            for (int settle = 0; settle < kSolvesPerStep; ++settle) {
                lastSolve_ = engine_.solveDcOperatingPoint(40, 1.0e-6f);
                if (lastSolve_.singular || !allNodesFinite()) return false;
            }
        }
        engine_.commitOperatingPointAsSteadyState();
        return true;
    }

    bool allNodesFinite() const noexcept {
        for (const auto& [name, node] : nodes_) {
            if (!std::isfinite(engine_.voltage(node))) return false;
        }
        return true;
    }

    void pushParamUpdate(const std::string& name, double value) noexcept {
        const auto pit = paramToPotBindings_.find(name);
        if (pit != paramToPotBindings_.end()) {
            for (std::size_t idx : pit->second) {
                const PotBinding& b = potBindings_[idx];
                const float position = b.invert ? 1.0f - static_cast<float>(value) : static_cast<float>(value);
                engine_.setPotentiometerPosition(b.pot, position);
            }
        }
        const auto rit = paramToResistorBindings_.find(name);
        if (rit != paramToResistorBindings_.end()) {
            for (std::size_t idx : rit->second) {
                const ResistorBinding& b = resistorBindings_[idx];
                engine_.setResistance(b.handle, static_cast<float>(evaluateSpiceExpr(b.expr, rawParams_)));
            }
        }
        const auto cit = paramToCapacitorBindings_.find(name);
        if (cit != paramToCapacitorBindings_.end()) {
            for (std::size_t idx : cit->second) {
                const CapacitorBinding& b = capacitorBindings_[idx];
                hq::CapacitorSpec spec = b.baseSpec;
                spec.capacitanceFarads = std::max(0.0f, static_cast<float>(evaluateSpiceExpr(b.expr, rawParams_)));
                engine_.setCapacitorSpec(b.handle, spec);
            }
        }
    }

    // Mirrors NetlistCircuit::applySmoothedControls() exactly: potentiometers
    // (and any other .PARAM-dependent component) are control signals, not
    // oversampled audio sources, so they're updated at no less than 24 kHz
    // while retaining a 5 ms physical ramp, rather than snapping instantly.
    void applySmoothedParams() noexcept {
        bool anyPending = false;
        for (const auto& [name, target] : targetParams_) {
            if (rawParams_[name] != target) { anyPending = true; break; }
        }
        if (!anyPending) {
            paramUpdateCountdown_ = 0;
            return;
        }
        if (paramUpdateCountdown_ > 0) {
            --paramUpdateCountdown_;
            return;
        }

        const int updateInterval = std::max(1, static_cast<int>(sampleRate_ / 24000.0));
        const double maximumStep = static_cast<double>(updateInterval) /
            std::max(1.0, sampleRate_ * 0.005);
        paramUpdateCountdown_ = updateInterval - 1;

        for (auto& [name, applied] : rawParams_) {
            const double target = targetParams_[name];
            const double next = applied + std::clamp(target - applied, -maximumStep, maximumStep);
            if (next != applied) {
                applied = next;
                pushParamUpdate(name, applied);
            }
        }
    }

    SpiceNetlist netlist_;
    bool loaded_ = false;
    MnaCircuitEngine engine_;
    double sampleRate_ = 48000.0;

    std::unordered_map<std::string, Node> nodes_;
    std::unordered_map<std::string, SourceHandle> voltageSources_;
    SourceHandle inputSource_{};
    SourceHandle supplySource_{};
    SourceHandle vrefSource_{};
    bool hasInput_ = false;
    bool hasSupply_ = false;
    bool hasVref_ = false;
    Node outputNode_ = ground;

    std::vector<PotPairInfo> potPairInfos_;
    std::unordered_map<std::size_t, std::size_t> potPairByIndex_;
    std::vector<PotBinding> potBindings_;
    std::unordered_map<std::string, std::vector<std::size_t>> paramToPotBindings_;
    std::vector<ResistorBinding> resistorBindings_;
    std::unordered_map<std::string, std::vector<std::size_t>> paramToResistorBindings_;
    std::vector<CapacitorBinding> capacitorBindings_;
    std::unordered_map<std::string, std::vector<std::size_t>> paramToCapacitorBindings_;

    std::unordered_map<std::string, double> targetParams_;
    std::unordered_map<std::string, double> rawParams_;
    int paramUpdateCountdown_ = 0;

    MnaCircuitEngine::SolveStats lastSolve_{};
};

} // namespace guitardsp::circuit
