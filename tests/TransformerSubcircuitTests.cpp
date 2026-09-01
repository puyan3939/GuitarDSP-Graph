#include "guitardsp/circuit/TransformerSubcircuit.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

float measureRatio(circuit::MnaCircuitEngine& engine,
                   circuit::SourceHandle drive,
                   circuit::Node input,
                   circuit::Node output,
                   int samples = 4800) {
    constexpr double sampleRate = 48000.0;
    constexpr double frequency = 1000.0;
    constexpr double twoPi = 6.28318530717958647692;
    double inputEnergy = 0.0;
    double outputEnergy = 0.0;
    int captured = 0;

    for (int i = 0; i < samples; ++i) {
        const float x = static_cast<float>(std::sin(twoPi * frequency * static_cast<double>(i) / sampleRate));
        engine.setVoltageSource(drive, x);
        const auto stats = engine.processSample(16, 1.0e-6f);
        if (stats.singular) return 0.0f;
        if (i >= samples / 2) {
            const double in = engine.voltage(input);
            const double out = engine.voltage(output);
            inputEnergy += in * in;
            outputEnergy += out * out;
            ++captured;
        }
    }

    if (captured == 0 || inputEnergy <= 0.0) return 0.0f;
    return static_cast<float>(std::sqrt(outputEnergy / inputEnergy));
}
}

int main() {
    bool ok = true;

    circuit::MnaCircuitEngine engine;
    const auto primary = engine.addNode();
    const auto secondary = engine.addNode();
    const auto drive = engine.addVoltageSource(primary, circuit::ground, 0.0f);

    hq::TransformerSpec spec{};
    spec.primaryInductanceHenries = 1.0f;
    spec.leakageInductanceHenries = 1.0e-3f;
    spec.primaryResistanceOhms = 1.0f;
    spec.secondaryResistanceOhms = 0.1f;
    spec.turnsRatio = 10.0f;
    spec.interwindingCapacitanceFarads = 0.0f;

    const auto transformer = circuit::addTransformerSubcircuit(engine,
                                                                primary,
                                                                circuit::ground,
                                                                secondary,
                                                                circuit::ground,
                                                                spec);
    hq::ResistorSpec load{};
    load.resistanceOhms = 1000.0f;
    engine.addResistor(secondary, circuit::ground, load);

    ok &= require(engine.prepare(48000.0), "transformer subcircuit prepares");
    const float ratio10 = measureRatio(engine, drive, primary, secondary);
    ok &= require(ratio10 > 0.09f && ratio10 < 0.11f,
                  "10:1 transformer subcircuit produces expected AC voltage ratio");
    ok &= require(std::isfinite(circuit::transformerSecondaryCurrent(engine, transformer)),
                  "transformer exposes secondary winding current sense");

    spec.turnsRatio = 5.0f;
    ok &= require(circuit::updateTransformerSubcircuit(engine, transformer, spec),
                  "transformer component spec updates without topology rebuild");
    engine.reset();
    const float ratio5 = measureRatio(engine, drive, primary, secondary);
    ok &= require(ratio5 > 0.18f && ratio5 < 0.22f,
                  "5:1 transformer edit changes AC voltage ratio");
    ok &= require(ratio5 > ratio10 * 1.7f,
                  "turns-ratio edit materially changes transformer response");

    return ok ? 0 : 1;
}
