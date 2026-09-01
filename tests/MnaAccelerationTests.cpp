#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec spec{};
    spec.resistanceOhms = ohms;
    return spec;
}

hq::CapacitorSpec capacitor(float farads) {
    hq::CapacitorSpec spec{};
    spec.capacitanceFarads = farads;
    spec.leakageResistanceOhms = 1.0e12f;
    spec.esrOhms = 0.0f;
    return spec;
}
}

int main() {
    bool ok = true;

    {
        circuit::MnaCircuitEngine c;
        const auto input = c.addNode();
        const auto output = c.addNode();
        const auto source = c.addVoltageSource(input, circuit::ground, 0.0f);
        c.addResistor(input, output, resistor(1000.0f));
        const auto load = c.addResistor(output, circuit::ground, resistor(2200.0f));
        c.addCapacitor(output, circuit::ground, capacitor(1.0e-6f));

        ok &= require(c.prepare(48000.0), "accelerated linear MNA prepares");
        auto initial = c.performanceStats();
        ok &= require(initial.staticCacheRebuilds == 1 && initial.fullFactorizations == 1,
                      "prepare compiles one static matrix and one LU factorization");

        c.resetPerformanceStats();
        for (int i = 0; i < 128; ++i) {
            const float volts = static_cast<float>(i) / 127.0f;
            c.setVoltageSource(source, volts);
            const auto stats = c.processSample();
            ok &= !stats.singular;
        }
        const auto steady = c.performanceStats();
        ok &= require(steady.samples == 128 && steady.staticCacheRebuilds == 0,
                      "per-sample source updates do not rebuild the matrix cache");
        ok &= require(steady.cachedLinearSolves == 128 && steady.generalLinearSolves == 0,
                      "linear samples use cached LU solves instead of Gaussian refactorization");
        ok &= require(steady.fullFactorizations == 0,
                      "stable linear circuit performs no repeated factorization");
        ok &= require(std::isfinite(c.voltage(output)) && c.voltage(output) > 0.0f,
                      "cached linear path remains numerically finite");

        c.resetPerformanceStats();
        ok &= require(c.setResistance(load, 4700.0f),
                      "matrix-affecting resistor edit is accepted");
        c.processSample();
        const auto edited = c.performanceStats();
        ok &= require(edited.staticCacheRebuilds == 1 && edited.fullFactorizations == 1,
                      "matrix-affecting edit rebuilds and refactorizes exactly once");
        c.processSample();
        const auto second = c.performanceStats();
        ok &= require(second.staticCacheRebuilds == 1 && second.fullFactorizations == 1,
                      "rebuilt linear cache is reused on following samples");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto input = c.addNode();
        const auto output = c.addNode();
        const auto source = c.addVoltageSource(input, circuit::ground, 0.7f);
        c.addResistor(input, output, resistor(2200.0f));
        c.addDiode(output, circuit::ground, hq::component_presets::oneN4148());
        ok &= require(c.prepare(48000.0), "accelerated nonlinear MNA prepares");

        c.resetPerformanceStats();
        for (int i = 0; i < 32; ++i) {
            c.setVoltageSource(source, 0.55f + 0.01f * static_cast<float>(i));
            const auto stats = c.processSample(24, 1.0e-7f);
            ok &= !stats.singular;
        }
        const auto nonlinear = c.performanceStats();
        ok &= require(nonlinear.staticCacheRebuilds == 0,
                      "nonlinear source updates preserve cached linear base matrix");
        ok &= require(nonlinear.nonlinearAssemblies >= nonlinear.samples,
                      "nonlinear path restamps only Newton-dependent device terms");
        ok &= require(nonlinear.generalLinearSolves == nonlinear.nonlinearAssemblies &&
                      nonlinear.cachedLinearSolves == 0,
                      "nonlinear Newton iterations use general solves over cached base stamps");
        ok &= require(nonlinear.fullFactorizations == 0,
                      "nonlinear circuit does not build an unusable static LU factorization");
        ok &= require(std::isfinite(c.voltage(output)),
                      "accelerated nonlinear path remains numerically finite");
    }

    return ok ? 0 : 1;
}
