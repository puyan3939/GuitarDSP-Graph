#include "guitardsp/circuit/PowerAmpCircuit.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/hq/Measurement.h"
#include "guitardsp/hq/PowerAmpCircuitNode.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

double rms(const std::vector<float>& x, std::size_t start) {
    double e = 0.0;
    std::size_t count = 0;
    for (std::size_t i = start; i < x.size(); ++i) {
        e += static_cast<double>(x[i]) * x[i];
        ++count;
    }
    return count > 0 ? std::sqrt(e / static_cast<double>(count)) : 0.0;
}
} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;
    constexpr double pi = 3.14159265358979323846;

    // 1. DC operating point: a self-biased single-ended EL34 stage must settle
    // to a physically plausible, nonsingular loaded operating point (grid near
    // the grid-leak's 0 V reference, plate/screen comfortably below the 420 V
    // B+ rail, cathode self-bias in the tens-of-volts range typical of a
    // 1.2k cathode resistor at this bias current) purely from Newton
    // continuation at high voltage, with no DC offset surviving the
    // transformer-coupled output.
    {
        circuit::PowerAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "component-level EL34 power amp prepares");
        const auto primeStats = amp.lastSolveStats();
        ok &= require(!primeStats.singular, "power amp DC operating point is nonsingular");

        const auto s = amp.stageVoltages();
        std::cout << "DIAG poweramp dc grid=" << s.grid << " plate=" << s.plate
                  << " screen=" << s.screen << " cathode=" << s.cathode
                  << " output=" << s.output << '\n';
        ok &= require(amp.engine().sparseNonlinearSolverAvailable()
                          && amp.engine().sparseNonlinearCachedLinearUnknowns() >= 13,
                      "cached linear Schur prefix retains most of the 18 power amp component unknowns");
        ok &= require(std::isfinite(s.grid) && std::isfinite(s.plate) &&
                          std::isfinite(s.screen) && std::isfinite(s.cathode) &&
                          std::isfinite(s.output),
                      "power amp DC stage voltages are finite");
        ok &= require(s.grid > -0.05f && s.grid < 0.05f,
                      "self-biased grid settles near the grid-leak's 0 V reference");
        ok &= require(s.plate > 250.0f && s.plate < circuit::PowerAmpCircuit::supplyVolts,
                      "plate settles to a loaded operating point between cutoff and B+");
        ok &= require(s.screen > 250.0f && s.screen < circuit::PowerAmpCircuit::supplyVolts,
                      "screen settles close to B+ through its dropping resistor");
        ok &= require(s.cathode > 10.0f && s.cathode < 100.0f,
                      "cathode self-bias settles to a plausible Vk for a 1.2k cathode resistor");
        ok &= require(std::abs(s.output) < 0.01f,
                      "transformer-coupled output carries no DC offset at silence");
    }

    // 2. Silence must not jitter or self-oscillate. This exercises the same
    // Newton-solver robustness (backtracking line search) that PreampCircuit/
    // TS808Circuit/DS1Circuit rely on, now at the higher B+ voltages and with
    // the four-terminal pentode stamp plus a transformer subcircuit in the
    // loop.
    {
        circuit::PowerAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "power amp prepares for the silence test");

        amp.engine().resetPerformanceStats();
        constexpr int silentSamples = 16384;
        constexpr int tailStart = silentSamples - 4096;
        constexpr int tailHalf = (silentSamples - tailStart) / 2;
        int totalIterations = 0;
        int unconverged = 0;
        float maxTailAbsOutput = 0.0f;
        double tailFirstHalfSumSquares = 0.0;
        double tailSecondHalfSumSquares = 0.0;
        for (int i = 0; i < silentSamples; ++i) {
            const float output = amp.processSample(0.0f);
            const auto stats = amp.lastSolveStats();
            totalIterations += stats.iterations;
            unconverged += stats.converged ? 0 : 1;
            ok &= std::isfinite(output) && !stats.singular;
            if (i >= tailStart) {
                maxTailAbsOutput = std::max(maxTailAbsOutput, std::abs(output));
                const double squared = static_cast<double>(output) * output;
                if (i < tailStart + tailHalf) tailFirstHalfSumSquares += squared;
                else tailSecondHalfSumSquares += squared;
            }
        }
        const float averageIterations =
            static_cast<float>(totalIterations) / static_cast<float>(silentSamples);
        const auto performance = amp.engine().performanceStats();
        const double tailFirstHalfRms = std::sqrt(tailFirstHalfSumSquares / tailHalf);
        const double tailSecondHalfRms = std::sqrt(tailSecondHalfSumSquares / tailHalf);
        std::cout << "DIAG silent-poweramp average_iterations=" << averageIterations
                  << " unconverged=" << unconverged << " tail_max_abs_output=" << maxTailAbsOutput
                  << " tail_rms_first_half=" << tailFirstHalfRms
                  << " tail_rms_second_half=" << tailSecondHalfRms
                  << " static_rebuilds=" << performance.staticCacheRebuilds << '\n';
        ok &= require(averageIterations < 1.25f && unconverged == 0,
                      "prolonged silence converges without Newton limit cycles");
        ok &= require(performance.staticCacheRebuilds == 0,
                      "prolonged silence preserves the cached physical operating point");
        ok &= require(maxTailAbsOutput < 5.0e-3f,
                      "the settled tail of a silent input decays close to zero");
        ok &= require(tailSecondHalfRms <= tailFirstHalfRms * 1.5 + 1.0e-6,
                      "the settled tail does not grow into a self-sustaining oscillation");
    }

    // 3. Guitar-amplifier-level signal: the power stage is driven by a preamp/
    // tone-stack output rather than a raw pickup, so its typical input level
    // is volts, not the tens-of-millivolts a pickup produces directly. As
    // drive rises toward the pentode's grid-conduction/plate-swing headroom,
    // harmonic content should rise in a well-behaved, monotonic way, and the
    // output must always stay finite and bounded well within the B+ scale.
    {
        circuit::PowerAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "power amp prepares for the signal-level test");

        constexpr int settleSamples = 4096;
        constexpr int measureSamples = 4096;
        constexpr double toneHz = 220.0;

        const auto measure = [&](float amplitude) {
            amp.reset();
            for (int i = 0; i < settleSamples; ++i) {
                const float x = amplitude *
                    static_cast<float>(std::sin(2.0 * pi * toneHz * i / sampleRate));
                amp.processSample(x);
            }
            std::vector<float> input(measureSamples);
            std::vector<float> output(measureSamples);
            for (int i = 0; i < measureSamples; ++i) {
                const float x = amplitude *
                    static_cast<float>(std::sin(2.0 * pi * toneHz * (settleSamples + i) / sampleRate));
                input[static_cast<std::size_t>(i)] = x;
                output[static_cast<std::size_t>(i)] = amp.processSample(x);
            }
            return std::make_pair(input, output);
        };

        const auto [quietIn, quietOut] = measure(0.3f);
        bool quietFinite = true;
        for (float y : quietOut) quietFinite &= std::isfinite(y);
        const double quietInRms = rms(quietIn, 0);
        const double quietOutRms = rms(quietOut, 0);
        const auto quietHarmonics = hq::analyzeHarmonics(quietOut, sampleRate, toneHz, 8);
        std::cout << "DIAG poweramp quiet-level in_rms=" << quietInRms
                  << " out_rms=" << quietOutRms << " thd_db=" << quietHarmonics.thdDb << '\n';
        ok &= require(quietFinite, "moderate-drive output stays finite");
        ok &= require(quietOutRms > 0.0,
                      "the stage drives a nonzero signal into the dummy speaker load");
        ok &= require(std::isfinite(quietHarmonics.thdDb), "moderate-drive harmonic metrics are finite");

        // A much hotter input pushes the pentode further into grid conduction
        // and plate-swing clipping -- exercising the nonlinear pentode/grid-
        // diode/transformer stamps together at the higher B+ voltage this
        // stage runs at.
        const auto [hotIn, hotOut] = measure(6.0f);
        (void)hotIn;
        bool hotFinite = true;
        float hotPeak = 0.0f;
        for (float y : hotOut) {
            hotFinite &= std::isfinite(y);
            hotPeak = std::max(hotPeak, std::abs(y));
        }
        const auto hotHarmonics = hq::analyzeHarmonics(hotOut, sampleRate, toneHz, 8);
        std::cout << "DIAG poweramp hot-level out_peak=" << hotPeak
                  << " thd_db=" << hotHarmonics.thdDb << '\n';
        ok &= require(hotFinite && hotPeak > 0.0f &&
                          hotPeak < static_cast<float>(circuit::PowerAmpCircuit::supplyVolts),
                      "driven output stays finite and within supply-scale bounds");
        ok &= require(hotHarmonics.thdDb > quietHarmonics.thdDb,
                      "a hotter input increases harmonic distortion (natural power-amp overdrive)");
    }

    // 4. PowerAmpCircuitNode: the AudioNode wrapper that lets the graph select
    // this circuit like PreampCircuitNode/TS808CircuitNode/DS1CircuitNode.
    // Checks the node prepares, is registered under a NodeRegistry type id,
    // exposes Drive as a UI-controllable parameter, and that the Drive
    // pre-gain audibly changes the output.
    {
        graph::NodeRegistry registry = graph::NodeRegistry::createBuiltins();
        auto registered = registry.create("amp.power_amp_circuit_hq");
        ok &= require(registered != nullptr, "NodeRegistry creates the power amp circuit node");

        hq::PowerAmpCircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "power amp circuit graph node prepares");

        ok &= require(node.parameterCount() == 1, "power amp circuit node exposes Drive");
        ok &= require(node.parameterIndex("drive") == 0,
                      "power amp circuit node parameter id matches Drive");

        graph::AudioBuffer input(1, 64);
        graph::AudioBuffer output(1, 64);
        for (int i = 0; i < 64; ++i)
            input.channel(0)[i] = 0.3f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));

        node.setParameterValue(0, 0.0f); // minimum Drive pre-gain
        node.process(input, output, 64);
        bool finiteQuiet = true;
        double quietEnergy = 0.0;
        for (int i = 0; i < 64; ++i) {
            finiteQuiet &= std::isfinite(output.channel(0)[i]);
            quietEnergy += static_cast<double>(output.channel(0)[i]) * output.channel(0)[i];
        }
        ok &= require(finiteQuiet, "power amp circuit graph node processes finite audio at low Drive");

        node.reset();
        node.setParameterValue(0, 1.0f); // maximum Drive pre-gain
        node.process(input, output, 64);
        bool finiteHot = true;
        double hotEnergy = 0.0;
        for (int i = 0; i < 64; ++i) {
            finiteHot &= std::isfinite(output.channel(0)[i]);
            hotEnergy += static_cast<double>(output.channel(0)[i]) * output.channel(0)[i];
        }
        ok &= require(finiteHot, "power amp circuit graph node processes finite audio at high Drive");
        ok &= require(hotEnergy > quietEnergy,
                      "raising Drive increases the power amp circuit node's output energy");
    }

    return ok ? 0 : 1;
}
