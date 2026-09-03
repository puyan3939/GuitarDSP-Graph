#include "guitardsp/circuit/CompressorCircuit.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/hq/CompressorCircuitNode.h"

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

std::vector<float> tone(int count, float amplitude, double sampleRate, double hz, int phaseOffset = 0) {
    constexpr double pi = 3.14159265358979323846;
    std::vector<float> samples(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double t = static_cast<double>(i + phaseOffset) / sampleRate;
        samples[static_cast<std::size_t>(i)] = amplitude * static_cast<float>(std::sin(2.0 * pi * hz * t));
    }
    return samples;
}
} // namespace

int main() {
    bool ok = true;
    constexpr double sampleRate = 48000.0;

    // 1. DC operating point: the whole signal path and sidechain bias around
    // vref (an internal reference, not exposed), so every stage voltage
    // reported relative to that reference -- and the true-ground-referenced
    // output after its coupling cap -- must settle to a finite, silent (near
    // zero) operating point purely from source-stepping continuation, with
    // the LDR resting at its dark resistance (no compression at silence).
    {
        circuit::CompressorCircuit comp;
        ok &= require(comp.prepare(sampleRate), "component-level optical compressor prepares");
        const auto primeStats = comp.lastSolveStats();
        ok &= require(!primeStats.singular, "compressor DC operating point is nonsingular");

        const auto s = comp.stageVoltages();
        std::cout << "DIAG compressor dc gainCell=" << s.gainCell << " envelope=" << s.envelope
                  << " output=" << s.output << " ldrOhms=" << comp.ldrResistanceOhms() << '\n';
        ok &= require(std::isfinite(s.gainCell) && std::isfinite(s.envelope) && std::isfinite(s.output),
                      "compressor DC stage voltages are finite");
        ok &= require(std::abs(s.gainCell) < 0.05f, "gain cell settles near its vref reference");
        ok &= require(std::abs(s.envelope) < 0.05f, "sidechain envelope settles near zero at silence");
        // The output coupling cap is a physically modelled electrolytic (ESR +
        // finite leakage resistance, not an ideal DC block -- see
        // hq::CapacitorSpec), so a few mV of leakage-induced offset through the
        // 10k output-to-ground resistor is expected, not a settling bug.
        ok &= require(std::abs(s.output) < 0.02f, "output carries only leakage-scale DC offset at silence");
        ok &= require(comp.ldrResistanceOhms() > 1.0e6f,
                      "LDR rests at (near-)dark resistance with no sidechain drive");
    }

    // 2. Silence must not jitter or self-oscillate. The gain cell, sidechain
    // peak detector and LDR update form a closed feedback loop (output feeds
    // the sidechain, which sets the LDR resistance, which sets the gain
    // cell's attenuation, which sets the output), so this is the specific
    // case the issue asked to verify: a perfectly silent input must not let
    // that loop self-sustain any residual oscillation or Newton limit cycle.
    {
        circuit::CompressorCircuit comp;
        ok &= require(comp.prepare(sampleRate), "compressor prepares for the silence test");

        comp.engine().resetPerformanceStats();
        constexpr int silentSamples = 16384;
        constexpr int tailStart = silentSamples - 4096;
        constexpr int tailHalf = (silentSamples - tailStart) / 2;
        int unconverged = 0;
        float maxTailAbsOutput = 0.0f;
        double tailFirstHalfSumSquares = 0.0;
        double tailSecondHalfSumSquares = 0.0;
        const float initialLdrOhms = comp.ldrResistanceOhms();
        for (int i = 0; i < silentSamples; ++i) {
            const float output = comp.processSample(0.0f);
            const auto stats = comp.lastSolveStats();
            unconverged += stats.converged ? 0 : 1;
            ok &= std::isfinite(output) && !stats.singular;
            if (i >= tailStart) {
                maxTailAbsOutput = std::max(maxTailAbsOutput, std::abs(output));
                const double squared = static_cast<double>(output) * output;
                if (i < tailStart + tailHalf) tailFirstHalfSumSquares += squared;
                else tailSecondHalfSumSquares += squared;
            }
        }
        const auto performance = comp.engine().performanceStats();
        const double tailFirstHalfRms = std::sqrt(tailFirstHalfSumSquares / tailHalf);
        const double tailSecondHalfRms = std::sqrt(tailSecondHalfSumSquares / tailHalf);
        std::cout << "DIAG silent-compressor unconverged=" << unconverged
                  << " tail_max_abs_output=" << maxTailAbsOutput
                  << " tail_rms_first_half=" << tailFirstHalfRms
                  << " tail_rms_second_half=" << tailSecondHalfRms
                  << " static_rebuilds=" << performance.staticCacheRebuilds
                  << " ldr_ohms=" << comp.ldrResistanceOhms() << '\n';
        ok &= require(unconverged == 0, "prolonged silence converges every sample without limit cycles");
        // Same physically-modelled output-cap leakage offset as the DC test
        // above, not residual jitter -- static_rebuilds == 0 (checked below)
        // is what actually rules out oscillation here.
        ok &= require(maxTailAbsOutput < 0.02f,
                      "the settled tail of a silent input decays to a leakage-scale constant");
        ok &= require(tailSecondHalfRms <= tailFirstHalfRms * 1.5 + 1.0e-6,
                      "the settled tail does not grow into a self-sustaining oscillation");
        ok &= require(std::abs(comp.ldrResistanceOhms() - initialLdrOhms) < initialLdrOhms * 0.01f,
                      "the LDR resistance itself does not drift/jitter away from dark at silence");
        ok &= require(performance.staticCacheRebuilds == 0,
                      "a settled silent input never re-triggers the LDR's setResistance() static rebuild");
    }

    // 3. Gain reduction: a quiet input should pass through close to unity
    // (LDR resting dark), while a much louder input should be audibly
    // compressed (output/input ratio drops, and the LDR resistance drops
    // toward its lit value) -- the core LA-2A-style behaviour.
    {
        circuit::CompressorCircuit comp;
        ok &= require(comp.prepare(sampleRate), "compressor prepares for the gain-reduction test");

        constexpr int settleSamples = 8000;
        constexpr int measureSamples = 4000;
        constexpr double toneHz = 220.0;

        const auto measure = [&](float amplitude) {
            comp.reset();
            for (float x : tone(settleSamples, amplitude, sampleRate, toneHz)) comp.processSample(x);
            std::vector<float> output(measureSamples);
            const auto probe = tone(measureSamples, amplitude, sampleRate, toneHz, settleSamples);
            for (std::size_t i = 0; i < probe.size(); ++i) output[i] = comp.processSample(probe[i]);
            return output;
        };

        const float quietAmplitude = 0.02f;
        const float loudAmplitude = 1.5f;
        const auto quietOut = measure(quietAmplitude);
        const float quietLdrOhms = comp.ldrResistanceOhms();
        const auto loudOut = measure(loudAmplitude);
        const float loudLdrOhms = comp.ldrResistanceOhms();

        bool quietFinite = true, loudFinite = true;
        for (float y : quietOut) quietFinite &= std::isfinite(y);
        for (float y : loudOut) loudFinite &= std::isfinite(y);
        ok &= require(quietFinite && loudFinite, "gain-reduction sweep stays finite at both levels");

        const double quietOutRms = rms(quietOut, 0);
        const double loudOutRms = rms(loudOut, 0);
        const double quietRatio = quietOutRms / quietAmplitude;
        const double loudRatio = loudOutRms / loudAmplitude;
        std::cout << "DIAG compressor quiet_ratio=" << quietRatio << " quiet_ldr_ohms=" << quietLdrOhms
                  << " loud_ratio=" << loudRatio << " loud_ldr_ohms=" << loudLdrOhms << '\n';

        ok &= require(quietRatio > 0.5, "a quiet signal passes through close to unity gain");
        ok &= require(loudRatio < quietRatio * 0.9,
                      "a much louder signal is compressed relative to a quiet one (gain reduction)");
        ok &= require(loudLdrOhms < quietLdrOhms * 0.5,
                      "a loud signal drives the LDR resistance down substantially from its dark value");
    }

    // 4. Attack/release time constants: after a step from silence to a loud
    // tone, the LDR resistance should collapse toward its lit value quickly
    // (attack); after stepping back to silence, it should recover toward
    // dark much more slowly (release), consistent with an LA-2A-style
    // envelope follower and the ~100 ms release RC network's design intent.
    // This also exercises the concern raised in the issue: that the
    // one-sample-delayed LDR update is negligible next to these time
    // constants, since if it were not, attack/release would show up as
    // erratic rather than a single, roughly monotonic decay.
    {
        circuit::CompressorCircuit comp;
        ok &= require(comp.prepare(sampleRate), "compressor prepares for the attack/release test");

        constexpr double toneHz = 220.0;
        constexpr float amplitude = 1.5f;

        for (float x : tone(4000, 0.0f, sampleRate, toneHz)) comp.processSample(x);
        const float darkOhms = comp.ldrResistanceOhms();

        constexpr int stepSamples = 12000;
        const auto burst = tone(stepSamples, amplitude, sampleRate, toneHz, 4000);
        int attackSample = -1;
        float minOhms = darkOhms;
        float previousOhms = darkOhms;
        double maxLogStepPerSample = 0.0;
        const double fullLogSpan = std::log(static_cast<double>(darkOhms) / 500.0);
        for (int i = 0; i < stepSamples; ++i) {
            comp.processSample(burst[static_cast<std::size_t>(i)]);
            const float ohms = comp.ldrResistanceOhms();
            minOhms = std::min(minOhms, ohms);
            if (attackSample < 0 && ohms < darkOhms * 0.5f) attackSample = i;
            // How much the single-sample-delayed LDR update moves the gain
            // cell's resistance from one sample to the next, in log-resistance
            // terms (resistance is exponential in LED drive), relative to the
            // whole dark-to-light span -- quantifies the issue's concern that
            // the one-sample delay should be negligible next to the LDR's own
            // multi-millisecond attack/release time constants, not just that
            // the overall transition happens to look fast.
            maxLogStepPerSample = std::max(maxLogStepPerSample,
                std::abs(std::log(static_cast<double>(ohms)) - std::log(static_cast<double>(previousOhms))));
            previousOhms = ohms;
        }
        std::cout << "DIAG compressor max_single_sample_log_step_fraction="
                  << (maxLogStepPerSample / fullLogSpan) << '\n';
        ok &= require(maxLogStepPerSample / fullLogSpan < 0.05,
                      "the one-sample-delayed LDR update never jumps more than ~5% of the "
                      "full dark-to-light range in a single sample");
        const double attackMs = attackSample >= 0
            ? static_cast<double>(attackSample) * 1000.0 / sampleRate : -1.0;

        constexpr int releaseSamples = 24000;
        const auto silence = tone(releaseSamples, 0.0f, sampleRate, toneHz, 4000 + stepSamples);
        int releaseSample = -1;
        const float releaseTarget = minOhms + (darkOhms - minOhms) * 0.5f;
        for (int i = 0; i < releaseSamples; ++i) {
            comp.processSample(silence[static_cast<std::size_t>(i)]);
            if (releaseSample < 0 && comp.ldrResistanceOhms() > releaseTarget) releaseSample = i;
        }
        const double releaseMs = releaseSample >= 0
            ? static_cast<double>(releaseSample) * 1000.0 / sampleRate : -1.0;

        std::cout << "DIAG compressor dark_ohms=" << darkOhms << " min_ohms=" << minOhms
                  << " attack_ms=" << attackMs << " release_ms=" << releaseMs << '\n';

        ok &= require(attackSample >= 0 && attackMs < 50.0,
                      "gain reduction engages within tens of milliseconds (attack)");
        ok &= require(releaseSample >= 0 && releaseMs > attackMs * 2.0 && releaseMs < 500.0,
                      "recovery to dark is slower than attack and within a plausible release window");
    }

    // 5. CompressorCircuitNode: the AudioNode wrapper that lets the graph
    // select this circuit, mirroring PowerAmpCircuitNode/TS808CircuitNode.
    {
        graph::NodeRegistry registry = graph::NodeRegistry::createBuiltins();
        auto registered = registry.create("dynamics.compressor_circuit_hq");
        ok &= require(registered != nullptr, "NodeRegistry creates the compressor circuit node");

        hq::CompressorCircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "compressor circuit graph node prepares");

        ok &= require(node.parameterCount() == 1, "compressor circuit node exposes Makeup Gain");
        ok &= require(node.parameterIndex("makeup_gain") == 0,
                      "compressor circuit node parameter id matches Makeup Gain");

        constexpr double pi = 3.14159265358979323846;
        graph::AudioBuffer input(1, 64);
        graph::AudioBuffer output(1, 64);
        for (int i = 0; i < 64; ++i)
            input.channel(0)[i] = 0.2f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));

        node.setParameterValue(0, 0.0f); // minimum makeup gain
        node.process(input, output, 64);
        bool finiteLow = true;
        double lowEnergy = 0.0;
        for (int i = 0; i < 64; ++i) {
            finiteLow &= std::isfinite(output.channel(0)[i]);
            lowEnergy += static_cast<double>(output.channel(0)[i]) * output.channel(0)[i];
        }
        ok &= require(finiteLow, "compressor circuit graph node processes finite audio at low makeup gain");

        node.reset();
        node.setParameterValue(0, 1.0f); // maximum makeup gain
        node.process(input, output, 64);
        bool finiteHigh = true;
        double highEnergy = 0.0;
        for (int i = 0; i < 64; ++i) {
            finiteHigh &= std::isfinite(output.channel(0)[i]);
            highEnergy += static_cast<double>(output.channel(0)[i]) * output.channel(0)[i];
        }
        ok &= require(finiteHigh, "compressor circuit graph node processes finite audio at high makeup gain");
        ok &= require(highEnergy > lowEnergy,
                      "raising Makeup Gain increases the compressor circuit node's output energy");
    }

    return ok ? 0 : 1;
}
