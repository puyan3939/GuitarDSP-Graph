#include "guitardsp/circuit/FullAmpCircuit.h"
#include "guitardsp/graph/NodeRegistry.h"
#include "guitardsp/hq/FullAmpCircuitNode.h"
#include "guitardsp/hq/Measurement.h"

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

    // 1. DC operating point: cascading PreampCircuit -> PowerAmpCircuit must
    // still settle each stage to the same kind of physically plausible,
    // nonsingular operating point the two circuits reach standalone (see
    // PreampCircuitTests/PowerAmpCircuitTests) -- prepare() runs each
    // circuit's own DC-priming/source-stepping independently, so neither
    // stage should be perturbed by the other at silence.
    {
        circuit::FullAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "full amp (preamp + power amp cascade) prepares");
        ok &= require(!amp.preampSolveStats().singular, "preamp stage DC operating point is nonsingular");
        ok &= require(!amp.powerAmpSolveStats().singular, "power amp stage DC operating point is nonsingular");

        const auto s = amp.stageVoltages();
        std::cout << "DIAG fullamp dc preamp.plate=" << s.preamp.plate
                  << " preamp.cathode=" << s.preamp.cathode
                  << " preamp.output=" << s.preamp.output
                  << " poweramp.plate=" << s.powerAmp.plate
                  << " poweramp.screen=" << s.powerAmp.screen
                  << " poweramp.cathode=" << s.powerAmp.cathode
                  << " poweramp.output=" << s.powerAmp.output << '\n';
        ok &= require(std::isfinite(s.preamp.plate) && std::isfinite(s.preamp.cathode) &&
                          std::isfinite(s.preamp.output) && std::isfinite(s.powerAmp.plate) &&
                          std::isfinite(s.powerAmp.screen) && std::isfinite(s.powerAmp.cathode) &&
                          std::isfinite(s.powerAmp.output),
                      "full amp DC stage voltages are finite");
        ok &= require(s.preamp.plate > 80.0f && s.preamp.plate < 260.0f,
                      "preamp stage plate settles between cutoff and its 300 V B+");
        ok &= require(s.preamp.cathode > 0.5f && s.preamp.cathode < 4.0f,
                      "preamp stage cathode self-bias is plausible for its 1.5k cathode resistor");
        ok &= require(std::abs(s.preamp.output) < 0.01f,
                      "preamp stage's AC-coupled output carries no DC offset at silence");
        ok &= require(s.powerAmp.plate > 250.0f && s.powerAmp.plate < circuit::PowerAmpCircuit::supplyVolts,
                      "power amp stage plate settles between cutoff and its 420 V B+");
        ok &= require(s.powerAmp.cathode > 10.0f && s.powerAmp.cathode < 100.0f,
                      "power amp stage cathode self-bias is plausible for its 1.2k cathode resistor");
        ok &= require(std::abs(s.powerAmp.output) < 0.01f,
                      "power amp stage's transformer-coupled output carries no DC offset at silence");
    }

    // 2. Silence must not jitter or self-oscillate through the full two-stage
    // cascade -- exercising the backtracking-line-search Newton solver
    // robustness across both the 12AX7 preamp stage and the EL34 power stage
    // running back to back, one solve driving the other's input every
    // sample.
    {
        circuit::FullAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "full amp prepares for the silence test");

        constexpr int silentSamples = 16384;
        constexpr int tailStart = silentSamples - 4096;
        constexpr int tailHalf = (silentSamples - tailStart) / 2;
        int unconverged = 0;
        float maxTailAbsOutput = 0.0f;
        double tailFirstHalfSumSquares = 0.0;
        double tailSecondHalfSumSquares = 0.0;
        for (int i = 0; i < silentSamples; ++i) {
            const float output = amp.processSample(0.0f);
            unconverged += (amp.preampSolveStats().converged && amp.powerAmpSolveStats().converged) ? 0 : 1;
            ok &= std::isfinite(output) &&
                  !amp.preampSolveStats().singular && !amp.powerAmpSolveStats().singular;
            if (i >= tailStart) {
                maxTailAbsOutput = std::max(maxTailAbsOutput, std::abs(output));
                const double squared = static_cast<double>(output) * output;
                if (i < tailStart + tailHalf) tailFirstHalfSumSquares += squared;
                else tailSecondHalfSumSquares += squared;
            }
        }
        const double tailFirstHalfRms = std::sqrt(tailFirstHalfSumSquares / tailHalf);
        const double tailSecondHalfRms = std::sqrt(tailSecondHalfSumSquares / tailHalf);
        std::cout << "DIAG silent-fullamp unconverged=" << unconverged
                  << " tail_max_abs_output=" << maxTailAbsOutput
                  << " tail_rms_first_half=" << tailFirstHalfRms
                  << " tail_rms_second_half=" << tailSecondHalfRms << '\n';
        ok &= require(unconverged == 0, "prolonged silence converges every sample through both stages");
        ok &= require(maxTailAbsOutput < 5.0e-3f,
                      "the settled tail of a silent input decays close to zero");
        ok &= require(tailSecondHalfRms <= tailFirstHalfRms * 1.5 + 1.0e-6,
                      "the settled tail does not grow into a self-sustaining oscillation");
    }

    // 3. Guitar-level signal through the full cascade: a typical pickup-level
    // input should come out amplified and finite, and a hotter input should
    // show more harmonic distortion than a quiet one -- the same monotonic
    // "more level in, more grit out" behaviour each stage shows standalone,
    // now compounded across both stages.
    {
        circuit::FullAmpCircuit amp;
        ok &= require(amp.prepare(sampleRate), "full amp prepares for the signal-level test");

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

        // A typical passive guitar pickup peaks around 0.05-0.3 V.
        const auto [quietIn, quietOut] = measure(0.10f);
        bool quietFinite = true;
        for (float y : quietOut) quietFinite &= std::isfinite(y);
        const double quietInRms = rms(quietIn, 0);
        const double quietOutRms = rms(quietOut, 0);
        const auto quietHarmonics = hq::analyzeHarmonics(quietOut, sampleRate, toneHz, 8);
        std::cout << "DIAG fullamp guitar-level in_rms=" << quietInRms
                  << " out_rms=" << quietOutRms << " gain=" << (quietOutRms / quietInRms)
                  << " thd_db=" << quietHarmonics.thdDb << '\n';
        ok &= require(quietFinite, "guitar-level output stays finite through both stages");
        // The preamp stage alone amplifies the raw pickup signal by tens of
        // volts (see PreampCircuitTests), but that swing is measured at the
        // power stage's *output transformer secondary* here -- a real output
        // transformer steps plate-swing voltage down (turnsRatio ~19.4, see
        // PowerAmpCircuit) to match a low-impedance speaker load, trading
        // voltage for current rather than preserving it, so the end-to-end
        // voltage gain of a full amp chain is not expected to exceed 1 at
        // the speaker-load node. What matters is that the cascade produces a
        // substantial, non-trivial audio signal, the same invariant
        // PowerAmpCircuitTests checks for PowerAmpCircuit's own output.
        ok &= require(quietOutRms > 1.0e-3,
                      "the full cascade produces a non-trivial audio signal at a typical playing level");
        ok &= require(std::isfinite(quietHarmonics.thdDb), "guitar-level harmonic metrics are finite");

        const auto [hotIn, hotOut] = measure(0.60f);
        (void)hotIn;
        bool hotFinite = true;
        float hotPeak = 0.0f;
        for (float y : hotOut) {
            hotFinite &= std::isfinite(y);
            hotPeak = std::max(hotPeak, std::abs(y));
        }
        const auto hotHarmonics = hq::analyzeHarmonics(hotOut, sampleRate, toneHz, 8);
        std::cout << "DIAG fullamp hot-level out_peak=" << hotPeak
                  << " thd_db=" << hotHarmonics.thdDb << '\n';
        ok &= require(hotFinite && hotPeak > 0.0f &&
                          hotPeak < static_cast<float>(circuit::PowerAmpCircuit::supplyVolts),
                      "driven output stays finite and within power-stage supply-scale bounds");
        ok &= require(hotHarmonics.thdDb > quietHarmonics.thdDb,
                      "a hotter input increases harmonic distortion through the full cascade");
    }

    // 4. A real full amp behaves like this: turning the preamp's Drive up
    // doesn't just distort the preamp harder, it also pushes the power stage
    // it feeds into harder, so the *power stage's* distortion should rise
    // too. Exercise this through FullAmpCircuitNode's Drive parameter
    // (pre-gain into the preamp, same role as PreampCircuitNode::Drive) and
    // confirm both output energy and harmonic content rise together with
    // Drive -- i.e. the interaction is a natural side effect of the cascade,
    // not something that needs a separate "power amp drive" control.
    {
        graph::NodeRegistry registry = graph::NodeRegistry::createBuiltins();
        auto registered = registry.create("amp.full_amp_circuit_hq");
        ok &= require(registered != nullptr, "NodeRegistry creates the full amp circuit node");

        hq::FullAmpCircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "full amp circuit graph node prepares");

        ok &= require(node.parameterCount() == 3, "full amp circuit node exposes Drive/Bass/Treble");
        ok &= require(node.parameterIndex("drive") == 0 && node.parameterIndex("bass") == 1 &&
                          node.parameterIndex("treble") == 2,
                      "full amp circuit node parameter ids match Drive/Bass/Treble");

        constexpr int settleBlocks = 64;
        constexpr int measureBlocks = 64;
        constexpr int blockSize = 64;
        const auto measure = [&](float drive) {
            node.reset();
            node.setParameterValue(0, drive);
            graph::AudioBuffer block(1, blockSize);
            long sampleIndex = 0;
            for (int b = 0; b < settleBlocks; ++b) {
                for (int i = 0; i < blockSize; ++i, ++sampleIndex)
                    block.channel(0)[i] = 0.10f *
                        static_cast<float>(std::sin(2.0 * pi * 220.0 * sampleIndex / sampleRate));
                graph::AudioBuffer discard(1, blockSize);
                node.process(block, discard, blockSize);
            }
            std::vector<float> output;
            output.reserve(static_cast<std::size_t>(measureBlocks) * blockSize);
            for (int b = 0; b < measureBlocks; ++b) {
                for (int i = 0; i < blockSize; ++i, ++sampleIndex)
                    block.channel(0)[i] = 0.10f *
                        static_cast<float>(std::sin(2.0 * pi * 220.0 * sampleIndex / sampleRate));
                graph::AudioBuffer out(1, blockSize);
                node.process(block, out, blockSize);
                for (int i = 0; i < blockSize; ++i) output.push_back(out.channel(0)[i]);
            }
            return output;
        };

        const auto lowDriveOut = measure(0.0f);
        const auto highDriveOut = measure(1.0f);
        bool lowFinite = true, highFinite = true;
        for (float y : lowDriveOut) lowFinite &= std::isfinite(y);
        for (float y : highDriveOut) highFinite &= std::isfinite(y);
        ok &= require(lowFinite && highFinite, "full amp circuit node output stays finite across Drive");

        const double lowRms = rms(lowDriveOut, 0);
        const double highRms = rms(highDriveOut, 0);
        const auto lowHarmonics = hq::analyzeHarmonics(lowDriveOut, sampleRate, 220.0, 8);
        const auto highHarmonics = hq::analyzeHarmonics(highDriveOut, sampleRate, 220.0, 8);
        std::cout << "DIAG fullamp-node low_drive_rms=" << lowRms << " low_thd_db=" << lowHarmonics.thdDb
                  << " high_drive_rms=" << highRms << " high_thd_db=" << highHarmonics.thdDb << '\n';
        ok &= require(highRms > lowRms,
                      "raising preamp Drive increases the full cascade's output energy");
        ok &= require(highHarmonics.thdDb > lowHarmonics.thdDb,
                      "raising preamp Drive increases distortion through the power stage too, "
                      "the natural amp-like interaction between the two stages");
    }

    return ok ? 0 : 1;
}
