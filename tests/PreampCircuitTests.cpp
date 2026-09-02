#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/hq/Measurement.h"
#include "guitardsp/hq/PreampCircuitNode.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
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

    // 1. DC operating point: a self-biased common-cathode 12AX7 stage must
    // settle to a physically plausible, nonsingular loaded operating point
    // (grid near the grid-leak's 0 V reference, plate comfortably between
    // cutoff and the 300 V B+ rail, cathode self-bias in the volt-or-so range
    // typical of a 1.5k cathode resistor) purely from Newton continuation,
    // with no DC offset surviving the AC-coupled output.
    {
        circuit::PreampCircuit preamp;
        ok &= require(preamp.prepare(sampleRate), "component-level 12AX7 preamp prepares");
        const auto primeStats = preamp.lastSolveStats();
        ok &= require(!primeStats.singular, "preamp DC operating point is nonsingular");

        const auto s = preamp.stageVoltages();
        std::cout << "DIAG preamp dc grid=" << s.grid << " plate=" << s.plate
                  << " cathode=" << s.cathode << " toneIn=" << s.toneIn
                  << " toneOut=" << s.toneOut << " output=" << s.output << '\n';
        ok &= require(std::isfinite(s.grid) && std::isfinite(s.plate) &&
                          std::isfinite(s.cathode) && std::isfinite(s.output),
                      "preamp DC stage voltages are finite");
        ok &= require(s.grid > -0.05f && s.grid < 0.05f,
                      "self-biased grid settles near the grid-leak's 0 V reference");
        ok &= require(s.plate > 80.0f && s.plate < 260.0f,
                      "plate settles to a loaded operating point between cutoff and B+");
        ok &= require(s.cathode > 0.5f && s.cathode < 4.0f,
                      "cathode self-bias settles to a plausible Vk for a 1.5k cathode resistor");
        ok &= require(std::abs(s.output) < 0.01f,
                      "AC-coupled output carries no DC offset at silence");
    }

    // 2. Silence must not jitter. The Newton-solver robustness work referenced
    // in the issue is specifically about quiet/silent samples no longer
    // limit-cycling between adjacent float-sized operating points; this
    // mirrors the same silence checks TS808CircuitTests/DS1CircuitTests use.
    {
        circuit::PreampCircuit preamp;
        ok &= require(preamp.prepare(sampleRate), "preamp prepares for the silence test");

        preamp.engine().resetPerformanceStats();
        constexpr int silentSamples = 16384;
        // The output coupling network still carries a slowly *decaying*
        // startup transient into this window (a ~100 ms RC tail, not solver
        // jitter), so jitter itself -- per-sample bounce rather than smooth
        // monotonic decay -- is judged from the settled tail, not the whole
        // window.
        constexpr int tailStart = silentSamples - 4096;
        constexpr int tailHalf = (silentSamples - tailStart) / 2;
        int totalIterations = 0;
        int unconverged = 0;
        float maxTailAbsOutput = 0.0f;
        double tailFirstHalfSumSquares = 0.0;
        double tailSecondHalfSumSquares = 0.0;
        for (int i = 0; i < silentSamples; ++i) {
            const float output = preamp.processSample(0.0f);
            const auto stats = preamp.lastSolveStats();
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
        const auto performance = preamp.engine().performanceStats();
        const double tailFirstHalfRms = std::sqrt(tailFirstHalfSumSquares / tailHalf);
        const double tailSecondHalfRms = std::sqrt(tailSecondHalfSumSquares / tailHalf);
        std::cout << "DIAG silent-preamp average_iterations=" << averageIterations
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
        // Sub-mV numerical dither at the Newton residual tolerance is
        // expected here (the same float-precision effect TS808/DS1 guard
        // against with a matched residual tolerance); what must not happen
        // is that dither *growing* into a self-sustaining oscillation, so
        // the second half of the tail must not be louder than the first.
        ok &= require(tailSecondHalfRms <= tailFirstHalfRms * 1.5 + 1.0e-6,
                      "the settled tail does not grow into a self-sustaining oscillation");
    }

    // 3. Guitar-level signal: a clean-ish single 12AX7 gain stage should
    // amplify a typical pickup-level input and show a small, well-behaved
    // rise in harmonic content (light overdrive) as the input level rises
    // toward the stage's headroom, without ever losing a finite, bounded
    // output.
    {
        circuit::PreampCircuit preamp;
        ok &= require(preamp.prepare(sampleRate), "preamp prepares for the signal-level test");

        constexpr int settleSamples = 4096;
        constexpr int measureSamples = 4096;
        constexpr double toneHz = 220.0;

        const auto measure = [&](float amplitude) {
            preamp.reset();
            for (int i = 0; i < settleSamples; ++i) {
                const float x = amplitude *
                    static_cast<float>(std::sin(2.0 * pi * toneHz * i / sampleRate));
                preamp.processSample(x);
            }
            std::vector<float> input(measureSamples);
            std::vector<float> output(measureSamples);
            for (int i = 0; i < measureSamples; ++i) {
                const float x = amplitude *
                    static_cast<float>(std::sin(2.0 * pi * toneHz * (settleSamples + i) / sampleRate));
                input[static_cast<std::size_t>(i)] = x;
                output[static_cast<std::size_t>(i)] = preamp.processSample(x);
            }
            return std::make_pair(input, output);
        };

        // A typical passive guitar pickup peaks somewhere around 0.05-0.3 V;
        // 0.1 V is a representative playing level.
        const auto [quietIn, quietOut] = measure(0.10f);
        bool quietFinite = true;
        for (float y : quietOut) quietFinite &= std::isfinite(y);
        const double quietInRms = rms(quietIn, 0);
        const double quietOutRms = rms(quietOut, 0);
        const auto quietHarmonics = hq::analyzeHarmonics(quietOut, sampleRate, toneHz, 8);
        std::cout << "DIAG preamp guitar-level in_rms=" << quietInRms
                  << " out_rms=" << quietOutRms << " gain=" << (quietOutRms / quietInRms)
                  << " thd_db=" << quietHarmonics.thdDb << '\n';
        ok &= require(quietFinite, "guitar-level output stays finite");
        ok &= require(quietOutRms > quietInRms,
                      "the stage amplifies a typical guitar playing level");
        ok &= require(std::isfinite(quietHarmonics.thdDb), "guitar-level harmonic metrics are finite");

        // A hotter input (closer to the stage's grid-conduction/plate-swing
        // headroom) should push harmonic distortion up rather than staying
        // perfectly linear -- exercising the same nonlinear triode/grid-diode
        // stamps the Newton hardening was meant to keep stable under drive.
        const auto [hotIn, hotOut] = measure(0.60f);
        (void)hotIn;
        bool hotFinite = true;
        float hotPeak = 0.0f;
        for (float y : hotOut) {
            hotFinite &= std::isfinite(y);
            hotPeak = std::max(hotPeak, std::abs(y));
        }
        const auto hotHarmonics = hq::analyzeHarmonics(hotOut, sampleRate, toneHz, 8);
        std::cout << "DIAG preamp hot-level out_peak=" << hotPeak
                  << " thd_db=" << hotHarmonics.thdDb << '\n';
        ok &= require(hotFinite && hotPeak > 0.0f && hotPeak < static_cast<float>(circuit::PreampCircuit::supplyVolts),
                      "driven guitar-level output stays finite and within supply-scale bounds");
        ok &= require(hotHarmonics.thdDb > quietHarmonics.thdDb,
                      "a hotter input increases harmonic distortion (light overdrive character)");
    }

    // 4. Bass/Treble controls must actually change the broadband response,
    // the same way TS808TopologyTests checks the TS808's tone control.
    {
        circuit::PreampCircuit preamp;
        ok &= require(preamp.prepare(sampleRate), "preamp prepares for the tone-control test");

        constexpr int settleSamples = 4096;
        constexpr int measureSamples = 4096;
        const auto measureTone = [&](float bass, float treble) {
            preamp.reset();
            preamp.setControls(bass, treble);
            for (int i = 0; i < settleSamples; ++i) {
                const double t = static_cast<double>(i) / sampleRate;
                const float x = 0.08f * static_cast<float>(
                    std::sin(2.0 * pi * 150.0 * t) + std::sin(2.0 * pi * 3000.0 * t));
                preamp.processSample(x);
            }
            std::vector<float> output(measureSamples);
            for (int i = 0; i < measureSamples; ++i) {
                const double t = static_cast<double>(settleSamples + i) / sampleRate;
                const float x = 0.08f * static_cast<float>(
                    std::sin(2.0 * pi * 150.0 * t) + std::sin(2.0 * pi * 3000.0 * t));
                output[static_cast<std::size_t>(i)] = preamp.processSample(x);
            }
            return output;
        };

        const auto darkOutput = measureTone(0.9f, 0.05f);
        const auto brightOutput = measureTone(0.05f, 0.9f);
        const double darkRms = rms(darkOutput, 0);
        const double brightRms = rms(brightOutput, 0);
        std::cout << "DIAG preamp tone dark_rms=" << darkRms << " bright_rms=" << brightRms << '\n';
        ok &= require(std::abs(brightRms - darkRms) > 1.0e-4,
                      "Bass/Treble controls change the mixed-frequency broadband response");
    }

    // 5. Bass and Treble share a single summing node through their own
    // potentiometers (see PreampCircuit::clampPotPosition). Commanding both
    // knobs to their extreme corners -- including literally 0/1, not just
    // near them -- must stay a bounded, finite audio signal rather than
    // exercise the near-zero-resistance edge case that previously diverged.
    {
        constexpr std::array<std::array<float, 2>, 4> corners{{
            {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}
        }};
        for (const auto& bt : corners) {
            circuit::PreampCircuit preamp;
            ok &= require(preamp.prepare(sampleRate), "preamp prepares for the tone-corner test");
            preamp.setControls(bt[0], bt[1]);
            float peak = 0.0f;
            bool finite = true;
            for (int i = 0; i < 8192; ++i) {
                const float x = 0.12f * static_cast<float>(std::sin(2.0 * pi * 220.0 * i / sampleRate));
                const float y = preamp.processSample(x);
                finite &= std::isfinite(y);
                peak = std::max(peak, std::abs(y));
            }
            std::cout << "DIAG preamp tone-corner bass=" << bt[0] << " treble=" << bt[1]
                      << " peak=" << peak << '\n';
            const std::string label = std::string("Bass=") + std::to_string(bt[0]) +
                " Treble=" + std::to_string(bt[1]) + " stays finite and bounded";
            ok &= require(finite && peak < 10.0f, label.c_str());
        }
    }

    // 6. Graph node wrapper: confirms the AudioNode adapter (the piece that
    // makes this circuit selectable from NodeRegistry/GraphBuilder-driven
    // signal chains, e.g. GuitarDSPApp's CIRCUIT PEDAL slot) exposes the
    // expected Drive/Bass/Treble parameters, prepares, and produces finite
    // audio -- mirroring TS808CircuitTests.cpp's own node-level block.
    {
        hq::PreampCircuitNode node;
        graph::PrepareSpec spec{};
        spec.sampleRate = 48000.0;
        spec.maximumBlockSize = 64;
        spec.channels = 1;
        node.prepare(spec);
        ok &= require(node.prepared(), "preamp circuit graph node prepares");

        ok &= require(node.parameterCount() == 3, "preamp circuit node exposes 3 parameters");
        ok &= require(std::string(node.parameterDescriptor(0).id) == "drive"
                          && std::string(node.parameterDescriptor(1).id) == "bass"
                          && std::string(node.parameterDescriptor(2).id) == "treble",
                      "preamp circuit node parameters are drive/bass/treble");

        graph::AudioBuffer input(1, 64);
        graph::AudioBuffer output(1, 64);
        for (int i = 0; i < 64; ++i)
            input.channel(0)[i] = 0.10f * std::sin(2.0f * 3.14159265358979323846f * 220.0f * i / 48000.0f);
        node.process(input, output, 64);
        bool finite = true;
        for (int i = 0; i < 64; ++i) finite &= std::isfinite(output.channel(0)[i]);
        ok &= require(finite, "preamp circuit graph node processes finite audio");

        // Drive is implemented as an input pre-gain ahead of the fixed-bias
        // 12AX7 stage (see PreampCircuitNode's driveToPreGain), so raising it
        // should measurably increase output energy through the oversampler.
        const auto measureOutputRms = [&](float drive) {
            hq::PreampCircuitNode measureNode;
            measureNode.prepare(spec);
            measureNode.setParameterValue(0, drive);
            graph::AudioBuffer in(1, 64);
            graph::AudioBuffer out(1, 64);
            double sumSquares = 0.0;
            constexpr int blocks = 32;
            for (int b = 0; b < blocks; ++b) {
                for (int i = 0; i < 64; ++i) {
                    const int n = b * 64 + i;
                    in.channel(0)[i] = 0.10f *
                        std::sin(2.0f * 3.14159265358979323846f * 220.0f * n / 48000.0f);
                }
                measureNode.process(in, out, 64);
                if (b >= blocks / 2) {
                    for (int i = 0; i < 64; ++i)
                        sumSquares += static_cast<double>(out.channel(0)[i]) * out.channel(0)[i];
                }
            }
            return std::sqrt(sumSquares / (64.0 * (blocks / 2)));
        };
        const double lowDriveRms = measureOutputRms(0.0f);
        const double highDriveRms = measureOutputRms(1.0f);
        std::cout << "DIAG preamp node drive-gain low_rms=" << lowDriveRms
                  << " high_rms=" << highDriveRms << '\n';
        ok &= require(std::isfinite(lowDriveRms) && std::isfinite(highDriveRms),
                      "preamp node drive sweep stays finite");
        ok &= require(highDriveRms > lowDriveRms,
                      "raising Drive increases output energy through the input pre-gain");
    }

    return ok ? 0 : 1;
}
