#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/ReferenceCabinetIR.h"
#include "guitardsp/hq/CabinetChainNode.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <vector>

using namespace guitardsp;

namespace {

bool require(bool condition, const char* description) {
    std::cout << (condition ? "PASS " : "FAIL ") << description << '\n';
    return condition;
}

double responseDb(const std::vector<float>& impulse, double sampleRate,
                  double frequency) {
    const double magnitude = app::detail::cabinetImpulseMagnitude(
        impulse, sampleRate, frequency);
    return 20.0 * std::log10(std::max(magnitude, 1.0e-12));
}

float processToneRms(hq::CabinetChainNode& node, double sampleRate,
                     double frequency, float amplitude) {
    constexpr int blockSize = 256;
    constexpr int blocks = 36;
    graph::AudioBuffer input(1, blockSize);
    graph::AudioBuffer output(1, blockSize);
    double energy = 0.0;
    int count = 0;
    for (int block = 0; block < blocks; ++block) {
        for (int index = 0; index < blockSize; ++index) {
            const int sample = block * blockSize + index;
            input.channel(0)[index] = amplitude * static_cast<float>(std::sin(
                2.0 * std::numbers::pi * frequency
                * static_cast<double>(sample) / sampleRate));
        }
        node.process(input, output, blockSize);
        if (block >= blocks / 2) {
            for (int index = 0; index < blockSize; ++index) {
                const double sample = output.channel(0)[index];
                energy += sample * sample;
                ++count;
            }
        }
    }
    return static_cast<float>(std::sqrt(energy / static_cast<double>(count)));
}

} // namespace

int main() {
    bool ok = true;

    for (const double sampleRate : {44100.0, 48000.0, 96000.0}) {
        const auto impulse = app::makeReferenceCabinetImpulse(sampleRate);
        const auto response = app::analyzeCabinetImpulse(impulse, sampleRate);
        const double lowE = responseDb(impulse, sampleRate, 82.41);
        const double body = responseDb(impulse, sampleRate, 110.0);
        const double middle = responseDb(impulse, sampleRate, 1000.0);
        const double subBass = responseDb(impulse, sampleRate, 30.0);
        const double upperTreble = responseDb(impulse, sampleRate, 10000.0);

        ok &= require(response.allFinite && response.maximumGainDb <= 3.3
                          && std::abs(response.midbandGainDb + 0.75) < 0.15,
                      "guitar fallback has calibrated broadband and bounded peak gain");
        ok &= require(lowE > -2.0 && body - middle < 4.5
                          && body - middle > 1.0,
                      "guitar cabinet keeps controlled body without runaway LF resonance");
        ok &= require(subBass < middle - 8.0 && upperTreble < middle - 4.5,
                      "guitar cabinet rolls off sub-bass and excessive top end");
    }

    {
        auto measured = app::makeReferenceCabinetImpulse(48000.0);
        for (float& value : measured) value *= 18.0f;
        const auto calibrated = app::calibrateMeasuredCabinetImpulse(measured, 48000.0);
        ok &= require(calibrated.before.maximumGainDb > 20.0
                          && calibrated.after.maximumGainDb <= 4.01
                          && calibrated.appliedGainDb < -15.0,
                      "external IR is calibrated by actual convolution response");

        const double originalShape = responseDb(measured, 48000.0, 220.0)
                                   - responseDb(measured, 48000.0, 2200.0);
        const double calibratedShape = responseDb(calibrated.impulse, 48000.0, 220.0)
                                     - responseDb(calibrated.impulse, 48000.0, 2200.0);
        ok &= require(std::abs(originalShape - calibratedShape) < 0.08,
                      "external IR broadband calibration preserves measured tonal shape");

        measured[12] = std::numeric_limits<float>::quiet_NaN();
        const auto sanitized = app::calibrateMeasuredCabinetImpulse(measured, 48000.0);
        ok &= require(!sanitized.before.allFinite && sanitized.after.allFinite,
                      "nonfinite external IR samples are contained before realtime preparation");
    }

    {
        graph::PrepareSpec spec;
        spec.sampleRate = 48000.0;
        spec.maximumBlockSize = 256;
        spec.channels = 1;
        hq::CabinetChainNode cabinet;
        cabinet.setPartitionSize(64);
        cabinet.setImpulseResponse({1.0f});
        cabinet.prepare(spec);
        cabinet.setParameterValue(0, 0.0f);
        cabinet.setParameterValue(1, 0.0f);
        cabinet.setParameterValue(2, 0.0f);
        cabinet.setParameterValue(4, 1.0f);

        cabinet.setParameterValue(5, 35.0f);
        cabinet.reset();
        const float lowCutOpen = processToneRms(cabinet, 48000.0, 65.0, 0.1f);
        cabinet.setParameterValue(5, 180.0f);
        cabinet.reset();
        const float lowCutClosed = processToneRms(cabinet, 48000.0, 65.0, 0.1f);
        ok &= require(lowCutClosed < lowCutOpen * 0.2f,
                      "live cabinet low-cut removes controllable LF energy");

        cabinet.setParameterValue(5, 35.0f);
        cabinet.setParameterValue(6, 14000.0f);
        cabinet.reset();
        const float highCutOpen = processToneRms(cabinet, 48000.0, 7200.0, 0.1f);
        cabinet.setParameterValue(6, 2800.0f);
        cabinet.reset();
        const float highCutClosed = processToneRms(cabinet, 48000.0, 7200.0, 0.1f);
        ok &= require(highCutClosed < highCutOpen * 0.16f,
                      "live cabinet high-cut removes controllable high-frequency fizz");
    }

    return ok ? 0 : 1;
}
