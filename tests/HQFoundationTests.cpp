#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/AliasAnalysis.h"
#include "guitardsp/hq/Components.h"
#include "guitardsp/hq/DeviceStages.h"
#include "guitardsp/hq/FFT.h"
#include "guitardsp/hq/PartitionedConvolver.h"
#include "guitardsp/hq/PolyphaseOversampler.h"
#include "guitardsp/hq/VectorOps.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <numbers>
#include <vector>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}
}

int main() {
    bool ok = true;

    {
        std::vector<std::complex<float>> data(8);
        data[0] = {1.0f, 0.0f}; data[1] = {0.5f, 0.0f}; data[2] = {-0.25f, 0.0f};
        const auto original = data;
        hq::Radix2FFT::transform(data, false);
        hq::Radix2FFT::transform(data, true);
        float error = 0.0f;
        for (std::size_t i = 0; i < data.size(); ++i) error += std::abs(data[i] - original[i]);
        ok &= require(error < 1.0e-4f, "radix2 FFT round trip");
    }

    {
        float a[8] {1,2,3,4,5,6,7,8};
        float b[8] {};
        hq::VectorOps::copy(a, b, 8);
        hq::VectorOps::multiply(b, 0.5f, 8);
        hq::VectorOps::addScaled(a, b, 0.25f, 8);
        ok &= require(std::abs(b[7] - 6.0f) < 1.0e-6f, "SIMD/scalar vector ops agree");
        ok &= require(std::abs(hq::VectorOps::peakAbs(b, 8) - 6.0f) < 1.0e-6f, "vector peak measurement");
    }

    {
        constexpr int n = 1024;
        constexpr double sr = 48000.0;
        constexpr double f = 1500.0;
        std::vector<float> sine(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
            sine[static_cast<std::size_t>(i)] = std::sin(2.0 * std::numbers::pi * f * static_cast<double>(i) / sr);
        const auto metrics = hq::analyzeAliasResidual(sine, sr, f, 12, 2);
        ok &= require(std::isfinite(metrics.residualDb), "alias FFT metric finite");
        ok &= require(metrics.harmonicEnergy > metrics.residualEnergy, "bin-centered sine classified mainly as harmonic energy");
    }

    {
        hq::PartitionedConvolver convolver;
        convolver.prepare(8, 32);
        const std::vector<float> ir {1.0f, 0.5f, 0.25f, 0.125f, 0,0,0,0, 0.0625f};
        ok &= require(convolver.setImpulseResponse(ir), "partitioned IR accepted");
        float in[8] {1,0,0,0,0,0,0,0};
        float out[8] {};
        ok &= require(convolver.processBlock(in, out, 8), "partitioned convolution processes first block");
        ok &= require(std::abs(out[0] - 1.0f) < 1.0e-4f && std::abs(out[1] - 0.5f) < 1.0e-4f,
                      "partitioned convolution reproduces early IR");
        float zero[8] {};
        ok &= require(convolver.processBlock(zero, out, 8), "partitioned convolution processes tail block");
        ok &= require(std::abs(out[0] - 0.0625f) < 1.0e-4f, "partitioned convolution carries later partition");
    }

    {
        graph::AudioBuffer in(2, 128), out(2, 128);
        for (int ch = 0; ch < 2; ++ch)
            for (int i = 0; i < 128; ++i) in.channel(ch)[i] = 0.2f * std::sin(0.05f * static_cast<float>(i));
        hq::PolyphaseOversampler os;
        os.prepare(2, 128, 8, 95);
        os.process(in, out, 128, [](int, float x) noexcept { return std::tanh(3.0f * x); });
        bool finite = true;
        float energy = 0.0f;
        for (int ch = 0; ch < 2; ++ch) for (int i = 0; i < 128; ++i) {
            finite &= std::isfinite(out.channel(ch)[i]);
            energy += out.channel(ch)[i] * out.channel(ch)[i];
        }
        ok &= require(os.factor() == 8, "polyphase oversampler configured for 8x");
        ok &= require(finite && energy > 1.0e-5f, "polyphase nonlinear path finite and active");
    }

    {
        const auto si = hq::DiodeModel::forType(hq::DiodeType::silicon);
        const auto ge = hq::DiodeModel::forType(hq::DiodeType::germanium);
        ok &= require(ge.current(0.35f) > si.current(0.35f), "germanium model conducts earlier than silicon model");

        hq::BJTModel bjt;
        ok &= require(bjt.collectorCurrent(0.80f, 1000.0f) > 0.0f, "BJT component produces collector current above Vbe");

        const auto triode = hq::TriodeModel::twelveAX7();
        const float plate = triode.plateCurrent(-1.0f, 250.0f);
        ok &= require(std::isfinite(plate) && plate >= 0.0f, "12AX7 component model finite");

        hq::CathodeBiasState bias;
        bias.setTimeConstant(48000.0, 40.0f);
        const float before = bias.voltage;
        for (int i = 0; i < 1000; ++i) bias.process(1.0f);
        ok &= require(bias.voltage > before && bias.voltage < 1.0f, "cathode bias state has memory");
    }

    {
        hq::DiodePairSolver pair;
        pair.setPositive(hq::DiodeModel::forType(hq::DiodeType::silicon));
        pair.setNegative(hq::DiodeModel::forType(hq::DiodeType::germanium));
        const float pos = pair.process(1.0f);
        pair.reset();
        const float neg = pair.process(-1.0f);
        ok &= require(std::isfinite(pos) && std::isfinite(neg) && std::abs(pos + neg) > 1.0e-4f,
                      "asymmetric diode pair solver is finite and asymmetric");

        hq::TriodeCommonCathodeStage triodeStage;
        triodeStage.prepare(48000.0);
        float triodePeak = 0.0f;
        bool triodeFinite = true;
        for (int i = 0; i < 2000; ++i) {
            const float y = triodeStage.process(0.7f * std::sin(0.03f * static_cast<float>(i)));
            triodeFinite &= std::isfinite(y);
            triodePeak = std::max(triodePeak, std::abs(y));
        }
        ok &= require(triodeFinite && triodePeak > 1.0e-5f, "common-cathode triode stage produces finite dynamic output");

        hq::BJTCommonEmitterStage bjtStage;
        bjtStage.prepare(48000.0);
        float bjtPeak = 0.0f;
        bool bjtFinite = true;
        for (int i = 0; i < 2000; ++i) {
            const float y = bjtStage.process(0.18f * std::sin(0.07f * static_cast<float>(i)));
            bjtFinite &= std::isfinite(y);
            bjtPeak = std::max(bjtPeak, std::abs(y));
        }
        ok &= require(bjtFinite && bjtPeak > 1.0e-5f, "common-emitter BJT stage produces finite dynamic output");
    }

    return ok ? 0 : 1;
}
