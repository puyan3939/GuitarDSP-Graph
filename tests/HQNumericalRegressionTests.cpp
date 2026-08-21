#include "guitardsp/graph/AudioBuffer.h"
#include "guitardsp/hq/FFT.h"
#include "guitardsp/hq/PartitionedConvolver.h"
#include "guitardsp/hq/PolyphaseOversampler.h"

#include <algorithm>
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

    // FFT energy sanity: forward FFT energy is N times time-domain energy.
    {
        constexpr std::size_t n = 256;
        std::vector<std::complex<float>> x(n);
        double timeEnergy = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const float s = 0.7f * std::sin(2.0f * std::numbers::pi_v<float> * 13.0f * static_cast<float>(i) / static_cast<float>(n));
            x[i] = {s, 0.0f};
            timeEnergy += static_cast<double>(s) * static_cast<double>(s);
        }
        hq::Radix2FFT::transform(x, false);
        double frequencyEnergy = 0.0;
        for (const auto& v : x) frequencyEnergy += static_cast<double>(std::norm(v));
        const double relative = std::abs(frequencyEnergy - timeEnergy * static_cast<double>(n))
                              / std::max(1.0, timeEnergy * static_cast<double>(n));
        ok &= require(relative < 2.0e-4, "FFT satisfies Parseval scaling");
    }

    // Long IR reference: uniform partitioned convolution must agree with direct convolution.
    {
        constexpr int block = 16;
        constexpr int blocks = 8;
        const std::vector<float> ir {
            0.80f, -0.20f, 0.10f, 0.05f, -0.025f, 0.0125f, 0.0f, 0.0f,
            0.04f, -0.02f, 0.01f, 0.005f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.03f, 0.015f, -0.008f, 0.004f, 0.002f
        };
        std::vector<float> input(static_cast<std::size_t>(block * blocks));
        for (int i = 0; i < block * blocks; ++i)
            input[static_cast<std::size_t>(i)] = 0.3f * std::sin(0.17f * static_cast<float>(i)) + 0.1f * std::cos(0.071f * static_cast<float>(i));

        std::vector<float> reference(input.size(), 0.0f);
        for (std::size_t n = 0; n < input.size(); ++n)
            for (std::size_t k = 0; k < ir.size() && k <= n; ++k)
                reference[n] += input[n - k] * ir[k];

        hq::PartitionedConvolver c;
        c.prepare(block, 64);
        c.setImpulseResponse(ir);
        std::vector<float> actual(input.size(), 0.0f);
        for (int b = 0; b < blocks; ++b)
            c.processBlock(input.data() + b * block, actual.data() + b * block, block);

        float maxError = 0.0f;
        for (std::size_t i = 0; i < actual.size(); ++i)
            maxError = std::max(maxError, std::abs(actual[i] - reference[i]));
        ok &= require(maxError < 2.0e-4f, "partitioned convolution matches direct convolution");
    }

    // Identity-through-polyphase should preserve DC after filter warm-up.
    {
        constexpr int samples = 1024;
        graph::AudioBuffer in(1, samples), out(1, samples);
        for (int i = 0; i < samples; ++i) in.channel(0)[i] = 1.0f;
        hq::PolyphaseOversampler os;
        os.prepare(1, samples, 8, 127);
        os.process(in, out, samples, [](int, float x) noexcept { return x; });

        double mean = 0.0;
        int count = 0;
        for (int i = samples / 2; i < samples; ++i) {
            mean += out.channel(0)[i];
            ++count;
        }
        mean /= static_cast<double>(count);
        ok &= require(std::isfinite(mean) && std::abs(mean - 1.0) < 0.03, "polyphase oversampling preserves steady DC gain");
    }

    return ok ? 0 : 1;
}
