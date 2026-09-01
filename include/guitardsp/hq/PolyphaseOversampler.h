#pragma once

#include "guitardsp/graph/AudioBuffer.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Integer oversampler with polyphase interpolation. The expensive zero-stuffing FIR
// is decomposed into phases; decimation uses a streaming anti-alias FIR. All storage
// is allocated in prepare(). Intended as the production successor to Oversampler.
//
// Latency contract:
// Both the interpolation and decimation FIRs use the same odd, linear-phase kernel.
// Their combined high-rate group delay is therefore exactly taps-1 samples. Rather
// than decimating at an arbitrary phase, we choose the phase congruent with that
// group delay. The effective base-rate impulse response is then symmetric around an
// integer sample and latencySamples() is the exact round-trip group delay reported
// to graph PDC; no fractional-delay approximation is required for this oversampler.
class PolyphaseOversampler {
public:
    void prepare(int channels, int maxBlockSize, int factor, int taps) {
        channels_ = std::max(1, channels);
        maxBlock_ = std::max(1, maxBlockSize);
        factor_ = std::clamp(factor, 1, 16);
        taps_ = std::max(7, taps | 1);
        designKernel();
        buildPhases();

        if (factor_ <= 1) {
            decimationPhase_ = 0;
            latencySamples_ = 0;
        } else {
            const int highRateRoundTripDelay = taps_ - 1;
            decimationPhase_ = highRateRoundTripDelay % factor_;
            latencySamples_ = (highRateRoundTripDelay - decimationPhase_) / factor_;
        }

        const int historySize = std::max(2, (taps_ + factor_ - 1) / factor_ + 2);
        upHistory_.assign(static_cast<std::size_t>(channels_),
                          std::vector<float>(static_cast<std::size_t>(historySize), 0.0f));
        upWrite_.assign(static_cast<std::size_t>(channels_), 0);
        downHistory_.assign(static_cast<std::size_t>(channels_),
                            std::vector<float>(static_cast<std::size_t>(taps_), 0.0f));
        downWrite_.assign(static_cast<std::size_t>(channels_), 0);
        work_.resize(channels_, maxBlock_ * factor_);
    }

    void reset() noexcept {
        for (auto& h : upHistory_) std::fill(h.begin(), h.end(), 0.0f);
        for (auto& h : downHistory_) std::fill(h.begin(), h.end(), 0.0f);
        std::fill(upWrite_.begin(), upWrite_.end(), 0);
        std::fill(downWrite_.begin(), downWrite_.end(), 0);
        work_.clear();
    }

    int factor() const noexcept { return factor_; }
    int taps() const noexcept { return taps_; }
    int decimationPhase() const noexcept { return decimationPhase_; }
    int latencySamples() const noexcept { return latencySamples_; }

    template <typename SampleProcessor>
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples,
                 SampleProcessor&& processor) noexcept {
        const int channels = std::min({channels_, input.channels(), output.channels()});
        if (factor_ == 1) {
            for (int ch = 0; ch < channels; ++ch) {
                const float* in = input.channel(ch);
                float* out = output.channel(ch);
                for (int i = 0; i < numSamples; ++i) out[i] = processor(ch, in[i]);
            }
            return;
        }

        const int highSamples = numSamples * factor_;
        for (int ch = 0; ch < channels; ++ch) {
            float* hi = work_.channel(ch);
            const float* in = input.channel(ch);
            auto& history = upHistory_[static_cast<std::size_t>(ch)];
            int& write = upWrite_[static_cast<std::size_t>(ch)];

            for (int n = 0; n < numSamples; ++n) {
                history[static_cast<std::size_t>(write)] = in[n];
                for (int phase = 0; phase < factor_; ++phase) {
                    double acc = 0.0;
                    int r = write;
                    const auto& coeffs = phases_[static_cast<std::size_t>(phase)];
                    for (float c : coeffs) {
                        acc += static_cast<double>(c) *
                               static_cast<double>(history[static_cast<std::size_t>(r)]);
                        if (--r < 0) r = static_cast<int>(history.size()) - 1;
                    }
                    hi[n * factor_ + phase] =
                        static_cast<float>(acc * static_cast<double>(factor_));
                }
                if (++write >= static_cast<int>(history.size())) write = 0;
            }

            for (int i = 0; i < highSamples; ++i) hi[i] = processor(ch, hi[i]);

            float* out = output.channel(ch);
            auto& down = downHistory_[static_cast<std::size_t>(ch)];
            int& dw = downWrite_[static_cast<std::size_t>(ch)];
            for (int i = 0; i < highSamples; ++i) {
                down[static_cast<std::size_t>(dw)] = hi[i];
                if ((i % factor_) == decimationPhase_) {
                    double acc = 0.0;
                    int r = dw;
                    for (int k = 0; k < taps_; ++k) {
                        acc += static_cast<double>(kernel_[static_cast<std::size_t>(k)]) *
                               static_cast<double>(down[static_cast<std::size_t>(r)]);
                        if (--r < 0) r = taps_ - 1;
                    }
                    out[i / factor_] = static_cast<float>(acc);
                }
                if (++dw >= taps_) dw = 0;
            }
        }
    }

private:
    void designKernel() {
        kernel_.assign(static_cast<std::size_t>(taps_), 0.0f);
        if (factor_ <= 1) {
            kernel_[static_cast<std::size_t>(taps_ / 2)] = 1.0f;
            return;
        }
        const double cutoff = 0.47 / static_cast<double>(factor_);
        const int mid = taps_ / 2;
        double sum = 0.0;
        for (int i = 0; i < taps_; ++i) {
            const int m = i - mid;
            const double sinc = m == 0 ? 2.0 * cutoff
                : std::sin(2.0 * std::numbers::pi * cutoff * static_cast<double>(m)) /
                  (std::numbers::pi * static_cast<double>(m));
            const double phase = static_cast<double>(i) / static_cast<double>(taps_ - 1);
            const double window = 0.42 - 0.5 * std::cos(2.0 * std::numbers::pi * phase)
                                      + 0.08 * std::cos(4.0 * std::numbers::pi * phase);
            kernel_[static_cast<std::size_t>(i)] = static_cast<float>(sinc * window);
            sum += sinc * window;
        }
        if (std::abs(sum) > 1.0e-12)
            for (auto& c : kernel_)
                c = static_cast<float>(static_cast<double>(c) / sum);
    }

    void buildPhases() {
        phases_.assign(static_cast<std::size_t>(factor_), {});
        for (int phase = 0; phase < factor_; ++phase) {
            auto& p = phases_[static_cast<std::size_t>(phase)];
            for (int k = phase; k < taps_; k += factor_)
                p.push_back(kernel_[static_cast<std::size_t>(k)]);
        }
    }

    int channels_ = 2;
    int maxBlock_ = 512;
    int factor_ = 1;
    int taps_ = 31;
    int decimationPhase_ = 0;
    int latencySamples_ = 0;
    std::vector<float> kernel_;
    std::vector<std::vector<float>> phases_;
    std::vector<std::vector<float>> upHistory_;
    std::vector<int> upWrite_;
    std::vector<std::vector<float>> downHistory_;
    std::vector<int> downWrite_;
    graph::AudioBuffer work_;
};

} // namespace guitardsp::hq
