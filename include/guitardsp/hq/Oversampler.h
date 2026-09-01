#pragma once
#include "guitardsp/graph/AudioBuffer.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

// Integer-factor oversampler for nonlinear nodes. Coefficients and work buffers
// are prepared off the audio thread. Processing performs no allocation.
class Oversampler {
public:
    void prepare(int channels, int maxBlockSize, int factor, int taps) {
        channels_ = std::max(1, channels);
        maxBlock_ = std::max(1, maxBlockSize);
        factor_ = std::clamp(factor, 1, 16);
        taps_ = std::max(7, taps | 1);
        designLowpass();
        upState_.assign(static_cast<std::size_t>(channels_), std::vector<float>(static_cast<std::size_t>(taps_), 0.0f));
        downState_.assign(static_cast<std::size_t>(channels_), std::vector<float>(static_cast<std::size_t>(taps_), 0.0f));
        upWrite_.assign(static_cast<std::size_t>(channels_), 0);
        downWrite_.assign(static_cast<std::size_t>(channels_), 0);
        work_.resize(channels_, maxBlock_ * factor_);
    }

    void reset() noexcept {
        for (auto& s : upState_) std::fill(s.begin(), s.end(), 0.0f);
        for (auto& s : downState_) std::fill(s.begin(), s.end(), 0.0f);
        std::fill(upWrite_.begin(), upWrite_.end(), 0);
        std::fill(downWrite_.begin(), downWrite_.end(), 0);
        work_.clear();
    }

    int factor() const noexcept { return factor_; }
    int latencySamples() const noexcept {
        if (factor_ <= 1) return 0;
        return (taps_ - 1) / (2 * factor_);
    }

    template <typename SampleProcessor>
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output, int numSamples, SampleProcessor&& processor) noexcept {
        if (factor_ == 1) {
            const int channels = std::min(input.channels(), output.channels());
            for (int ch = 0; ch < channels; ++ch) {
                const float* in = input.channel(ch);
                float* out = output.channel(ch);
                for (int i = 0; i < numSamples; ++i) out[i] = processor(ch, in[i]);
            }
            return;
        }

        const int channels = std::min({input.channels(), output.channels(), channels_});
        const int highSamples = numSamples * factor_;
        for (int ch = 0; ch < channels; ++ch) {
            const float* in = input.channel(ch);
            float* hi = work_.channel(ch);
            for (int n = 0; n < numSamples; ++n) {
                for (int phase = 0; phase < factor_; ++phase) {
                    const float z = phase == 0 ? in[n] * static_cast<float>(factor_) : 0.0f;
                    hi[n * factor_ + phase] = filterSample(z, upState_[static_cast<std::size_t>(ch)], upWrite_[static_cast<std::size_t>(ch)]);
                }
            }
            for (int n = 0; n < highSamples; ++n) hi[n] = processor(ch, hi[n]);

            float* out = output.channel(ch);
            for (int n = 0; n < highSamples; ++n) {
                const float y = filterSample(hi[n], downState_[static_cast<std::size_t>(ch)], downWrite_[static_cast<std::size_t>(ch)]);
                if ((n % factor_) == factor_ - 1) out[n / factor_] = y;
            }
        }
    }

private:
    void designLowpass() {
        kernel_.assign(static_cast<std::size_t>(taps_), 0.0f);
        if (factor_ <= 1) { kernel_[static_cast<std::size_t>(taps_ / 2)] = 1.0f; return; }
        const double fc = 0.46 / static_cast<double>(factor_);
        const int mid = taps_ / 2;
        double sum = 0.0;
        for (int i = 0; i < taps_; ++i) {
            const int m = i - mid;
            const double sinc = m == 0 ? 2.0 * fc : std::sin(2.0 * std::numbers::pi * fc * m) / (std::numbers::pi * m);
            const double x = static_cast<double>(i) / static_cast<double>(taps_ - 1);
            const double window = 0.42 - 0.5 * std::cos(2.0 * std::numbers::pi * x) + 0.08 * std::cos(4.0 * std::numbers::pi * x);
            kernel_[static_cast<std::size_t>(i)] = static_cast<float>(sinc * window);
            sum += sinc * window;
        }
        if (std::abs(sum) > 1.0e-12) for (auto& c : kernel_) c = static_cast<float>(c / sum);
    }

    float filterSample(float x, std::vector<float>& state, int& write) noexcept {
        state[static_cast<std::size_t>(write)] = x;
        double acc = 0.0;
        int r = write;
        for (int k = 0; k < taps_; ++k) {
            acc += static_cast<double>(kernel_[static_cast<std::size_t>(k)]) * state[static_cast<std::size_t>(r)];
            if (--r < 0) r = taps_ - 1;
        }
        if (++write >= taps_) write = 0;
        return static_cast<float>(acc);
    }

    int channels_ = 2;
    int maxBlock_ = 512;
    int factor_ = 1;
    int taps_ = 31;
    std::vector<float> kernel_;
    std::vector<std::vector<float>> upState_, downState_;
    std::vector<int> upWrite_, downWrite_;
    graph::AudioBuffer work_;
};

} // namespace guitardsp::hq
