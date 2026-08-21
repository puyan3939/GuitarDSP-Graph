#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

namespace guitardsp::graph {

class AudioBuffer {
public:
    AudioBuffer() = default;
    AudioBuffer(int channels, int samples) { resize(channels, samples); }

    void resize(int channels, int samples) {
        assert(channels >= 0 && samples >= 0);
        channels_ = channels;
        samples_ = samples;
        data_.assign(static_cast<std::size_t>(channels_) * static_cast<std::size_t>(samples_), 0.0f);
    }

    [[nodiscard]] int channels() const noexcept { return channels_; }
    [[nodiscard]] int samples() const noexcept { return samples_; }

    float* channel(int ch) noexcept {
        assert(ch >= 0 && ch < channels_);
        return data_.data() + static_cast<std::size_t>(ch) * static_cast<std::size_t>(samples_);
    }

    const float* channel(int ch) const noexcept {
        assert(ch >= 0 && ch < channels_);
        return data_.data() + static_cast<std::size_t>(ch) * static_cast<std::size_t>(samples_);
    }

    void clear() noexcept { std::fill(data_.begin(), data_.end(), 0.0f); }

    void clear(int numSamples) noexcept {
        const int n = std::clamp(numSamples, 0, samples_);
        for (int ch = 0; ch < channels_; ++ch)
            std::fill_n(channel(ch), n, 0.0f);
    }

    void copyFrom(const AudioBuffer& other) noexcept { copyFrom(other, std::min(samples_, other.samples_)); }

    void copyFrom(const AudioBuffer& other, int numSamples) noexcept {
        const int n = std::clamp(numSamples, 0, std::min(samples_, other.samples_));
        const int chs = std::min(channels_, other.channels_);
        for (int ch = 0; ch < chs; ++ch)
            std::copy_n(other.channel(ch), n, channel(ch));
        for (int ch = chs; ch < channels_; ++ch)
            std::fill_n(channel(ch), n, 0.0f);
    }

    void addFrom(const AudioBuffer& other, int numSamples, float gain = 1.0f) noexcept {
        const int n = std::clamp(numSamples, 0, std::min(samples_, other.samples_));
        const int chs = std::min(channels_, other.channels_);
        for (int ch = 0; ch < chs; ++ch) {
            const float* src = other.channel(ch);
            float* dst = channel(ch);
            for (int i = 0; i < n; ++i) dst[i] += src[i] * gain;
        }
    }

private:
    int channels_ = 0;
    int samples_ = 0;
    std::vector<float> data_;
};

} // namespace guitardsp::graph
