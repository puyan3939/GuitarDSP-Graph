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
        data_.assign(static_cast<std::size_t>(channels * samples), 0.0f);
    }

    void clear() noexcept { std::fill(data_.begin(), data_.end(), 0.0f); }
    int channels() const noexcept { return channels_; }
    int samples() const noexcept { return samples_; }

    float* channel(int ch) noexcept {
        assert(ch >= 0 && ch < channels_);
        return data_.data() + static_cast<std::size_t>(ch * samples_);
    }
    const float* channel(int ch) const noexcept {
        assert(ch >= 0 && ch < channels_);
        return data_.data() + static_cast<std::size_t>(ch * samples_);
    }

    void copyFrom(const AudioBuffer& other) noexcept {
        assert(other.channels_ == channels_ && other.samples_ == samples_);
        std::copy(other.data_.begin(), other.data_.end(), data_.begin());
    }

    void addFrom(const AudioBuffer& other, float gain = 1.0f) noexcept {
        assert(other.channels_ == channels_ && other.samples_ == samples_);
        const auto n = data_.size();
        for (std::size_t i = 0; i < n; ++i)
            data_[i] += other.data_[i] * gain;
    }

private:
    int channels_ = 0;
    int samples_ = 0;
    std::vector<float> data_;
};

} // namespace guitardsp::graph
