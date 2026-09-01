#pragma once

#include "AudioBuffer.h"
#include <algorithm>
#include <cstddef>
#include <vector>

namespace guitardsp::graph {

class DelayCompensator {
public:
    void prepare(int channels, int maximumDelaySamples, int maximumBlockSize) {
        channels_ = std::max(1, channels);
        const int size = std::max(2, maximumDelaySamples + maximumBlockSize + 2);
        lines_.assign(static_cast<std::size_t>(channels_), std::vector<float>(static_cast<std::size_t>(size), 0.0f));
        write_.assign(static_cast<std::size_t>(channels_), 0);
        delaySamples_ = 0;
    }

    void reset() noexcept {
        for (auto& line : lines_) std::fill(line.begin(), line.end(), 0.0f);
        std::fill(write_.begin(), write_.end(), 0);
    }

    void setDelaySamples(int samples) noexcept { delaySamples_ = std::max(0, samples); }
    [[nodiscard]] int delaySamples() const noexcept { return delaySamples_; }

    void processAdd(const AudioBuffer& input, AudioBuffer& destination, int numSamples, float gain = 1.0f) noexcept {
        const int chs = std::min({channels_, input.channels(), destination.channels()});
        const int n = std::clamp(numSamples, 0, std::min(input.samples(), destination.samples()));
        if (delaySamples_ == 0) {
            destination.addFrom(input, n, gain);
            return;
        }
        for (int ch = 0; ch < chs; ++ch) {
            auto& line = lines_[static_cast<std::size_t>(ch)];
            int& w = write_[static_cast<std::size_t>(ch)];
            const int size = static_cast<int>(line.size());
            const float* src = input.channel(ch);
            float* dst = destination.channel(ch);
            for (int i = 0; i < n; ++i) {
                line[static_cast<std::size_t>(w)] = src[i];
                int r = w - delaySamples_;
                while (r < 0) r += size;
                dst[i] += line[static_cast<std::size_t>(r)] * gain;
                if (++w >= size) w = 0;
            }
        }
    }

private:
    int channels_ = 0;
    int delaySamples_ = 0;
    std::vector<std::vector<float>> lines_;
    std::vector<int> write_;
};

} // namespace guitardsp::graph
