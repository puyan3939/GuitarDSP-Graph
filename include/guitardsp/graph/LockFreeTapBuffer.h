#pragma once

#include "UtilityNodes.h"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <vector>

namespace guitardsp::graph {

class LockFreeTapBuffer final : public TapSink {
public:
    void prepare(int channels, int capacitySamples) {
        channels_ = std::clamp(channels, 1, 2);
        capacity_ = std::max(64, capacitySamples);
        storage_.assign(static_cast<std::size_t>(channels_ * capacity_), 0.0f);
        writeCounter_.store(0, std::memory_order_relaxed);
    }

    void push(const AudioBuffer& block, int numSamples) noexcept override {
        if (capacity_ <= 0 || storage_.empty()) return;
        const int n = std::min(numSamples, block.samples());
        std::uint64_t w = writeCounter_.load(std::memory_order_relaxed);
        const int chs = std::min(channels_, block.channels());
        for (int i = 0; i < n; ++i) {
            const int index = static_cast<int>((w + static_cast<std::uint64_t>(i)) % static_cast<std::uint64_t>(capacity_));
            for (int ch = 0; ch < chs; ++ch)
                storage_[static_cast<std::size_t>(ch * capacity_ + index)] = block.channel(ch)[i];
        }
        writeCounter_.store(w + static_cast<std::uint64_t>(n), std::memory_order_release);
    }

    int readLatest(AudioBuffer& destination, int requestedSamples) const noexcept {
        if (capacity_ <= 0 || storage_.empty()) return 0;
        const std::uint64_t end = writeCounter_.load(std::memory_order_acquire);
        const int available = static_cast<int>(std::min<std::uint64_t>(end, static_cast<std::uint64_t>(capacity_)));
        const int n = std::min({requestedSamples, available, destination.samples()});
        const int chs = std::min(channels_, destination.channels());
        const std::uint64_t start = end - static_cast<std::uint64_t>(n);
        for (int i = 0; i < n; ++i) {
            const int index = static_cast<int>((start + static_cast<std::uint64_t>(i)) % static_cast<std::uint64_t>(capacity_));
            for (int ch = 0; ch < chs; ++ch)
                destination.channel(ch)[i] = storage_[static_cast<std::size_t>(ch * capacity_ + index)];
        }
        return n;
    }

private:
    int channels_ = 0, capacity_ = 0;
    std::vector<float> storage_;
    std::atomic<std::uint64_t> writeCounter_{0};
};

} // namespace guitardsp::graph
