#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace guitardsp::app {

struct RealtimePerformanceSnapshot {
    std::uint64_t callbacks = 0;
    std::uint64_t deadlineMisses = 0;
    std::uint64_t latestDurationNanoseconds = 0;
    std::uint64_t peakDurationNanoseconds = 0;
    std::uint64_t latestBudgetNanoseconds = 0;
    float averageLoad = 0.0f;
    float peakLoad = 0.0f;
    float percentile95Load = 0.0f;
    float percentile99Load = 0.0f;
};

// One audio-thread producer and a message-thread observer. Timing values are
// deliberately supplied by the caller so deadline arithmetic can be tested
// deterministically without sleeping or depending on a CI runner's speed.
class RealtimePerformanceMonitor {
public:
    void prepare(double sampleRate) noexcept {
        sampleRate_.store(std::max(1.0, sampleRate), std::memory_order_release);
        reset();
    }

    void reset() noexcept {
        callbacks_.store(0, std::memory_order_relaxed);
        deadlineMisses_.store(0, std::memory_order_relaxed);
        latestDurationNanoseconds_.store(0, std::memory_order_relaxed);
        peakDurationNanoseconds_.store(0, std::memory_order_relaxed);
        latestBudgetNanoseconds_.store(0, std::memory_order_relaxed);
        averageLoad_.store(0.0f, std::memory_order_relaxed);
        peakLoad_.store(0.0f, std::memory_order_relaxed);
        for (auto& bucket : loadHistogram_)
            bucket.store(0, std::memory_order_relaxed);
    }

    void recordCallback(int samples, std::uint64_t elapsedNanoseconds) noexcept {
        if (samples <= 0) return;

        const auto budget = static_cast<std::uint64_t>(
            static_cast<double>(samples) * 1000000000.0
            / sampleRate_.load(std::memory_order_acquire));
        if (budget == 0) return;

        const float load = static_cast<float>(
            static_cast<double>(elapsedNanoseconds) / static_cast<double>(budget));
        const auto count = callbacks_.fetch_add(1, std::memory_order_relaxed);
        const float previousAverage = averageLoad_.load(std::memory_order_relaxed);
        const float average = count == 0
            ? load
            : previousAverage + 0.10f * (load - previousAverage);

        latestDurationNanoseconds_.store(elapsedNanoseconds, std::memory_order_relaxed);
        latestBudgetNanoseconds_.store(budget, std::memory_order_relaxed);
        averageLoad_.store(average, std::memory_order_relaxed);
        peakDurationNanoseconds_.store(
            std::max(peakDurationNanoseconds_.load(std::memory_order_relaxed),
                     elapsedNanoseconds),
            std::memory_order_relaxed);
        peakLoad_.store(std::max(peakLoad_.load(std::memory_order_relaxed), load),
                        std::memory_order_relaxed);

        // One fixed atomic increment is sufficient on the realtime thread. The
        // message thread performs the histogram scan when it refreshes the UI;
        // no callback allocates, locks, or sorts timing samples.
        const auto bucket = static_cast<std::size_t>(std::min(
            load * 100.0f, static_cast<float>(histogramBucketCount - 1U)));
        loadHistogram_[bucket].fetch_add(1, std::memory_order_relaxed);

        if (elapsedNanoseconds > budget)
            deadlineMisses_.fetch_add(1, std::memory_order_relaxed);
    }

    [[nodiscard]] RealtimePerformanceSnapshot snapshot() const noexcept {
        const auto callbackCount = callbacks_.load(std::memory_order_relaxed);
        float percentile95 = 0.0f;
        float percentile99 = 0.0f;
        if (callbackCount != 0U) {
            const auto rank95 = callbackCount - callbackCount / 20U;
            const auto rank99 = callbackCount - callbackCount / 100U;
            std::uint64_t observed = 0;
            bool found95 = false;
            for (std::size_t bucket = 0; bucket < histogramBucketCount; ++bucket) {
                observed += loadHistogram_[bucket].load(std::memory_order_relaxed);
                if (!found95 && observed >= rank95) {
                    percentile95 = static_cast<float>(bucket) * 0.01f;
                    found95 = true;
                }
                if (observed >= rank99) {
                    percentile99 = static_cast<float>(bucket) * 0.01f;
                    break;
                }
            }
        }
        return {
            callbackCount,
            deadlineMisses_.load(std::memory_order_relaxed),
            latestDurationNanoseconds_.load(std::memory_order_relaxed),
            peakDurationNanoseconds_.load(std::memory_order_relaxed),
            latestBudgetNanoseconds_.load(std::memory_order_relaxed),
            averageLoad_.load(std::memory_order_relaxed),
            peakLoad_.load(std::memory_order_relaxed),
            percentile95,
            percentile99
        };
    }

private:
    static constexpr std::size_t histogramBucketCount = 512U;
    std::atomic<double> sampleRate_{48000.0};
    std::atomic<std::uint64_t> callbacks_{0};
    std::atomic<std::uint64_t> deadlineMisses_{0};
    std::atomic<std::uint64_t> latestDurationNanoseconds_{0};
    std::atomic<std::uint64_t> peakDurationNanoseconds_{0};
    std::atomic<std::uint64_t> latestBudgetNanoseconds_{0};
    std::atomic<float> averageLoad_{0.0f};
    std::atomic<float> peakLoad_{0.0f};
    std::array<std::atomic<std::uint64_t>, histogramBucketCount> loadHistogram_{};
};

} // namespace guitardsp::app
