#pragma once

#include <algorithm>
#include <atomic>
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

        if (elapsedNanoseconds > budget)
            deadlineMisses_.fetch_add(1, std::memory_order_relaxed);
    }

    [[nodiscard]] RealtimePerformanceSnapshot snapshot() const noexcept {
        return {
            callbacks_.load(std::memory_order_relaxed),
            deadlineMisses_.load(std::memory_order_relaxed),
            latestDurationNanoseconds_.load(std::memory_order_relaxed),
            peakDurationNanoseconds_.load(std::memory_order_relaxed),
            latestBudgetNanoseconds_.load(std::memory_order_relaxed),
            averageLoad_.load(std::memory_order_relaxed),
            peakLoad_.load(std::memory_order_relaxed)
        };
    }

private:
    std::atomic<double> sampleRate_{48000.0};
    std::atomic<std::uint64_t> callbacks_{0};
    std::atomic<std::uint64_t> deadlineMisses_{0};
    std::atomic<std::uint64_t> latestDurationNanoseconds_{0};
    std::atomic<std::uint64_t> peakDurationNanoseconds_{0};
    std::atomic<std::uint64_t> latestBudgetNanoseconds_{0};
    std::atomic<float> averageLoad_{0.0f};
    std::atomic<float> peakLoad_{0.0f};
};

} // namespace guitardsp::app
