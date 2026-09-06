#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

// Thread CPU time is a POSIX facility (clock_gettime(CLOCK_THREAD_CPUTIME_ID)).
// Isolated behind this macro so a future SHARC/PIC port (or any platform
// without it) simply falls back to wall-clock-only timing -- see
// currentThreadCpuTimeNanoseconds() below.
#if defined(__has_include)
#  if __has_include(<time.h>)
#    include <time.h>
#  endif
#endif
#if defined(CLOCK_THREAD_CPUTIME_ID)
#define GUITARDSP_HAS_THREAD_CPU_TIME 1
#else
#define GUITARDSP_HAS_THREAD_CPU_TIME 0
#endif

namespace guitardsp::app {

// Returns the calling thread's CPU time (time actually spent executing on a
// core, excluding time spent preempted/waiting to be scheduled), or
// std::nullopt on platforms without CLOCK_THREAD_CPUTIME_ID. Contrast with
// std::chrono::steady_clock, which measures wall-clock elapsed time and so
// includes any scheduler preemption in between.
[[nodiscard]] inline std::optional<std::uint64_t> currentThreadCpuTimeNanoseconds() noexcept {
#if GUITARDSP_HAS_THREAD_CPU_TIME
    struct timespec ts {};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return std::nullopt;
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL
        + static_cast<std::uint64_t>(ts.tv_nsec);
#else
    return std::nullopt;
#endif
}

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
    // Thread-CPU-time counterparts of averageLoad/peakLoad above, computed
    // against the same per-callback budget. Only meaningful when
    // cpuTimeAvailable is true (see currentThreadCpuTimeNanoseconds()); the
    // gap between this and the wall-clock load above is an estimate of time
    // spent preempted/waiting for the scheduler rather than computing.
    float cpuAverageLoad = 0.0f;
    float cpuPeakLoad = 0.0f;
    bool cpuTimeAvailable = false;
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
        cpuAverageLoad_.store(0.0f, std::memory_order_relaxed);
        cpuPeakLoad_.store(0.0f, std::memory_order_relaxed);
        cpuTimeAvailable_.store(false, std::memory_order_relaxed);
        for (auto& bucket : loadHistogram_)
            bucket.store(0, std::memory_order_relaxed);
    }

    // cpuElapsedNanoseconds is this thread's CPU time for the callback (see
    // currentThreadCpuTimeNanoseconds()), std::nullopt if unavailable on this
    // platform. Existing wall-clock-only callers are unaffected -- the
    // parameter defaults to std::nullopt and every wall-clock computation
    // below is unchanged from before CPU-time tracking was added.
    void recordCallback(int samples, std::uint64_t elapsedNanoseconds,
                         std::optional<std::uint64_t> cpuElapsedNanoseconds = std::nullopt) noexcept {
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

        if (cpuElapsedNanoseconds.has_value()) {
            const float cpuLoad = static_cast<float>(
                static_cast<double>(*cpuElapsedNanoseconds) / static_cast<double>(budget));
            const float previousCpuAverage = cpuAverageLoad_.load(std::memory_order_relaxed);
            const float cpuAverage = count == 0
                ? cpuLoad
                : previousCpuAverage + 0.10f * (cpuLoad - previousCpuAverage);
            cpuAverageLoad_.store(cpuAverage, std::memory_order_relaxed);
            cpuPeakLoad_.store(std::max(cpuPeakLoad_.load(std::memory_order_relaxed), cpuLoad),
                                std::memory_order_relaxed);
            cpuTimeAvailable_.store(true, std::memory_order_relaxed);
        }

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
            percentile99,
            cpuAverageLoad_.load(std::memory_order_relaxed),
            cpuPeakLoad_.load(std::memory_order_relaxed),
            cpuTimeAvailable_.load(std::memory_order_relaxed)
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
    std::atomic<float> cpuAverageLoad_{0.0f};
    std::atomic<float> cpuPeakLoad_{0.0f};
    std::atomic<bool> cpuTimeAvailable_{false};
    std::array<std::atomic<std::uint64_t>, histogramBucketCount> loadHistogram_{};
};

} // namespace guitardsp::app
