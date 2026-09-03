#pragma once

#include "guitardsp/hq/Measurement.h"

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

namespace guitardsp::app {

// Message-thread-only THD/harmonic analyser. Samples are handed in via
// pushSamples(), expected to be called from MainComponent::timerCallback()
// with data drained from an AudioTapFifo -- the same FIFO/UI pattern used
// for the waveform and spectrum displays. Accumulates a fixed-size,
// non-overlapping block of samples and runs guitardsp::hq::analyzeHarmonics()
// once the block fills; that call (windowed DFT probes at the fundamental
// and each harmonic) only ever happens here, never from the audio callback.
class ThdAnalyser {
public:
    explicit ThdAnalyser(int windowSize = 4096) { setWindowSize(windowSize); }

    void setWindowSize(int numSamples) {
        windowSize_ = std::max(256, numSamples);
        accumulator_.assign(static_cast<std::size_t>(windowSize_), 0.0f);
        accumulated_ = 0;
    }

    void setSampleRate(double sampleRate) noexcept { sampleRate_ = sampleRate; }
    void setFundamentalHz(double fundamentalHz) noexcept { fundamentalHz_ = fundamentalHz; }
    void setHarmonicCount(int harmonicCount) noexcept { harmonicCount_ = std::max(2, harmonicCount); }

    void reset() {
        accumulated_ = 0;
        metrics_ = guitardsp::hq::HarmonicMetrics{};
        hasMetrics_ = false;
    }

    // Message thread only -- see class comment. Runs one analysis pass each
    // time windowSize() new samples have arrived.
    void pushSamples(const float* samples, int numSamples) {
        if (samples == nullptr || numSamples <= 0 || windowSize_ <= 0) return;
        for (int i = 0; i < numSamples; ++i) {
            accumulator_[static_cast<std::size_t>(accumulated_)] = samples[i];
            if (++accumulated_ >= windowSize_) {
                metrics_ = guitardsp::hq::analyzeHarmonics(
                    std::span<const float>(accumulator_.data(), accumulator_.size()),
                    sampleRate_, fundamentalHz_, harmonicCount_);
                hasMetrics_ = true;
                accumulated_ = 0;
            }
        }
    }

    bool hasMetrics() const noexcept { return hasMetrics_; }
    const guitardsp::hq::HarmonicMetrics& metrics() const noexcept { return metrics_; }

private:
    int windowSize_ = 4096;
    int accumulated_ = 0;
    double sampleRate_ = 48000.0;
    double fundamentalHz_ = 440.0;
    int harmonicCount_ = 8;
    std::vector<float> accumulator_;
    guitardsp::hq::HarmonicMetrics metrics_{};
    bool hasMetrics_ = false;
};

} // namespace guitardsp::app
