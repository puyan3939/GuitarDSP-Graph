#pragma once

#include "FFT.h"
#include <algorithm>
#include <complex>
#include <cstddef>
#include <span>
#include <vector>

namespace guitardsp::hq {

// Uniform partitioned FFT convolution. prepare()/setImpulseResponse() are control-thread
// operations. processBlock() is allocation-free and expects exactly partitionSize samples.
class PartitionedConvolver {
public:
    void prepare(int partitionSize, int maxImpulseSamples) {
        partitionSize_ = std::max(1, partitionSize);
        fftSize_ = static_cast<int>(nextPowerOfTwo(static_cast<std::size_t>(partitionSize_ * 2)));
        maxPartitions_ = std::max(1, (std::max(1, maxImpulseSamples) + partitionSize_ - 1) / partitionSize_);

        impulsePartitions_.assign(static_cast<std::size_t>(maxPartitions_),
                                  std::vector<std::complex<float>>(static_cast<std::size_t>(fftSize_)));
        inputHistory_.assign(static_cast<std::size_t>(maxPartitions_),
                             std::vector<std::complex<float>>(static_cast<std::size_t>(fftSize_)));
        fftInput_.assign(static_cast<std::size_t>(fftSize_), {});
        accumulator_.assign(static_cast<std::size_t>(fftSize_), {});
        overlap_.assign(static_cast<std::size_t>(partitionSize_), 0.0f);
        activePartitions_ = 1;
        historyWrite_ = 0;
    }

    void reset() noexcept {
        for (auto& partition : inputHistory_) std::fill(partition.begin(), partition.end(), std::complex<float>{});
        std::fill(overlap_.begin(), overlap_.end(), 0.0f);
        historyWrite_ = 0;
    }

    bool setImpulseResponse(std::span<const float> impulse) {
        if (partitionSize_ <= 0 || impulsePartitions_.empty()) return false;
        const int requested = std::max(1, static_cast<int>((impulse.size() + static_cast<std::size_t>(partitionSize_) - 1u)
                                                           / static_cast<std::size_t>(partitionSize_)));
        activePartitions_ = std::min(requested, maxPartitions_);
        for (auto& partition : impulsePartitions_) std::fill(partition.begin(), partition.end(), std::complex<float>{});

        for (int p = 0; p < activePartitions_; ++p) {
            auto& dst = impulsePartitions_[static_cast<std::size_t>(p)];
            for (int i = 0; i < partitionSize_; ++i) {
                const std::size_t sourceIndex = static_cast<std::size_t>(p * partitionSize_ + i);
                if (sourceIndex < impulse.size()) dst[static_cast<std::size_t>(i)] = {impulse[sourceIndex], 0.0f};
            }
            Radix2FFT::transform(dst, false);
        }
        reset();
        return true;
    }

    bool processBlock(const float* input, float* output, int numSamples) noexcept {
        if (input == nullptr || output == nullptr || numSamples != partitionSize_ || fftSize_ <= 0) return false;

        std::fill(fftInput_.begin(), fftInput_.end(), std::complex<float>{});
        for (int i = 0; i < partitionSize_; ++i) fftInput_[static_cast<std::size_t>(i)] = {input[i], 0.0f};
        Radix2FFT::transform(fftInput_, false);
        inputHistory_[static_cast<std::size_t>(historyWrite_)] = fftInput_;

        std::fill(accumulator_.begin(), accumulator_.end(), std::complex<float>{});
        for (int p = 0; p < activePartitions_; ++p) {
            int historyIndex = historyWrite_ - p;
            while (historyIndex < 0) historyIndex += maxPartitions_;
            const auto& x = inputHistory_[static_cast<std::size_t>(historyIndex)];
            const auto& h = impulsePartitions_[static_cast<std::size_t>(p)];
            for (int k = 0; k < fftSize_; ++k)
                accumulator_[static_cast<std::size_t>(k)] += x[static_cast<std::size_t>(k)] * h[static_cast<std::size_t>(k)];
        }
        Radix2FFT::transform(accumulator_, true);

        for (int i = 0; i < partitionSize_; ++i) {
            output[i] = accumulator_[static_cast<std::size_t>(i)].real() + overlap_[static_cast<std::size_t>(i)];
            overlap_[static_cast<std::size_t>(i)] = accumulator_[static_cast<std::size_t>(i + partitionSize_)].real();
        }

        if (++historyWrite_ >= maxPartitions_) historyWrite_ = 0;
        return true;
    }

    int partitionSize() const noexcept { return partitionSize_; }
    int fftSize() const noexcept { return fftSize_; }
    int activePartitions() const noexcept { return activePartitions_; }

private:
    int partitionSize_ = 0;
    int fftSize_ = 0;
    int maxPartitions_ = 0;
    int activePartitions_ = 0;
    int historyWrite_ = 0;
    std::vector<std::vector<std::complex<float>>> impulsePartitions_;
    std::vector<std::vector<std::complex<float>>> inputHistory_;
    std::vector<std::complex<float>> fftInput_;
    std::vector<std::complex<float>> accumulator_;
    std::vector<float> overlap_;
};

} // namespace guitardsp::hq
