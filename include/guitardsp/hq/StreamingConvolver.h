#pragma once

#include "PartitionedConvolver.h"
#include <algorithm>
#include <span>
#include <vector>

namespace guitardsp::hq {

// Streaming adapter around the fixed-partition FFT convolver. Accepts arbitrary
// host block sizes and trades one partition of deterministic latency for a simple,
// allocation-free realtime contract.
class StreamingPartitionedConvolver {
public:
    void prepare(int partitionSize, int maxImpulseSamples) {
        partitionSize_ = std::max(1, partitionSize);
        convolver_.prepare(partitionSize_, maxImpulseSamples);
        inputBlock_.assign(static_cast<std::size_t>(partitionSize_), 0.0f);
        processedBlock_.assign(static_cast<std::size_t>(partitionSize_), 0.0f);
        inputFill_ = 0;
        outputRead_ = 0;
        outputAvailable_ = 0;
    }

    bool setImpulseResponse(std::span<const float> impulse) {
        const bool ok = convolver_.setImpulseResponse(impulse);
        reset();
        return ok;
    }

    void reset() noexcept {
        convolver_.reset();
        std::fill(inputBlock_.begin(), inputBlock_.end(), 0.0f);
        std::fill(processedBlock_.begin(), processedBlock_.end(), 0.0f);
        inputFill_ = 0;
        outputRead_ = 0;
        outputAvailable_ = 0;
    }

    void process(const float* input, float* output, int numSamples) noexcept {
        if (input == nullptr || output == nullptr || partitionSize_ <= 0) return;
        for (int i = 0; i < numSamples; ++i) {
            output[i] = outputAvailable_ > 0
                ? processedBlock_[static_cast<std::size_t>(outputRead_)]
                : 0.0f;
            if (outputAvailable_ > 0) {
                ++outputRead_;
                --outputAvailable_;
            }

            inputBlock_[static_cast<std::size_t>(inputFill_)] = input[i];
            ++inputFill_;
            if (inputFill_ == partitionSize_) {
                convolver_.processBlock(inputBlock_.data(), processedBlock_.data(), partitionSize_);
                inputFill_ = 0;
                outputRead_ = 0;
                outputAvailable_ = partitionSize_;
            }
        }
    }

    int latencySamples() const noexcept { return partitionSize_; }
    int partitionSize() const noexcept { return partitionSize_; }

private:
    PartitionedConvolver convolver_;
    int partitionSize_ = 0;
    int inputFill_ = 0;
    int outputRead_ = 0;
    int outputAvailable_ = 0;
    std::vector<float> inputBlock_;
    std::vector<float> processedBlock_;
};

} // namespace guitardsp::hq
