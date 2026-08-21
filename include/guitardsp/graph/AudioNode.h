#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace guitardsp::graph
{
using NodeId = std::uint32_t;

struct ProcessContext
{
    double sampleRate = 48000.0;
    std::size_t numSamples = 0;
};

struct AudioBlock
{
    std::span<float> left;
    std::span<float> right;

    [[nodiscard]] std::size_t size() const noexcept
    {
        return left.size();
    }
};

class AudioNode
{
public:
    virtual ~AudioNode() = default;

    [[nodiscard]] virtual std::string_view typeName() const noexcept = 0;
    virtual void prepare(double sampleRate, std::size_t maximumBlockSize) = 0;
    virtual void reset() noexcept = 0;
    virtual void process(const ProcessContext& context, const AudioBlock& input, AudioBlock& output) noexcept = 0;

    // Algorithmic latency only. The graph engine uses this for parallel-path analysis.
    [[nodiscard]] virtual std::size_t latencySamples() const noexcept { return 0; }
};
}
