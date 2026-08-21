#pragma once

#include "AudioBuffer.h"
#include "Parameter.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace guitardsp::graph {

using NodeId = std::uint32_t;

enum class NodeCategory {
    io, utility, dynamics, drive, amp, cab, pitch, modulation, time, analysis
};

enum class ProcessingQuality {
    eco, live, high, studio
};

struct PrepareSpec {
    double sampleRate = 48000.0;
    int maximumBlockSize = 512;
    int channels = 2;
    ProcessingQuality quality = ProcessingQuality::high;
};

class AudioNode {
public:
    virtual ~AudioNode() = default;

    [[nodiscard]] virtual std::string_view typeName() const noexcept = 0;
    [[nodiscard]] virtual NodeCategory category() const noexcept { return NodeCategory::utility; }
    virtual void prepare(const PrepareSpec& spec) = 0;
    virtual void reset() noexcept = 0;
    virtual void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept = 0;

    [[nodiscard]] virtual int latencySamples() const noexcept { return 0; }
    [[nodiscard]] virtual int tailSamples() const noexcept { return 0; }

    [[nodiscard]] virtual std::size_t parameterCount() const noexcept { return 0; }
    [[nodiscard]] virtual ParameterDescriptor parameterDescriptor(std::size_t) const noexcept { return {}; }
    [[nodiscard]] virtual float parameterValue(std::size_t) const noexcept { return 0.0f; }
    virtual bool setParameterValue(std::size_t, float) noexcept { return false; }

    [[nodiscard]] int parameterIndex(std::string_view id) const noexcept {
        for (std::size_t i = 0; i < parameterCount(); ++i)
            if (parameterDescriptor(i).id == id) return static_cast<int>(i);
        return -1;
    }

    void setBypassed(bool value) noexcept { bypassed_.store(value, std::memory_order_relaxed); }
    [[nodiscard]] bool isBypassed() const noexcept { return bypassed_.load(std::memory_order_relaxed); }
    void setMuted(bool value) noexcept { muted_.store(value, std::memory_order_relaxed); }
    [[nodiscard]] bool isMuted() const noexcept { return muted_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool> bypassed_{false};
    std::atomic<bool> muted_{false};
};

} // namespace guitardsp::graph
