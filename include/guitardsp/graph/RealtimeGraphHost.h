#pragma once

#include "CompiledAudioGraph.h"
#include "GraphBuilder.h"
#include "NodeRegistry.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace guitardsp::graph {

struct PreparedGraph {
    Graph graph;
    CompiledAudioGraph runtime;
    std::unordered_map<NodeId, NodeId> documentToRuntimeId;
};

class RealtimeGraphHost {
public:
    RealtimeGraphHost() = default;
    ~RealtimeGraphHost();
    RealtimeGraphHost(const RealtimeGraphHost&) = delete;
    RealtimeGraphHost& operator=(const RealtimeGraphHost&) = delete;

    [[nodiscard]] std::unique_ptr<PreparedGraph> prepare(
        const GraphDocument& document,
        const NodeRegistry& registry,
        double sampleRate,
        int maximumBlockSize,
        int channels,
        ProcessingQuality quality = ProcessingQuality::high) const;

    void submit(std::unique_ptr<PreparedGraph> prepared) noexcept;
    void process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept;
    std::size_t collectRetired() noexcept;

    [[nodiscard]] bool hasActiveGraph() const noexcept { return active_.load(std::memory_order_acquire) != nullptr; }
    [[nodiscard]] bool hasPendingGraph() const noexcept { return pending_.load(std::memory_order_acquire) != nullptr; }
    [[nodiscard]] std::uint64_t generation() const noexcept { return generation_.load(std::memory_order_acquire); }

private:
    static constexpr std::size_t retireCapacity = 32;

    class RetireQueue {
    public:
        bool tryPush(PreparedGraph* pointer) noexcept;
        PreparedGraph* tryPop() noexcept;
        [[nodiscard]] bool canPush() const noexcept;
    private:
        std::array<PreparedGraph*, retireCapacity> data_{};
        std::atomic<std::size_t> write_{0};
        std::atomic<std::size_t> read_{0};
    };

    void adoptPendingAtBlockBoundary() noexcept;

    std::atomic<PreparedGraph*> active_{nullptr};
    std::atomic<PreparedGraph*> pending_{nullptr};
    RetireQueue retired_;
    std::atomic<std::uint64_t> generation_{0};
};

} // namespace guitardsp::graph
