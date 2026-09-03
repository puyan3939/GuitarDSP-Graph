#pragma once

#include "CompiledAudioGraph.h"
#include "GraphBuilder.h"
#include "NodeRegistry.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

namespace guitardsp::graph {

// Transparent hash/equal so typeIdToRuntimeId below can be looked up with a
// std::string_view (e.g. from the audio thread) without allocating a
// std::string just to probe the map.
struct TransparentStringHash {
    using is_transparent = void;
    [[nodiscard]] std::size_t operator()(std::string_view value) const noexcept {
        return std::hash<std::string_view>{}(value);
    }
};

struct PreparedGraph {
    Graph graph;
    CompiledAudioGraph runtime;
    std::unordered_map<NodeId, NodeId> documentToRuntimeId;
    // Maps each document node's NodeRegistry typeId (e.g. "cab.chain_hq") to
    // its runtime NodeId, for tap/introspection lookups that only know a
    // node by its type rather than its document id. If a typeId appears more
    // than once in the document, this keeps the first occurrence -- fine for
    // LiveRig's rigs, where every node type is a singleton per graph.
    std::unordered_map<std::string, NodeId, TransparentStringHash, std::equal_to<>> typeIdToRuntimeId;
};

// Populates PreparedGraph::typeIdToRuntimeId from a document, given
// prepared.documentToRuntimeId is already populated. Shared by every
// PreparedGraph builder (RealtimeGraphHost::prepare(), LiveRig::
// prepareLiveRig(), which constructs a PreparedGraph directly rather than
// going through prepare()).
void indexTypeIds(const GraphDocument& document, PreparedGraph& prepared);

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

    // Audio thread safe: resolves typeId against the currently active
    // prepared graph (not the pending one) and returns that node's output
    // buffer for the block just processed, or nullptr if there's no active
    // graph or no node of that type. Same lifetime caveat as
    // CompiledAudioGraph::nodeOutput() -- valid only until the next
    // process() call.
    [[nodiscard]] const AudioBuffer* nodeOutputByTypeId(std::string_view typeId, int port = 0) const noexcept;

    // Message/control thread only. Prepared graph topology is immutable, and the
    // supported node parameters are atomic. Apply edits to both the active and
    // queued graphs so a block-boundary graph swap cannot discard a knob change.
    bool setCategoryParameter(NodeCategory category,
                              std::size_t parameterIndex,
                              float value) noexcept;
    bool setTypeParameter(std::string_view typeName,
                          std::size_t parameterIndex,
                          float value) noexcept;

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
