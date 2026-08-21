#include "guitardsp/graph/RealtimeGraphHost.h"

namespace guitardsp::graph {

RealtimeGraphHost::~RealtimeGraphHost() {
    delete pending_.exchange(nullptr, std::memory_order_acq_rel);
    delete active_.exchange(nullptr, std::memory_order_acq_rel);
    collectRetired();
}

std::unique_ptr<PreparedGraph> RealtimeGraphHost::prepare(
    const GraphDocument& document,
    const NodeRegistry& registry,
    double sampleRate,
    int maximumBlockSize,
    int channels,
    ProcessingQuality quality) const {
    auto prepared = std::make_unique<PreparedGraph>();
    auto build = buildGraphFromDocument(document, registry, prepared->graph);
    if (!build.ok) return nullptr;
    prepared->documentToRuntimeId = std::move(build.documentToRuntimeId);
    if (!prepared->runtime.build(prepared->graph, sampleRate, maximumBlockSize, channels, quality)) return nullptr;
    return prepared;
}

void RealtimeGraphHost::submit(std::unique_ptr<PreparedGraph> prepared) noexcept {
    PreparedGraph* incoming = prepared.release();
    PreparedGraph* replaced = pending_.exchange(incoming, std::memory_order_acq_rel);
    delete replaced;
}

bool RealtimeGraphHost::RetireQueue::canPush() const noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1U) % retireCapacity;
    return next != read_.load(std::memory_order_acquire);
}

bool RealtimeGraphHost::RetireQueue::tryPush(PreparedGraph* pointer) noexcept {
    if (pointer == nullptr) return true;
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1U) % retireCapacity;
    if (next == read_.load(std::memory_order_acquire)) return false;
    data_[write] = pointer;
    write_.store(next, std::memory_order_release);
    return true;
}

PreparedGraph* RealtimeGraphHost::RetireQueue::tryPop() noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return nullptr;
    PreparedGraph* pointer = data_[read];
    data_[read] = nullptr;
    read_.store((read + 1U) % retireCapacity, std::memory_order_release);
    return pointer;
}

void RealtimeGraphHost::adoptPendingAtBlockBoundary() noexcept {
    PreparedGraph* candidate = pending_.load(std::memory_order_acquire);
    if (candidate == nullptr) return;

    PreparedGraph* current = active_.load(std::memory_order_relaxed);
    if (current != nullptr && !retired_.canPush()) return;

    candidate = pending_.exchange(nullptr, std::memory_order_acq_rel);
    if (candidate == nullptr) return;
    current = active_.exchange(candidate, std::memory_order_acq_rel);
    if (current != nullptr) {
        const bool queued = retired_.tryPush(current);
        (void) queued;
    }
    generation_.fetch_add(1, std::memory_order_release);
}

void RealtimeGraphHost::process(const AudioBuffer& input, AudioBuffer& output, int numSamples) noexcept {
    adoptPendingAtBlockBoundary();
    if (PreparedGraph* active = active_.load(std::memory_order_acquire)) {
        active->runtime.process(input, output, numSamples);
    } else {
        output.copyFrom(input, numSamples);
    }
}

bool RealtimeGraphHost::setCategoryParameter(NodeCategory category,
                                             std::size_t parameterIndex,
                                             float value) noexcept {
    bool changed = false;
    auto apply = [&](PreparedGraph* prepared) noexcept {
        if (prepared == nullptr) return;
        for (const NodeId id : prepared->graph.schedule()) {
            if (auto* node = prepared->graph.node(id);
                node != nullptr && node->category() == category) {
                changed = node->setParameterValue(parameterIndex, value) || changed;
            }
        }
    };

    PreparedGraph* active = active_.load(std::memory_order_acquire);
    apply(active);
    PreparedGraph* pending = pending_.load(std::memory_order_acquire);
    if (pending != active) apply(pending);
    return changed;
}

std::size_t RealtimeGraphHost::collectRetired() noexcept {
    std::size_t count = 0;
    while (PreparedGraph* pointer = retired_.tryPop()) {
        delete pointer;
        ++count;
    }
    return count;
}

} // namespace guitardsp::graph
