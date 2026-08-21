#include "guitardsp/graph/CompiledAudioGraph.h"
#include "guitardsp/graph/UtilityNodes.h"
#include <cmath>
#include <iostream>
#include <memory>

using namespace guitardsp::graph;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}
float sampleAt(const AudioBuffer& b, int i) { return b.channel(0)[i]; }
class CountingTap final : public TapSink {
public:
    void push(const AudioBuffer& block, int numSamples) noexcept override {
        ++calls; last = numSamples > 0 ? block.channel(0)[0] : 0.0f;
    }
    int calls = 0; float last = 0.0f;
};
}

int main() {
    bool ok = true;
    constexpr int block = 32;

    {
        Graph g;
        const auto split = g.addNode(std::make_unique<SplitNode>());
        const auto a = g.addNode(std::make_unique<GainNode>(0.5f));
        const auto b = g.addNode(std::make_unique<GainNode>(0.25f));
        const auto merge = g.addNode(std::make_unique<MergeNode>());
        g.connect(split, a); g.connect(split, b); g.connect(a, merge); g.connect(b, merge);
        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block, 2), "runtime builds split/merge graph");
        AudioBuffer in(2, block), out(2, block);
        for (int i = 0; i < block; ++i) { in.channel(0)[i] = 1.0f; in.channel(1)[i] = 1.0f; }
        runtime.process(in, out, block);
        ok &= require(std::abs(sampleAt(out, 5) - 0.75f) < 1.0e-6f, "parallel branches sum exactly");
    }

    {
        Graph g;
        const auto source = g.addNode(std::make_unique<SplitNode>());
        const auto dry = g.addNode(std::make_unique<GainNode>(1.0f));
        const auto delayed = g.addNode(std::make_unique<DelayNode>(5));
        const auto merge = g.addNode(std::make_unique<MergeNode>());
        g.connect(source, dry); g.connect(source, delayed); g.connect(dry, merge); g.connect(delayed, merge);
        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block, 2), "runtime builds PDC graph");
        ok &= require(runtime.totalLatencySamples() == 5, "graph reports five sample latency");
        AudioBuffer in(2, block), out(2, block); in.clear(); in.channel(0)[0] = 1.0f; in.channel(1)[0] = 1.0f;
        runtime.process(in, out, block);
        ok &= require(std::abs(sampleAt(out, 0)) < 1.0e-6f, "PDC removes early dry impulse");
        ok &= require(std::abs(sampleAt(out, 5) - 2.0f) < 1.0e-6f, "PDC aligns and sums parallel impulse");
    }

    {
        Graph g;
        const auto invert = g.addNode(std::make_unique<PolarityNode>(true));
        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block, 2), "runtime builds polarity graph");
        AudioBuffer in(2, block), out(2, block); in.clear(); in.channel(0)[3] = 0.4f; in.channel(1)[3] = -0.2f;
        runtime.process(in, out, block);
        ok &= require(std::abs(out.channel(0)[3] + 0.4f) < 1.0e-6f && std::abs(out.channel(1)[3] - 0.2f) < 1.0e-6f,
                      "polarity node inverts stereo signal");
    }

    {
        Graph g;
        auto gainNode = std::make_unique<GainNode>(0.25f);
        auto* gain = gainNode.get();
        g.addNode(std::move(gainNode));
        CompiledAudioGraph runtime; ok &= require(runtime.build(g, 48000.0, block, 2), "runtime builds bypass graph");
        AudioBuffer in(2, block), out(2, block); in.clear(); in.channel(0)[0] = 1.0f;
        runtime.process(in, out, block);
        ok &= require(std::abs(out.channel(0)[0] - 0.25f) < 1.0e-6f, "node processes before bypass");
        gain->setBypassed(true); runtime.process(in, out, block);
        ok &= require(std::abs(out.channel(0)[0] - 1.0f) < 1.0e-6f, "node bypass is realtime safe");
        gain->setMuted(true); runtime.process(in, out, block);
        ok &= require(std::abs(out.channel(0)[0]) < 1.0e-7f, "mute overrides bypass");
    }

    {
        CountingTap sink;
        Graph g; g.addNode(std::make_unique<TapNode>(&sink));
        CompiledAudioGraph runtime; ok &= require(runtime.build(g, 48000.0, block, 2), "runtime builds tap graph");
        AudioBuffer in(2, block), out(2, block); in.clear(); in.channel(0)[0] = 0.33f;
        runtime.process(in, out, block);
        ok &= require(sink.calls == 1 && std::abs(sink.last - 0.33f) < 1.0e-6f, "tap observes signal without altering it");
    }

    return ok ? 0 : 1;
}
