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
}

int main() {
    bool ok = true;
    constexpr std::size_t block = 32;

    // Split by topology: one source feeds two gain branches, then a merge node sums them.
    {
        Graph g;
        const auto source = g.addNode(std::make_unique<PassthroughNode>());
        const auto a = g.addNode(std::make_unique<GainNode>(0.5f));
        const auto b = g.addNode(std::make_unique<GainNode>(0.25f));
        const auto merge = g.addNode(std::make_unique<PassthroughNode>());
        g.connect(source, a); g.connect(source, b); g.connect(a, merge); g.connect(b, merge);

        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block), "runtime builds split/merge graph");
        AudioBuffer in(2, static_cast<int>(block)), out(2, static_cast<int>(block));
        for (std::size_t i = 0; i < block; ++i) { in.channel(0)[i] = 1.0f; in.channel(1)[i] = 1.0f; }
        runtime.process(in, out, block);
        ok &= require(std::abs(sampleAt(out, 5) - 0.75f) < 1.0e-6f, "parallel branches sum exactly");
    }

    // Automatic PDC: dry branch is delayed to match a 5-sample latency branch before merge.
    {
        Graph g;
        const auto source = g.addNode(std::make_unique<PassthroughNode>());
        const auto dry = g.addNode(std::make_unique<GainNode>(1.0f));
        const auto delayed = g.addNode(std::make_unique<DelayNode>(5));
        const auto merge = g.addNode(std::make_unique<PassthroughNode>());
        g.connect(source, dry); g.connect(source, delayed); g.connect(dry, merge); g.connect(delayed, merge);

        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block), "runtime builds PDC graph");
        ok &= require(runtime.totalLatencySamples() == 5, "graph reports five sample latency");

        AudioBuffer in(2, static_cast<int>(block)), out(2, static_cast<int>(block));
        in.clear(); in.channel(0)[0] = 1.0f; in.channel(1)[0] = 1.0f;
        runtime.process(in, out, block);
        ok &= require(std::abs(sampleAt(out, 0)) < 1.0e-6f, "PDC removes early dry impulse");
        ok &= require(std::abs(sampleAt(out, 5) - 2.0f) < 1.0e-6f, "PDC aligns and sums parallel impulse");
    }

    // Polarity is a first-class utility node for phase-sensitive parallel rigs.
    {
        Graph g;
        const auto invert = g.addNode(std::make_unique<PolarityNode>(true));
        CompiledAudioGraph runtime;
        ok &= require(runtime.build(g, 48000.0, block), "runtime builds polarity graph");
        AudioBuffer in(2, static_cast<int>(block)), out(2, static_cast<int>(block));
        in.clear(); in.channel(0)[3] = 0.4f; in.channel(1)[3] = -0.2f;
        runtime.process(in, out, block);
        ok &= require(std::abs(out.channel(0)[3] + 0.4f) < 1.0e-6f && std::abs(out.channel(1)[3] - 0.2f) < 1.0e-6f,
                      "polarity node inverts stereo signal");
    }

    return ok ? 0 : 1;
}
