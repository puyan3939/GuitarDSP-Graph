#include "guitardsp/circuit/BjtEbersMollSubcircuit.h"
#include "guitardsp/circuit/OperatingPointContinuation.h"
#include "guitardsp/graph/CompiledAudioGraph.h"
#include "guitardsp/graph/Graph.h"
#include "guitardsp/hq/PolyphaseOversampler.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <utility>
#include <vector>

namespace {
std::atomic<bool> trackAllocations{false};
std::atomic<std::size_t> allocationCount{0};
}

void* operator new(std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(size)) return p;
    throw std::bad_alloc{};
}

void* operator new[](std::size_t size) {
    if (trackAllocations.load(std::memory_order_relaxed))
        allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(size)) return p;
    throw std::bad_alloc{};
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

hq::ResistorSpec resistor(float ohms) {
    hq::ResistorSpec r{};
    r.resistanceOhms = ohms;
    r.tolerancePercent = 0.0f;
    return r;
}

class PassNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Readiness Pass"; }
    void prepare(const graph::PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        output.copyFrom(input, numSamples);
    }
};

class OversampledIdentityNode final : public graph::AudioNode {
public:
    std::string_view typeName() const noexcept override { return "Readiness Oversampled Identity"; }
    graph::NodeCategory category() const noexcept override { return graph::NodeCategory::drive; }

    void prepare(const graph::PrepareSpec& spec) override {
        oversampler_.prepare(spec.channels, spec.maximumBlockSize, 8, 47);
        oversampler_.reset();
    }
    void reset() noexcept override { oversampler_.reset(); }
    void process(const graph::AudioBuffer& input, graph::AudioBuffer& output,
                 int numSamples) noexcept override {
        oversampler_.process(input, output, numSamples,
                             [](int, float x) noexcept { return x; });
    }
    int latencySamples() const noexcept override { return oversampler_.latencySamples(); }

private:
    hq::PolyphaseOversampler oversampler_;
};

int peakIndex(const graph::AudioBuffer& buffer, int samples) {
    int index = 0;
    float peak = 0.0f;
    for (int i = 0; i < samples; ++i) {
        const float magnitude = std::abs(buffer.channel(0)[i]);
        if (magnitude > peak) {
            peak = magnitude;
            index = i;
        }
    }
    return index;
}
} // namespace

int main() {
    bool ok = true;

    // Standardized nonlinear operating-point continuation: the exact TS808-style
    // floating emitter-follower bias arrangement should settle without each pedal
    // owning a bespoke startup algorithm.
    {
        circuit::MnaCircuitEngine engine;
        const auto supply = engine.addNode();
        const auto vref = engine.addNode();
        const auto base = engine.addNode();
        const auto emitter = engine.addNode();
        const auto supplySource = engine.addVoltageSource(supply, circuit::ground, 0.0f);
        const auto vrefSource = engine.addVoltageSource(vref, circuit::ground, 0.0f);
        engine.addResistor(vref, base, resistor(510000.0f));
        engine.addResistor(emitter, circuit::ground, resistor(10000.0f));
        auto q = hq::component_presets::twoN3904();
        q.beta = 350.0f;
        q.nominalVbe = 0.62f;
        circuit::addBjtEbersMollSubcircuit(engine, supply, base, emitter, q);

        ok &= require(engine.prepare(48000.0), "operating-point fixture prepares");
        const circuit::OperatingPointSourceTarget targets[]{{supplySource, 9.0f},
                                                             {vrefSource, 4.5f}};
        const circuit::Node probes[]{base, emitter};
        circuit::OperatingPointOptions options{};
        options.maximumSettleSamples = 4096;
        options.requiredStableSamples = 16;
        options.steadyStateVoltageTolerance = 5.0e-6f;
        const auto result = circuit::establishOperatingPoint(engine, targets, probes, options);
        const float emitterVoltage = engine.voltage(emitter);
        ok &= require(result.converged && !result.singular,
                      "operating-point continuation reaches a stable nonlinear bias");
        ok &= require(emitterVoltage > 2.5f && emitterVoltage < 4.5f,
                      "operating-point continuation lands in emitter-follower bias range");
    }

    // Exact round-trip oversampler delay: the decimation phase is chosen so the
    // sampled combined FIR is centered on an integer base-rate sample.
    {
        const std::pair<int, int> configurations[]{{2, 23}, {4, 31}, {8, 47}, {16, 63}};
        for (const auto [factor, taps] : configurations) {
            graph::AudioBuffer input(1, 128), output(1, 128);
            input.clear();
            output.clear();
            input.channel(0)[0] = 1.0f;
            hq::PolyphaseOversampler oversampler;
            oversampler.prepare(1, 128, factor, taps);
            oversampler.process(input, output, 128,
                                [](int, float x) noexcept { return x; });
            const int expected = (taps - 1 - ((taps - 1) % factor)) / factor;
            ok &= require(oversampler.latencySamples() == expected,
                          "oversampler reports analytically phase-aligned round-trip delay");
            ok &= require(peakIndex(output, 128) == oversampler.latencySamples(),
                          "oversampler impulse peak matches reported latency");
        }
    }

    // PDC must use latency *after* node preparation. Two roots see the same impulse;
    // the direct root is delayed to the oversampled root before their common input is
    // mixed. A stale pre-prepare latency would place the direct delta at sample zero.
    {
        graph::Graph graph;
        const auto direct = graph.addNode(std::make_unique<PassNode>());
        const auto oversampled = graph.addNode(std::make_unique<OversampledIdentityNode>());
        const auto sum = graph.addNode(std::make_unique<PassNode>());
        graph.connect(direct, sum);
        graph.connect(oversampled, sum);

        graph::CompiledAudioGraph compiled;
        ok &= require(compiled.build(graph, 48000.0, 128, 1, graph::ProcessingQuality::high),
                      "PDC impulse fixture builds");
        ok &= require(compiled.totalLatencySamples() == 5,
                      "graph recompiles prepared node latency for PDC");

        graph::AudioBuffer input(1, 128), output(1, 128);
        input.clear();
        output.clear();
        input.channel(0)[0] = 1.0f;
        compiled.process(input, output, 128);
        ok &= require(peakIndex(output, 128) == compiled.totalLatencySamples(),
                      "direct and oversampled branches align at reported graph latency");
        ok &= require(output.channel(0)[compiled.totalLatencySamples()] > 1.0f,
                      "PDC-aligned direct impulse sums with oversampled impulse center");
    }

    // Realtime audit: after prepare, both the nonlinear MNA sample path and the
    // graph's very first callback must run without heap allocation. The graph test
    // intentionally does not perform a warm-up callback, catching unreserved
    // callback pointer tables.
    {
        circuit::MnaCircuitEngine engine;
        const auto input = engine.addNode();
        const auto clipped = engine.addNode();
        const auto source = engine.addVoltageSource(input, circuit::ground, 0.0f);
        engine.addResistor(input, clipped, resistor(10000.0f));
        engine.addDiode(clipped, circuit::ground, hq::component_presets::oneN4148());
        ok &= require(engine.prepare(48000.0), "realtime MNA fixture prepares");
        for (int i = 0; i < 32; ++i) engine.processSample(24, 1.0e-6f);

        allocationCount.store(0, std::memory_order_relaxed);
        trackAllocations.store(true, std::memory_order_relaxed);
        for (int i = 0; i < 512; ++i) {
            engine.setVoltageSource(source, 0.2f * std::sin(0.03f * static_cast<float>(i)));
            engine.processSample(24, 1.0e-6f);
        }
        trackAllocations.store(false, std::memory_order_relaxed);
        const auto mnaAllocations = allocationCount.load(std::memory_order_relaxed);
        ok &= require(mnaAllocations == 0,
                      "prepared nonlinear MNA processing performs zero heap allocations");

        graph::Graph graph;
        const auto a = graph.addNode(std::make_unique<PassNode>());
        const auto b = graph.addNode(std::make_unique<OversampledIdentityNode>());
        const auto c = graph.addNode(std::make_unique<PassNode>());
        graph.connect(a, c);
        graph.connect(b, c);
        graph::CompiledAudioGraph compiled;
        ok &= require(compiled.build(graph, 48000.0, 64, 1, graph::ProcessingQuality::high),
                      "realtime graph allocation fixture builds");
        graph::AudioBuffer inputBuffer(1, 64), outputBuffer(1, 64);
        for (int i = 0; i < 64; ++i)
            inputBuffer.channel(0)[i] = 0.1f * std::sin(0.05f * static_cast<float>(i));

        allocationCount.store(0, std::memory_order_relaxed);
        trackAllocations.store(true, std::memory_order_relaxed);
        compiled.process(inputBuffer, outputBuffer, 64);
        trackAllocations.store(false, std::memory_order_relaxed);
        const auto graphAllocations = allocationCount.load(std::memory_order_relaxed);
        ok &= require(graphAllocations == 0,
                      "first prepared graph audio callback performs zero heap allocations");
    }

    return ok ? 0 : 1;
}
