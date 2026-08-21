#include "guitardsp/graph/Graph.h"
#include <cstdlib>
#include <iostream>
#include <memory>

using namespace guitardsp::graph;

namespace {
class TestNode final : public AudioNode {
public:
    explicit TestNode(int latency = 0) : latency_(latency) {}
    std::string_view typeName() const noexcept override { return "TestNode"; }
    void prepare(const PrepareSpec&) override {}
    void reset() noexcept override {}
    void process(const AudioBuffer& input, AudioBuffer& output, int n) noexcept override { output.copyFrom(input, n); }
    int latencySamples() const noexcept override { return latency_; }
private:
    int latency_ = 0;
};

bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}
}

int main() {
    bool ok = true;
    {
        Graph graph;
        const auto input = graph.addNode(std::make_unique<TestNode>(0));
        const auto drive = graph.addNode(std::make_unique<TestNode>(8));
        const auto amp = graph.addNode(std::make_unique<TestNode>(16));
        const auto output = graph.addNode(std::make_unique<TestNode>(0));
        ok &= require(graph.connect(input, drive), "connect input-drive");
        ok &= require(graph.connect(drive, amp), "connect drive-amp");
        ok &= require(graph.connect(amp, output), "connect amp-output");
        const auto result = graph.compile();
        ok &= require(result.ok, "linear graph compiles");
        ok &= require(graph.schedule().size() == 4, "linear graph schedules every node");
        ok &= require(graph.cumulativeLatencySamples(output).value_or(0) == 24, "linear graph latency accumulates");
    }
    {
        Graph graph;
        const auto input = graph.addNode(std::make_unique<TestNode>());
        const auto fast = graph.addNode(std::make_unique<TestNode>(4));
        const auto slow = graph.addNode(std::make_unique<TestNode>(64));
        const auto merge = graph.addNode(std::make_unique<TestNode>(2));
        graph.connect(input, fast); graph.connect(input, slow); graph.connect(fast, merge); graph.connect(slow, merge);
        const auto result = graph.compile();
        ok &= require(result.ok, "parallel graph compiles");
        ok &= require(graph.cumulativeLatencySamples(merge).value_or(0) == 66, "merge follows slowest upstream path");
        ok &= require(graph.maximumGraphLatencySamples() == 66, "maximum graph latency tracked");
    }
    {
        Graph graph;
        const auto a = graph.addNode(std::make_unique<TestNode>());
        const auto b = graph.addNode(std::make_unique<TestNode>());
        const auto c = graph.addNode(std::make_unique<TestNode>());
        graph.connect(a, b); graph.connect(b, c); graph.connect(c, a);
        const auto result = graph.compile();
        ok &= require(!result.ok, "cycle rejected");
        ok &= require(graph.schedule().empty(), "cyclic graph has no execution schedule");
    }
    {
        Graph graph;
        const auto a = graph.addNode(std::make_unique<TestNode>());
        const auto b = graph.addNode(std::make_unique<TestNode>());
        ok &= require(graph.connect(a, b), "first connection accepted");
        ok &= require(!graph.connect(a, b), "duplicate connection rejected");
        ok &= require(!graph.connect(a, a), "self connection rejected");
        ok &= require(!graph.connect(9999, a), "missing source rejected");
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
