// Regression tests for issue #67: GraphDocument <-> JSON serialization, so a
// rig preset (nodes/connections/scenes) can be saved and restored bit for
// bit. Covers a realistic multi-stage rig round-tripping exactly, and that
// malformed/structurally-invalid JSON is rejected with an error instead of
// crashing or silently producing a wrong document.
#include "guitardsp/graph/GraphBuilder.h"
#include "guitardsp/graph/GraphDocumentJson.h"
#include "guitardsp/graph/NodeRegistry.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace guitardsp::graph;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

bool nearlyEqual(float a, float b) { return std::abs(a - b) < 1.0e-5f; }

bool parametersEqual(const std::vector<ParameterValueDocument>& a, const std::vector<ParameterValueDocument>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].id != b[i].id || !nearlyEqual(a[i].value, b[i].value)) return false;
    return true;
}

bool documentsEqual(const GraphDocument& a, const GraphDocument& b) {
    if (a.version != b.version || a.name != b.name || a.quality != b.quality) return false;
    if (a.nodes.size() != b.nodes.size()) return false;
    for (std::size_t i = 0; i < a.nodes.size(); ++i) {
        const auto& na = a.nodes[i];
        const auto& nb = b.nodes[i];
        if (na.id != nb.id || na.typeId != nb.typeId || na.displayName != nb.displayName) return false;
        if (!nearlyEqual(na.x, nb.x) || !nearlyEqual(na.y, nb.y)) return false;
        if (na.bypassed != nb.bypassed || na.muted != nb.muted) return false;
        if (!parametersEqual(na.parameters, nb.parameters)) return false;
    }
    if (a.connections.size() != b.connections.size()) return false;
    for (std::size_t i = 0; i < a.connections.size(); ++i) {
        const auto& ca = a.connections[i];
        const auto& cb = b.connections[i];
        if (ca.from != cb.from || ca.fromPort != cb.fromPort || ca.to != cb.to || ca.toPort != cb.toPort)
            return false;
    }
    if (a.scenes.size() != b.scenes.size()) return false;
    for (std::size_t i = 0; i < a.scenes.size(); ++i) {
        const auto& sa = a.scenes[i];
        const auto& sb = b.scenes[i];
        if (sa.name != sb.name || !nearlyEqual(sa.crossfadeMs, sb.crossfadeMs)) return false;
        if (sa.nodes.size() != sb.nodes.size()) return false;
        for (std::size_t j = 0; j < sa.nodes.size(); ++j) {
            const auto& na = sa.nodes[j];
            const auto& nb = sb.nodes[j];
            if (na.id != nb.id || na.bypassed != nb.bypassed || na.muted != nb.muted) return false;
            if (!parametersEqual(na.parameters, nb.parameters)) return false;
        }
    }
    return true;
}

// TS808 -> reference amp -> cabinet chain, the exact rig quoted in the issue,
// plus a scene, non-default quality and a bypassed/muted node so every field
// GraphDocument carries gets exercised by the round trip.
GraphDocument buildSampleRig() {
    GraphDocument doc;
    doc.version = 3;
    doc.name = "TS808 into amp and cab";
    doc.quality = ProcessingQuality::studio;
    doc.nodes = {
        NodeDocument{10, "drive.ts808_circuit_hq", "Drive", -0.5f, 0.0f, false, false, {{"drive", 0.6f}}},
        NodeDocument{20, "amp.reference_hq", "Amp", 0.0f, 0.0f, false, false, {{"gain", 0.7f}}},
        NodeDocument{30, "cab.chain_hq", "Cab", 0.5f, 0.0f, true, false, {}},
    };
    doc.connections = {
        ConnectionDocument{10, 0, 20, 0},
        ConnectionDocument{20, 0, 30, 0},
    };
    SceneDocument scene;
    scene.name = "Lead";
    scene.crossfadeMs = 40.0f;
    scene.nodes.push_back(SceneNodeState{10, false, true, {{"drive", 0.9f}}});
    doc.scenes.push_back(scene);
    return doc;
}
} // namespace

int main() {
    bool ok = true;

    {
        const GraphDocument original = buildSampleRig();
        const std::string json = graphDocumentToJson(original);

        GraphDocument restored;
        std::string error;
        ok &= require(graphDocumentFromJson(json, restored, &error), "sample rig JSON round-trips without error");
        ok &= require(error.empty(), "successful parse clears the error string");
        ok &= require(documentsEqual(original, restored), "restored document matches the original field-for-field");

        auto registry = NodeRegistry::createBuiltins();
        Graph graph;
        const auto result = buildGraphFromDocument(restored, registry, graph);
        ok &= require(result.ok, "restored document still builds a valid graph (typeIds resolved)");
    }

    {
        GraphDocument restored;
        std::string error;
        ok &= require(!graphDocumentFromJson("{ this is not valid json", restored, &error),
                      "malformed JSON syntax is rejected instead of crashing");
        ok &= require(!error.empty(), "malformed JSON syntax reports a non-empty error");
    }

    {
        GraphDocument restored;
        std::string error;
        ok &= require(!graphDocumentFromJson("[1, 2, 3]", restored, &error),
                      "a JSON array at the top level is rejected (not an object)");
        ok &= require(!error.empty(), "wrong top-level shape reports a non-empty error");
    }

    {
        GraphDocument restored;
        std::string error;
        ok &= require(!graphDocumentFromJson(R"({"nodes":[{"displayName":"no id or typeId"}]})", restored, &error),
                      "a node entry missing required id/typeId is rejected");
        ok &= require(!error.empty(), "missing required node fields reports a non-empty error");
    }

    {
        GraphDocument restored;
        std::string error;
        ok &= require(!graphDocumentFromJson(R"({"connections":[{"from":1}]})", restored, &error),
                      "a connection entry missing required 'to' is rejected");
        ok &= require(!error.empty(), "missing required connection fields reports a non-empty error");
    }

    {
        // An empty document is a legitimate, minimal rig: no crash, no error.
        GraphDocument restored;
        std::string error;
        ok &= require(graphDocumentFromJson("{}", restored, &error), "an empty JSON object parses as an empty document");
        ok &= require(restored.nodes.empty() && restored.connections.empty() && restored.scenes.empty(),
                      "empty document has no nodes/connections/scenes");
    }

    return ok ? 0 : 1;
}
