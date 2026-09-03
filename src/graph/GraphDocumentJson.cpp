#include "guitardsp/graph/GraphDocumentJson.h"

#include "guitardsp/circuit/JsonValue.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <utility>

namespace guitardsp::graph {

namespace {

using guitardsp::circuit::JsonParseError;
using guitardsp::circuit::JsonValue;
using guitardsp::circuit::parseJson;

std::string_view qualityToString(ProcessingQuality quality) noexcept {
    switch (quality) {
        case ProcessingQuality::eco: return "eco";
        case ProcessingQuality::live: return "live";
        case ProcessingQuality::high: return "high";
        case ProcessingQuality::studio: return "studio";
    }
    return "high";
}

ProcessingQuality qualityFromString(const std::string& text) noexcept {
    if (text == "eco") return ProcessingQuality::eco;
    if (text == "live") return ProcessingQuality::live;
    if (text == "studio") return ProcessingQuality::studio;
    return ProcessingQuality::high;
}

void writeEscapedString(std::string& out, std::string_view value) {
    out.push_back('"');
    for (const char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

// %.9g round-trips a 32-bit float exactly (9 significant decimal digits is
// always enough to disambiguate any two distinct float values).
void writeFloat(std::string& out, float value) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(value));
    out += buf;
}

void writeParameters(std::string& out, const std::vector<ParameterValueDocument>& parameters) {
    out.push_back('[');
    for (std::size_t i = 0; i < parameters.size(); ++i) {
        if (i != 0) out.push_back(',');
        out += "{\"id\":";
        writeEscapedString(out, parameters[i].id);
        out += ",\"value\":";
        writeFloat(out, parameters[i].value);
        out.push_back('}');
    }
    out.push_back(']');
}

void writeNode(std::string& out, const NodeDocument& node) {
    out += "{\"id\":" + std::to_string(node.id);
    out += ",\"typeId\":"; writeEscapedString(out, node.typeId);
    out += ",\"displayName\":"; writeEscapedString(out, node.displayName);
    out += ",\"x\":"; writeFloat(out, node.x);
    out += ",\"y\":"; writeFloat(out, node.y);
    out += ",\"bypassed\":"; out += node.bypassed ? "true" : "false";
    out += ",\"muted\":"; out += node.muted ? "true" : "false";
    out += ",\"parameters\":"; writeParameters(out, node.parameters);
    out.push_back('}');
}

void writeConnection(std::string& out, const ConnectionDocument& connection) {
    out += "{\"from\":" + std::to_string(connection.from);
    out += ",\"fromPort\":" + std::to_string(connection.fromPort);
    out += ",\"to\":" + std::to_string(connection.to);
    out += ",\"toPort\":" + std::to_string(connection.toPort);
    out.push_back('}');
}

void writeSceneNodeState(std::string& out, const SceneNodeState& state) {
    out += "{\"id\":" + std::to_string(state.id);
    out += ",\"bypassed\":"; out += state.bypassed ? "true" : "false";
    out += ",\"muted\":"; out += state.muted ? "true" : "false";
    out += ",\"parameters\":"; writeParameters(out, state.parameters);
    out.push_back('}');
}

void writeScene(std::string& out, const SceneDocument& scene) {
    out += "{\"name\":"; writeEscapedString(out, scene.name);
    out += ",\"crossfadeMs\":"; writeFloat(out, scene.crossfadeMs);
    out += ",\"nodes\":[";
    for (std::size_t i = 0; i < scene.nodes.size(); ++i) {
        if (i != 0) out.push_back(',');
        writeSceneNodeState(out, scene.nodes[i]);
    }
    out += "]}";
}

bool readParameters(const JsonValue& array, std::vector<ParameterValueDocument>& out, std::string* error) {
    if (array.isNull()) return true;
    if (!array.isArray()) {
        if (error != nullptr) *error = "'parameters' must be an array";
        return false;
    }
    out.reserve(array.items().size());
    for (const JsonValue& item : array.items()) {
        if (!item.isObject() || !item.has("id") || !item["id"].isString()) {
            if (error != nullptr) *error = "parameter entry must be an object with a string 'id'";
            return false;
        }
        ParameterValueDocument value;
        value.id = item["id"].asString();
        value.value = item["value"].asFloat();
        out.push_back(std::move(value));
    }
    return true;
}

bool readNode(const JsonValue& item, NodeDocument& out, std::string* error) {
    if (!item.isObject()) {
        if (error != nullptr) *error = "node entry must be a JSON object";
        return false;
    }
    if (!item.has("id") || !item["id"].isNumber()) {
        if (error != nullptr) *error = "node entry missing numeric 'id'";
        return false;
    }
    if (!item.has("typeId") || !item["typeId"].isString()) {
        if (error != nullptr) *error = "node entry missing string 'typeId'";
        return false;
    }
    out.id = static_cast<NodeId>(item["id"].asInt());
    out.typeId = item["typeId"].asString();
    out.displayName = item["displayName"].asString();
    out.x = item["x"].asFloat();
    out.y = item["y"].asFloat();
    out.bypassed = item["bypassed"].asBool(false);
    out.muted = item["muted"].asBool(false);
    return readParameters(item["parameters"], out.parameters, error);
}

bool readConnection(const JsonValue& item, ConnectionDocument& out, std::string* error) {
    if (!item.isObject() || !item.has("from") || !item["from"].isNumber() ||
        !item.has("to") || !item["to"].isNumber()) {
        if (error != nullptr) *error = "connection entry requires numeric 'from' and 'to'";
        return false;
    }
    out.from = static_cast<NodeId>(item["from"].asInt());
    out.fromPort = item["fromPort"].asInt(0);
    out.to = static_cast<NodeId>(item["to"].asInt());
    out.toPort = item["toPort"].asInt(0);
    return true;
}

bool readSceneNodeState(const JsonValue& item, SceneNodeState& out, std::string* error) {
    if (!item.isObject() || !item.has("id") || !item["id"].isNumber()) {
        if (error != nullptr) *error = "scene node entry missing numeric 'id'";
        return false;
    }
    out.id = static_cast<NodeId>(item["id"].asInt());
    out.bypassed = item["bypassed"].asBool(false);
    out.muted = item["muted"].asBool(false);
    return readParameters(item["parameters"], out.parameters, error);
}

bool readScene(const JsonValue& item, SceneDocument& out, std::string* error) {
    if (!item.isObject()) {
        if (error != nullptr) *error = "scene entry must be a JSON object";
        return false;
    }
    out.name = item["name"].asString();
    out.crossfadeMs = item["crossfadeMs"].asFloat(25.0f);
    const JsonValue& nodes = item["nodes"];
    if (nodes.isNull()) return true;
    if (!nodes.isArray()) {
        if (error != nullptr) *error = "scene 'nodes' must be an array";
        return false;
    }
    out.nodes.reserve(nodes.items().size());
    for (const JsonValue& nodeItem : nodes.items()) {
        SceneNodeState state;
        if (!readSceneNodeState(nodeItem, state, error)) return false;
        out.nodes.push_back(std::move(state));
    }
    return true;
}

} // namespace

std::string graphDocumentToJson(const GraphDocument& document) {
    std::string out;
    out += "{\"version\":" + std::to_string(document.version);
    out += ",\"name\":"; writeEscapedString(out, document.name);
    out += ",\"quality\":"; writeEscapedString(out, qualityToString(document.quality));

    out += ",\"nodes\":[";
    for (std::size_t i = 0; i < document.nodes.size(); ++i) {
        if (i != 0) out.push_back(',');
        writeNode(out, document.nodes[i]);
    }

    out += "],\"connections\":[";
    for (std::size_t i = 0; i < document.connections.size(); ++i) {
        if (i != 0) out.push_back(',');
        writeConnection(out, document.connections[i]);
    }

    out += "],\"scenes\":[";
    for (std::size_t i = 0; i < document.scenes.size(); ++i) {
        if (i != 0) out.push_back(',');
        writeScene(out, document.scenes[i]);
    }
    out += "]}";
    return out;
}

bool graphDocumentFromJson(std::string_view json, GraphDocument& outDocument, std::string* error) {
    JsonValue root;
    try {
        root = parseJson(json);
    } catch (const JsonParseError& e) {
        if (error != nullptr) *error = e.what();
        return false;
    }
    if (!root.isObject()) {
        if (error != nullptr) *error = "graph document must be a JSON object";
        return false;
    }

    GraphDocument document;
    document.version = root["version"].asInt(3);
    document.name = root.has("name") ? root["name"].asString() : "Untitled Rig";
    document.quality = root.has("quality") ? qualityFromString(root["quality"].asString())
                                            : ProcessingQuality::high;

    const JsonValue& nodes = root["nodes"];
    if (!nodes.isNull()) {
        if (!nodes.isArray()) {
            if (error != nullptr) *error = "'nodes' must be an array";
            return false;
        }
        document.nodes.reserve(nodes.items().size());
        for (const JsonValue& item : nodes.items()) {
            NodeDocument node;
            if (!readNode(item, node, error)) return false;
            document.nodes.push_back(std::move(node));
        }
    }

    const JsonValue& connections = root["connections"];
    if (!connections.isNull()) {
        if (!connections.isArray()) {
            if (error != nullptr) *error = "'connections' must be an array";
            return false;
        }
        document.connections.reserve(connections.items().size());
        for (const JsonValue& item : connections.items()) {
            ConnectionDocument connection;
            if (!readConnection(item, connection, error)) return false;
            document.connections.push_back(connection);
        }
    }

    const JsonValue& scenes = root["scenes"];
    if (!scenes.isNull()) {
        if (!scenes.isArray()) {
            if (error != nullptr) *error = "'scenes' must be an array";
            return false;
        }
        document.scenes.reserve(scenes.items().size());
        for (const JsonValue& item : scenes.items()) {
            SceneDocument scene;
            if (!readScene(item, scene, error)) return false;
            document.scenes.push_back(std::move(scene));
        }
    }

    outDocument = std::move(document);
    if (error != nullptr) error->clear();
    return true;
}

bool saveGraphDocumentToFile(const GraphDocument& document, const std::string& path, std::string* error) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        if (error != nullptr) *error = "unable to open file for writing: " + path;
        return false;
    }
    file << graphDocumentToJson(document);
    if (!file) {
        if (error != nullptr) *error = "failed writing graph document to file: " + path;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

bool loadGraphDocumentFromFile(const std::string& path, GraphDocument& outDocument, std::string* error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error != nullptr) *error = "unable to open graph document file: " + path;
        return false;
    }
    std::ostringstream contents;
    contents << file.rdbuf();
    return graphDocumentFromJson(contents.str(), outDocument, error);
}

} // namespace guitardsp::graph
