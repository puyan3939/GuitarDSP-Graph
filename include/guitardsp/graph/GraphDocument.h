#pragma once

#include "AudioNode.h"
#include <string>
#include <vector>

namespace guitardsp::graph {

struct ParameterValueDocument { std::string id; float value = 0.0f; };
struct NodeDocument {
    NodeId id = 0; std::string typeId; std::string displayName; float x = 0.0f; float y = 0.0f;
    bool bypassed = false; bool muted = false; std::vector<ParameterValueDocument> parameters;
};
struct ConnectionDocument { NodeId from = 0; int fromPort = 0; NodeId to = 0; int toPort = 0; };
struct SceneNodeState { NodeId id = 0; bool bypassed = false; bool muted = false; std::vector<ParameterValueDocument> parameters; };
struct SceneDocument { std::string name; float crossfadeMs = 25.0f; std::vector<SceneNodeState> nodes; };
struct GraphDocument {
    int version = 3; std::string name = "Untitled Rig"; ProcessingQuality quality = ProcessingQuality::high;
    std::vector<NodeDocument> nodes; std::vector<ConnectionDocument> connections; std::vector<SceneDocument> scenes;
};

} // namespace guitardsp::graph
