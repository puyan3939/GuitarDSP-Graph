#pragma once

#include "AudioNode.h"
#include <string>
#include <vector>

namespace guitardsp::graph {

struct NodeDocument {
    NodeId id = 0;
    std::string typeId;
    std::string displayName;
    float x = 0.0f;
    float y = 0.0f;
    bool bypassed = false;
    bool muted = false;
};

struct ConnectionDocument {
    NodeId from = 0;
    NodeId to = 0;
};

struct GraphDocument {
    int version = 1;
    std::string name = "Untitled Rig";
    ProcessingQuality quality = ProcessingQuality::high;
    std::vector<NodeDocument> nodes;
    std::vector<ConnectionDocument> connections;
};

} // namespace guitardsp::graph
