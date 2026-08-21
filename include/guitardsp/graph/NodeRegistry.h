#pragma once

#include "AudioNode.h"
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace guitardsp::graph {

class NodeRegistry {
public:
    using Factory = std::function<std::unique_ptr<AudioNode>()>;

    bool registerType(std::string typeId, Factory factory) {
        if (typeId.empty() || !factory || factories_.contains(typeId)) return false;
        factories_.emplace(std::move(typeId), std::move(factory)); return true;
    }

    [[nodiscard]] std::unique_ptr<AudioNode> create(std::string_view typeId) const {
        const auto it = factories_.find(std::string(typeId));
        return it == factories_.end() ? nullptr : it->second();
    }

    [[nodiscard]] std::vector<std::string> registeredTypes() const {
        std::vector<std::string> result; result.reserve(factories_.size());
        for (const auto& [name, factory] : factories_) { (void)factory; result.push_back(name); }
        return result;
    }

    static NodeRegistry createBuiltins();

private:
    std::unordered_map<std::string, Factory> factories_;
};

} // namespace guitardsp::graph
