#pragma once

// Directory-backed store for LiveRigPreset presets (see LiveRigPreset.h /
// LiveRigPresetJson.h): one JSON file per preset under a caller-supplied
// directory. Deliberately JUCE-free -- resolving the *real* directory (e.g.
// juce::File::getSpecialLocation(userApplicationDataDirectory)) is the
// caller's job (GuitarDSPApp/Main.cpp), the same split GraphDocumentJson's
// file helpers already follow, so this class stays testable via ctest
// without pulling in JUCE, and portable to any future non-JUCE host.
//
// Real-time contract: every member here does blocking filesystem I/O
// (directory creation/listing, file read/write/remove) and must only run on
// the control/message thread -- never from AudioNode::process,
// RealtimeAudioEngine::process, or anything reachable from the real-time
// audio callback. Same contract as GraphDocumentJson's file helpers and
// NetlistLoader::loadFromFile.

#include "LiveRigPreset.h"

#include <string>
#include <vector>

namespace guitardsp::app {

struct PresetSummary {
    std::string name;
    std::string fileName;
};

class PresetStore {
public:
    explicit PresetStore(std::string directory);

    // Lists presets currently on disk, sorted by display name. Returns an
    // empty list (not an error) if the directory doesn't exist yet -- a
    // fresh install simply has no presets.
    [[nodiscard]] std::vector<PresetSummary> list() const;

    // Creates the preset directory if needed and writes `preset` as
    // "<sanitizedName>.json", overwriting any existing preset with the same
    // (sanitized) name. Returns false and sets *error on failure, including
    // when preset.name is empty after sanitization.
    bool save(const LiveRigPreset& preset, std::string* error = nullptr) const;

    bool load(const std::string& name, LiveRigPreset& outPreset, std::string* error = nullptr) const;

    bool remove(const std::string& name, std::string* error = nullptr) const;

    [[nodiscard]] const std::string& directory() const noexcept { return directory_; }

    // Resolves `name` to the on-disk path save()/load()/remove() would use,
    // without performing any I/O. Exposed mainly for tests; sanitizes `name`
    // into a safe filename component the same way the I/O methods do
    // internally (no path separators/traversal can escape directory()).
    [[nodiscard]] std::string filePathForName(const std::string& name) const;

private:
    std::string directory_;
};

} // namespace guitardsp::app
