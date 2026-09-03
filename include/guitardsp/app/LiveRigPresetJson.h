#pragma once

// JSON serialization for LiveRigPreset, so a full LiveRig configuration
// (pedal/amp model, routing, every knob) can be saved as a named preset and
// restored later. Mirrors guitardsp::graph::GraphDocumentJson's design
// (issue #67): reuses guitardsp::circuit::JsonValue/parseJson for the read
// side rather than adding a second dependency-free JSON reader to the core,
// and a small direct string builder for writing. This keeps
// GuitarDSPGraphCore free of any new third-party dependency, per the core's
// JUCE-free/no-heavyweight-dependency constraint -- PresetStore.h layers
// directory/file management on top of these functions, still JUCE-free.
//
// Real-time contract: liveRigPresetToJson/liveRigPresetFromJson and the
// file-based helpers below only ever run on the control thread while a
// preset is being saved/loaded -- the same contract GraphDocumentJson's and
// NetlistLoader's loadFromFile/loadFromJson follow. Never call these from
// AudioNode::process, RealtimeAudioEngine::process, or anything reachable
// from the real-time audio callback.
//
// Forward-compatibility for future LiveRigSettings fields: the reader looks
// up JSON object keys by name and ignores anything it doesn't recognize, and
// every field has an explicit default applied when absent (mirroring
// LiveRigSettings' own default member initializers). That means an older
// reader tolerates a preset written by a newer version that adds settings
// fields, and a preset written before such a field existed loads fine into a
// newer reader.

#include "LiveRigPreset.h"

#include <string>
#include <string_view>

namespace guitardsp::app {

// Serializes `preset` to a JSON string. Control-thread only.
std::string liveRigPresetToJson(const LiveRigPreset& preset);

// Parses `json` into `outPreset`. Returns false and sets *error (if
// non-null) on malformed JSON; never throws and never crashes on malformed
// input -- `outPreset` is left unmodified on failure. Control-thread only.
bool liveRigPresetFromJson(std::string_view json, LiveRigPreset& outPreset, std::string* error = nullptr);

// Convenience file wrappers around the two functions above, mirroring
// GraphDocumentJson's saveGraphDocumentToFile/loadGraphDocumentFromFile.
// Control-thread only.
bool savePresetToFile(const LiveRigPreset& preset, const std::string& path, std::string* error = nullptr);
bool loadPresetFromFile(const std::string& path, LiveRigPreset& outPreset, std::string* error = nullptr);

} // namespace guitardsp::app
