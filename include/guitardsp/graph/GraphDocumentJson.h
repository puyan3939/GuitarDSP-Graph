#pragma once

// JSON serialization for GraphDocument (NodeDocument / ConnectionDocument /
// SceneDocument), so a rig built via the generic routing API (GraphBuilder,
// RigTopologyBuilder) can be saved as a preset and restored later with its
// node typeIds, parameter values and connections intact.
//
// Reuses guitardsp::circuit::JsonValue/parseJson (see
// guitardsp/circuit/JsonValue.h) for the read side rather than adding a
// second dependency-free JSON reader to the core; JsonValue.h only
// implements parsing, so writing is a small direct string builder here.
// This keeps GuitarDSPGraphCore free of any new third-party dependency, per
// the core's JUCE-free/no-heavyweight-dependency constraint.
//
// Real-time contract: graphDocumentToJson/graphDocumentFromJson and the
// file-based helpers below only ever run on the control thread while a rig
// preset is being saved/loaded -- the same contract NetlistLoader's
// loadFromFile/loadFromJson/prepare follow for circuit netlists. Never call
// these from AudioNode::process or anything reachable from the real-time
// audio callback.
//
// Forward-compatibility for future fields (e.g. the stable asset-ID
// references proposed for cabinet IRs): the reader looks up JSON object
// keys by name and ignores anything it doesn't recognize, and every field
// read from a node/connection/scene entry has an explicit default applied
// when absent. That means an older reader tolerates a JSON document written
// by a newer version that adds fields to NodeDocument, and a document
// written before such a field existed loads fine into a newer reader. Adding
// a new NodeDocument field later only requires: (1) writing it out
// alongside displayName/x/y/etc. in the .cpp's node writer, and (2) reading
// it with a default in the node reader -- no schema versioning is required
// for an additive field.

#include "GraphDocument.h"
#include <string>
#include <string_view>

namespace guitardsp::graph {

// Serializes `document` to a JSON string. Control-thread only.
std::string graphDocumentToJson(const GraphDocument& document);

// Parses `json` into `outDocument`. Returns false and sets *error (if
// non-null) on malformed JSON or a structurally invalid document (e.g. a
// node missing its required "id"/"typeId"); never throws and never crashes
// on malformed input -- `outDocument` is left unmodified on failure.
// Control-thread only.
bool graphDocumentFromJson(std::string_view json, GraphDocument& outDocument, std::string* error = nullptr);

// Convenience file wrappers around the two functions above, mirroring
// NetlistLoader's loadFromFile split for netlists. Control-thread only.
bool saveGraphDocumentToFile(const GraphDocument& document, const std::string& path, std::string* error = nullptr);
bool loadGraphDocumentFromFile(const std::string& path, GraphDocument& outDocument, std::string* error = nullptr);

} // namespace guitardsp::graph
