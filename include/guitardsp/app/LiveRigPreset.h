#pragma once

#include "LiveRig.h"

#include <string>

namespace guitardsp::app {

// A saved preset: the full LiveRigSettings driving prepareLiveRig(), plus
// enough to restore any measured cabinet IR the rig was using.
//
// LiveRigSettings::cabinetImpulse/bassCabinetImpulse deliberately aren't part
// of what gets persisted here -- they're host-resampled PCM data tied to
// whatever device sample rate was active when they were computed (see
// LiveRig.h's doc comment on cabinetImpulse), not something to save
// byte-for-byte. Instead a preset remembers the *source* IR file path, the
// same identity GuitarDSPApp/Main.cpp's MeasuredImpulseState/
// loadImpulseResponse() already track, so the host can reload and re-resample
// it against whatever sample rate is active when the preset is restored.
// Empty path means "use the built-in reference IR", matching
// LiveRigSettings' own empty-vector fallback semantics.
struct LiveRigPreset {
    int version = 1;
    std::string name;
    LiveRigSettings settings;
    std::string guitarCabinetIrPath;
    std::string bassCabinetIrPath;
};

} // namespace guitardsp::app
