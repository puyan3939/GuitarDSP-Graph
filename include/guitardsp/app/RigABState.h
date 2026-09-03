#pragma once

#include "LiveRig.h"

namespace guitardsp::app {

// Two independently-held rig configurations ("A" and "B") for one-button
// comparison, plus which one is currently active. Deliberately just plain
// data: driving the *audio engine* to match the active slot still goes
// through RealtimeAudioEngine::rebuildRig() (a full topology rebuild +
// block-boundary hot swap via RealtimeGraphHost -- see its class doc
// comment), the same mechanism every other topology-affecting control
// (pedal/amp model, amp/cabinet enable, signal routing) already uses in
// GuitarDSPApp/Main.cpp's updateSettingsFromControls()/rebuildRig(). A and B
// can hold different pedal/amp models or routing, so there's no general way
// to avoid a full rebuild on toggle; RealtimeGraphHost's active/pending
// double buffer is already what keeps that rebuild real-time safe and low
// latency (the previous graph keeps running until the next block boundary,
// no audio-thread allocation or blocking -- see RealtimeGraphHost.h). This
// is a separate mechanism from the per-knob real-time path
// (RealtimeAudioEngine::setNodeParameter/setNodeTypeParameter, issue #47),
// which only ever touches parameters of the currently active, already-built
// graph.
struct RigABState {
    LiveRigSettings slotA;
    LiveRigSettings slotB;
    bool activeIsA = true;

    [[nodiscard]] LiveRigSettings& active() noexcept { return activeIsA ? slotA : slotB; }
    [[nodiscard]] const LiveRigSettings& active() const noexcept { return activeIsA ? slotA : slotB; }
    [[nodiscard]] LiveRigSettings& inactive() noexcept { return activeIsA ? slotB : slotA; }
    [[nodiscard]] const LiveRigSettings& inactive() const noexcept { return activeIsA ? slotB : slotA; }

    // Copies the currently active slot's settings into the other slot, e.g.
    // to seed B from A before the user starts tweaking it as an alternative.
    void copyActiveToInactive() { inactive() = active(); }

    void toggle() noexcept { activeIsA = !activeIsA; }
};

} // namespace guitardsp::app
