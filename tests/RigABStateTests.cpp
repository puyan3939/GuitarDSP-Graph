// Regression tests for issue #79: RigABState's slot bookkeeping for the A/B
// rig comparison toggle. Pure data-structure logic (no audio engine
// involved) -- the actual low-latency swap is RealtimeAudioEngine::
// rebuildRig(), already covered by RealtimeAudioEngineTests.cpp.
#include "guitardsp/app/RigABState.h"

#include <cmath>
#include <iostream>

using namespace guitardsp::app;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}
} // namespace

int main() {
    bool ok = true;

    {
        RigABState state;
        state.slotA.pedalDrive = 0.3f;
        state.slotB.pedalDrive = 0.9f;

        ok &= require(state.activeIsA, "RigABState defaults to slot A active");
        ok &= require(std::abs(state.active().pedalDrive - 0.3f) < 1.0e-6f,
                      "active() returns slot A while activeIsA is true");
        ok &= require(std::abs(state.inactive().pedalDrive - 0.9f) < 1.0e-6f,
                      "inactive() returns slot B while activeIsA is true");

        state.toggle();
        ok &= require(!state.activeIsA, "toggle() flips activeIsA");
        ok &= require(std::abs(state.active().pedalDrive - 0.9f) < 1.0e-6f,
                      "active() returns slot B after toggling");
        ok &= require(std::abs(state.inactive().pedalDrive - 0.3f) < 1.0e-6f,
                      "inactive() returns slot A after toggling");

        state.toggle();
        ok &= require(state.activeIsA, "toggling twice returns to slot A active");
    }

    {
        RigABState state;
        state.slotA.pedalDrive = 0.55f;
        state.slotA.ampGain = 0.44f;
        state.slotB.pedalDrive = 0.11f;
        state.slotB.ampGain = 0.22f;

        state.copyActiveToInactive();
        ok &= require(std::abs(state.slotB.pedalDrive - 0.55f) < 1.0e-6f
                          && std::abs(state.slotB.ampGain - 0.44f) < 1.0e-6f,
                      "copyActiveToInactive() overwrites the inactive slot with the active one");
        ok &= require(std::abs(state.slotA.pedalDrive - 0.55f) < 1.0e-6f,
                      "copyActiveToInactive() leaves the active slot untouched");
    }

    {
        const RigABState state;
        ok &= require(&state.active() == &state.slotA,
                      "const active() aliases slotA when activeIsA is true");
        ok &= require(&state.inactive() == &state.slotB,
                      "const inactive() aliases slotB when activeIsA is true");
    }

    return ok ? 0 : 1;
}
