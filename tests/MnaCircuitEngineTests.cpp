#include "guitardsp/circuit/MnaCircuitEngine.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}

circuit::MnaCircuitEngine makeRc(float capacitance) {
    circuit::MnaCircuitEngine c;
    const auto in = c.addNode();
    const auto out = c.addNode();
    hq::ResistorSpec r{};
    r.resistanceOhms = 1000.0f;
    hq::CapacitorSpec cap{};
    cap.capacitanceFarads = capacitance;
    cap.leakageResistanceOhms = 1.0e12f;
    cap.esrOhms = 0.0f;
    c.addVoltageSource(in, circuit::ground, 1.0f);
    c.addResistor(in, out, r);
    c.addCapacitor(out, circuit::ground, cap);
    c.prepare(48000.0);
    return c;
}
}

int main() {
    bool ok = true;

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 1000.0f;
        const auto source = c.addVoltageSource(in, circuit::ground, 1.0f);
        c.addResistor(in, out, r);
        const auto load = c.addResistor(out, circuit::ground, r);
        ok &= require(c.prepare(48000.0), "MNA prepares resistor divider");
        const auto stats = c.processSample();
        ok &= require(!stats.singular && std::abs(c.voltage(out) - 0.5f) < 1.0e-5f,
                      "MNA resistor divider solves 0.5 V");
        c.setVoltageSource(source, -0.8f);
        c.processSample();
        ok &= require(std::abs(c.voltage(out) + 0.4f) < 1.0e-5f,
                      "MNA realtime voltage source update works");
        c.setVoltageSource(source, 1.0f);
        ok &= require(c.setResistance(load, 3000.0f),
                      "resistor handle accepts value edit without topology rebuild");
        c.processSample();
        ok &= require(std::abs(c.voltage(out) - 0.75f) < 1.0e-5f,
                      "resistor handle edit changes prepared circuit response");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto high = c.addNode();
        const auto wiper = c.addNode();
        hq::PotentiometerSpec pot{};
        pot.totalResistanceOhms = 100000.0f;
        pot.taper = hq::PotTaper::linear;
        pot.position = 0.25f;
        c.addVoltageSource(high, circuit::ground, 1.0f);
        const auto handle = c.addPotentiometer(high, wiper, circuit::ground, pot);
        ok &= require(c.prepare(48000.0), "MNA prepares three-terminal potentiometer");
        c.processSample();
        ok &= require(std::abs(c.voltage(wiper) - 0.25f) < 1.0e-4f,
                      "linear potentiometer maps mechanical position to divider voltage");
        c.setPotentiometerPosition(handle, 0.75f);
        c.processSample();
        ok &= require(std::abs(c.voltage(wiper) - 0.75f) < 1.0e-4f,
                      "potentiometer position updates without topology rebuild");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        hq::ResistorSpec load{};
        load.resistanceOhms = 1000.0f;
        c.addVoltageSource(control, circuit::ground, 1.0f);
        c.addResistor(output, circuit::ground, load);
        c.addVccs(circuit::ground, output, control, circuit::ground, 1.0e-3f);
        ok &= require(c.prepare(48000.0), "MNA prepares VCCS");
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 1.0f) < 1.0e-4f,
                      "VCCS converts control voltage into output current");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto control = c.addNode();
        const auto output = c.addNode();
        c.addVoltageSource(control, circuit::ground, 0.2f);
        c.addVcvs(output, circuit::ground, control, circuit::ground, 5.0f);
        ok &= require(c.prepare(48000.0), "MNA prepares VCVS branch unknown");
        c.processSample();
        ok &= require(std::abs(c.voltage(output) - 1.0f) < 1.0e-4f,
                      "VCVS enforces controlled output voltage");
    }

    {
        auto fast = makeRc(1.0e-6f);
        auto slow = makeRc(10.0e-6f);
        fast.processSample();
        slow.processSample();
        ok &= require(fast.voltage(2) > slow.voltage(2),
                      "capacitor value changes transient response");
        for (int i = 0; i < 2400; ++i) {
            fast.processSample();
            slow.processSample();
        }
        ok &= require(fast.voltage(2) > 0.99f && slow.voltage(2) > 0.98f,
                      "trapezoidal capacitors settle to DC");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 1000.0f;
        hq::CapacitorSpec cap{};
        cap.capacitanceFarads = 1.0e-6f;
        cap.leakageResistanceOhms = 1.0e12f;
        cap.esrOhms = 0.0f;
        c.addVoltageSource(in, circuit::ground, 1.0f);
        c.addResistor(in, out, r);
        const auto capHandle = c.addCapacitor(out, circuit::ground, cap);
        c.prepare(48000.0);
        c.processSample();
        const float fastFirst = c.voltage(out);
        c.reset();
        ok &= require(c.setCapacitance(capHandle, 10.0e-6f),
                      "capacitor handle accepts capacitance edit");
        c.processSample();
        ok &= require(c.voltage(out) < fastFirst,
                      "capacitance handle edit changes transient without topology rebuild");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::ResistorSpec r{};
        r.resistanceOhms = 2200.0f;
        c.addVoltageSource(in, circuit::ground, 0.8f);
        c.addResistor(in, out, r);
        const auto diode = c.addDiode(out, circuit::ground, hq::component_presets::oneN4148());
        c.prepare(48000.0);
        const auto stats = c.processSample(24, 1.0e-7f);
        ok &= require(!stats.singular && stats.converged, "nonlinear diode circuit converges");
        const float siliconVoltage = c.voltage(out);
        ok &= require(siliconVoltage > 0.45f && siliconVoltage < 0.65f,
                      "1N4148-style diode bends transfer through series resistor");
        c.reset();
        ok &= require(c.setDiodeSpec(diode, hq::component_presets::oneN34A()),
                      "diode handle accepts device-family replacement");
        c.processSample(32, 1.0e-7f);
        ok &= require(c.voltage(out) < siliconVoltage,
                      "germanium-style diode replacement lowers clamp voltage");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vcc = c.addNode();
        const auto base = c.addNode();
        const auto collector = c.addNode();
        const auto emitter = c.addNode();
        hq::ResistorSpec rc{};
        rc.resistanceOhms = 4700.0f;
        hq::ResistorSpec re{};
        re.resistanceOhms = 680.0f;
        c.addVoltageSource(vcc, circuit::ground, 9.0f);
        c.addVoltageSource(base, circuit::ground, 0.72f);
        c.addResistor(vcc, collector, rc);
        c.addResistor(emitter, circuit::ground, re);
        const auto transistor = c.addBjt(collector, base, emitter, hq::component_presets::twoN3904());
        ok &= require(c.prepare(48000.0), "MNA prepares BJT three-terminal stamp");
        const auto stats = c.processSample(32, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "BJT nonlinear stamp converges");
        ok &= require(c.voltage(emitter) > 0.02f && c.voltage(collector) < 8.95f,
                      "BJT bias produces emitter current and collector drop");
        ok &= require(c.setBjtSpec(transistor, hq::component_presets::twoN5088()),
                      "BJT handle accepts transistor replacement");
        c.processSample(32, 1.0e-6f);
        ok &= require(std::isfinite(c.voltage(collector)),
                      "BJT replacement remains numerically finite");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vdd = c.addNode();
        const auto drain = c.addNode();
        const auto source = c.addNode();
        hq::ResistorSpec rd{};
        rd.resistanceOhms = 10000.0f;
        hq::ResistorSpec rs{};
        rs.resistanceOhms = 1500.0f;
        c.addVoltageSource(vdd, circuit::ground, 9.0f);
        c.addResistor(vdd, drain, rd);
        c.addResistor(source, circuit::ground, rs);
        c.addJfet(drain, circuit::ground, source, hq::component_presets::j201());
        ok &= require(c.prepare(48000.0), "MNA prepares JFET common-source stamp");
        const auto stats = c.processSample(32, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "JFET nonlinear stamp converges");
        ok &= require(c.voltage(source) > 0.05f && c.voltage(drain) > 1.0f && c.voltage(drain) < 8.8f,
                      "J201-style self bias changes drain and source voltages");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto vdd = c.addNode();
        const auto gate = c.addNode();
        const auto drain = c.addNode();
        const auto source = c.addNode();
        hq::ResistorSpec rd{};
        rd.resistanceOhms = 4700.0f;
        hq::ResistorSpec rs{};
        rs.resistanceOhms = 470.0f;
        c.addVoltageSource(vdd, circuit::ground, 9.0f);
        c.addVoltageSource(gate, circuit::ground, 2.8f);
        c.addResistor(vdd, drain, rd);
        c.addResistor(source, circuit::ground, rs);
        c.addMosfet(drain, gate, source, hq::component_presets::bs170());
        ok &= require(c.prepare(48000.0), "MNA prepares MOSFET three-terminal stamp");
        const auto stats = c.processSample(36, 1.0e-6f);
        ok &= require(!stats.singular && stats.converged, "MOSFET nonlinear stamp converges");
        ok &= require(c.voltage(source) > 0.10f && c.voltage(drain) > 0.5f && c.voltage(drain) < 8.8f,
                      "BS170-style bias produces nonlinear drain current");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto input = c.addNode();
        const auto output = c.addNode();
        auto opAmp = hq::component_presets::jrc4558();
        opAmp.inputOffsetVoltage = 0.0f;
        c.addVoltageSource(input, circuit::ground, 0.25f);
        const auto handle = c.addOpAmp(output, input, output, circuit::ground, opAmp);
        ok &= require(c.prepare(48000.0), "MNA prepares finite-gain op-amp macro stamp");
        const auto stats = c.processSample();
        ok &= require(!stats.singular && std::abs(c.voltage(output) - 0.25f) < 1.0e-4f,
                      "high-open-loop-gain op-amp closes as unity follower");
        opAmp.openLoopGainDb = 20.0f;
        ok &= require(c.setOpAmpSpec(handle, opAmp),
                      "op-amp handle accepts macro-model edit");
        c.processSample();
        const float expected = 0.25f * 10.0f / 11.0f;
        ok &= require(std::abs(c.voltage(output) - expected) < 1.0e-4f,
                      "finite op-amp gain changes closed-loop accuracy");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto supply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 100000.0f;
        c.addVoltageSource(supply, circuit::ground, 250.0f);
        c.addVoltageSource(grid, circuit::ground, -1.2f);
        c.addResistor(supply, plate, plateLoad);
        const auto tube = c.addTriode(plate, grid, circuit::ground,
                                     hq::component_presets::twelveAX7());
        ok &= require(c.prepare(48000.0), "MNA prepares triode plate-grid-cathode stamp");
        const auto stats = c.processSample(40, 1.0e-5f);
        ok &= require(!stats.singular && stats.converged, "12AX7 nonlinear MNA stamp converges");
        const float ax7Plate = c.voltage(plate);
        ok &= require(ax7Plate > 40.0f && ax7Plate < 220.0f,
                      "12AX7 plate current establishes plausible loaded operating point");
        ok &= require(c.setTriodeSpec(tube, hq::component_presets::twelveAT7()),
                      "triode handle accepts tube-family replacement");
        c.processSample(40, 1.0e-5f);
        ok &= require(std::abs(c.voltage(plate) - ax7Plate) > 1.0f,
                      "12AT7 replacement changes the same circuit operating point");
    }

    // A pentode power-tube stage swings its plate/screen nodes from a cold 0 V start
    // to several hundred volts, well beyond the per-iteration trust-region step the
    // Newton solver uses to protect low-voltage semiconductor junctions elsewhere.
    // Reaching the operating point is expected to take a few processSample() calls
    // (much like the charge-up transient in a real amplifier's B+ supply) rather than
    // fully settling within a single 40-iteration Newton solve from a cold state; this
    // helper runs that short settling window and returns the final sample's stats.
    const auto settlePentode = [](circuit::MnaCircuitEngine& engine, int warmupSamples) {
        circuit::MnaCircuitEngine::SolveStats stats{};
        for (int i = 0; i < warmupSamples; ++i) stats = engine.processSample(40, 1.0e-5f);
        return stats;
    };

    {
        circuit::MnaCircuitEngine c;
        const auto plateSupply = c.addNode();
        const auto screenSupply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        const auto screen = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 4000.0f;
        hq::ResistorSpec screenLoad{};
        screenLoad.resistanceOhms = 470.0f;
        c.addVoltageSource(plateSupply, circuit::ground, 420.0f);
        c.addVoltageSource(screenSupply, circuit::ground, 420.0f);
        c.addVoltageSource(grid, circuit::ground, -20.0f);
        c.addResistor(plateSupply, plate, plateLoad);
        c.addResistor(screenSupply, screen, screenLoad);
        const auto tube = c.addPentode(plate, grid, screen, circuit::ground,
                                       hq::component_presets::pentodeEl34());
        ok &= require(c.prepare(48000.0), "MNA prepares pentode plate-grid-screen-cathode stamp");
        const auto stats = settlePentode(c, 5);
        ok &= require(!stats.singular && stats.converged, "EL34 pentode nonlinear MNA stamp converges");
        const float plateVoltage = c.voltage(plate);
        const float screenVoltage = c.voltage(screen);
        ok &= require(plateVoltage > 0.0f && plateVoltage < 420.0f,
                      "pentode plate settles to a loaded operating point below the supply rail");
        ok &= require(screenVoltage > 0.0f && screenVoltage < 420.0f,
                      "pentode screen settles to a loaded operating point below the supply rail");
        ok &= require(c.setPentodeSpec(tube, hq::component_presets::pentodeKt88()),
                      "pentode handle accepts tube-family replacement");
        settlePentode(c, 5);
        ok &= require(std::abs(c.voltage(plate) - plateVoltage) > 0.5f,
                      "KT88 replacement changes the same circuit operating point");
    }

    {
        // High-voltage power-stage convergence check (issue #27): pentode/beam-tetrode
        // power tubes run their plate/screen rails at 400-800 V, well above the small-
        // signal triode operating range. Newton's trust region must still land here
        // (within the short cold-start settling window above).
        circuit::MnaCircuitEngine c;
        const auto plateSupply = c.addNode();
        const auto screenSupply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        const auto screen = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 6000.0f;
        hq::ResistorSpec screenLoad{};
        screenLoad.resistanceOhms = 1000.0f;
        c.addVoltageSource(plateSupply, circuit::ground, 800.0f);
        c.addVoltageSource(screenSupply, circuit::ground, 800.0f);
        c.addVoltageSource(grid, circuit::ground, -35.0f);
        c.addResistor(plateSupply, plate, plateLoad);
        c.addResistor(screenSupply, screen, screenLoad);
        c.addPentode(plate, grid, screen, circuit::ground, hq::component_presets::pentodeKt88());
        ok &= require(c.prepare(48000.0), "MNA prepares an 800 V pentode power stage");
        const auto stats = settlePentode(c, 8);
        ok &= require(!stats.singular && stats.converged,
                      "pentode Newton solve converges at an 800 V plate/screen rail");
        ok &= require(std::isfinite(c.voltage(plate)) && std::isfinite(c.voltage(screen)),
                      "pentode high-voltage operating point stays finite");
    }

    {
        // Boundary test: grid driven hard negative should cut the pentode off, leaving
        // plate and screen close to their unloaded supply rails.
        circuit::MnaCircuitEngine c;
        const auto plateSupply = c.addNode();
        const auto screenSupply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        const auto screen = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 4000.0f;
        hq::ResistorSpec screenLoad{};
        screenLoad.resistanceOhms = 470.0f;
        c.addVoltageSource(plateSupply, circuit::ground, 420.0f);
        c.addVoltageSource(screenSupply, circuit::ground, 420.0f);
        c.addVoltageSource(grid, circuit::ground, -150.0f);
        c.addResistor(plateSupply, plate, plateLoad);
        c.addResistor(screenSupply, screen, screenLoad);
        c.addPentode(plate, grid, screen, circuit::ground, hq::component_presets::pentodeEl34());
        ok &= require(c.prepare(48000.0), "MNA prepares a cutoff-biased pentode stage");
        const auto stats = settlePentode(c, 5);
        ok &= require(!stats.singular && stats.converged, "cutoff-biased pentode still converges");
        ok &= require(c.voltage(plate) > 415.0f,
                      "deep grid cutoff leaves the pentode plate near the unloaded supply rail");
    }

    {
        // Newton solver robustness sweep (issue #16): rather than checking a single
        // hand-picked bias point, this cold-starts a fresh pentode stage at every
        // grid bias from deep cutoff to near-zero and requires every one of them to
        // converge. Pentode plate current is a stronger, more kinked nonlinearity
        // than the triode/BJT/diode stages exercised elsewhere, so this is the
        // most direct test of the claim that a physically valid pentode circuit
        // converges numerically regardless of where its operating point lands.
        bool allConverged = true;
        for (int step = 0; step <= 8; ++step) {
            const float gridBias = -150.0f + step * (145.0f / 8.0f);
            circuit::MnaCircuitEngine c;
            const auto plateSupply = c.addNode();
            const auto screenSupply = c.addNode();
            const auto grid = c.addNode();
            const auto plate = c.addNode();
            const auto screen = c.addNode();
            hq::ResistorSpec plateLoad{};
            plateLoad.resistanceOhms = 4000.0f;
            hq::ResistorSpec screenLoad{};
            screenLoad.resistanceOhms = 470.0f;
            c.addVoltageSource(plateSupply, circuit::ground, 420.0f);
            c.addVoltageSource(screenSupply, circuit::ground, 420.0f);
            c.addVoltageSource(grid, circuit::ground, gridBias);
            c.addResistor(plateSupply, plate, plateLoad);
            c.addResistor(screenSupply, screen, screenLoad);
            c.addPentode(plate, grid, screen, circuit::ground, hq::component_presets::pentodeEl34());
            const bool prepared = c.prepare(48000.0);
            const auto stats = settlePentode(c, 8);
            const bool healthy = prepared && !stats.singular && stats.converged
                && std::isfinite(c.voltage(plate)) && std::isfinite(c.voltage(screen));
            std::cout << "DIAG pentode-sweep gridBias=" << gridBias
                      << " plate=" << c.voltage(plate) << " converged=" << stats.converged
                      << " singular=" << stats.singular << '\n';
            allConverged &= healthy;
        }
        ok &= require(allConverged,
                      "pentode Newton solve converges across the full cutoff-to-near-zero grid bias sweep");
    }

    {
        // Audio-rate companion to the DC sweep above: a pentode gain stage driven
        // hard and then cut to silence, mirroring the exact repro shape that
        // caught DS-1's original Newton-limit-cycle hiss (issue #14/#16). Pentode
        // plate current is more sharply nonlinear than DS-1's op-amp stage, so a
        // clean, low-jitter settle here is the strongest available check that the
        // engine-level line search generalizes beyond the circuit it was
        // diagnosed on.
        circuit::MnaCircuitEngine c;
        const auto plateSupply = c.addNode();
        const auto screenSupply = c.addNode();
        const auto grid = c.addNode();
        const auto plate = c.addNode();
        const auto screen = c.addNode();
        hq::ResistorSpec plateLoad{};
        plateLoad.resistanceOhms = 4000.0f;
        hq::ResistorSpec screenLoad{};
        screenLoad.resistanceOhms = 470.0f;
        c.addVoltageSource(plateSupply, circuit::ground, 420.0f);
        c.addVoltageSource(screenSupply, circuit::ground, 420.0f);
        const auto gridSource = c.addVoltageSource(grid, circuit::ground, -20.0f);
        c.addResistor(plateSupply, plate, plateLoad);
        c.addResistor(screenSupply, screen, screenLoad);
        c.addPentode(plate, grid, screen, circuit::ground, hq::component_presets::pentodeEl34());
        ok &= require(c.prepare(48000.0), "MNA prepares a driven pentode gain stage");

        constexpr int driveSamples = 2048;
        for (int i = 0; i < driveSamples; ++i) {
            const double phase = 2.0 * 3.14159265358979323846 * 120.0
                * static_cast<double>(i) / 48000.0;
            c.setVoltageSource(gridSource, -20.0f + 15.0f * static_cast<float>(std::sin(phase)));
            c.processSample(40, 1.0e-5f);
        }

        c.setVoltageSource(gridSource, -20.0f);
        constexpr int silentSamples = 2048;
        double sumSquaredJitter = 0.0;
        bool healthy = true;
        int unconverged = 0;
        c.processSample(40, 1.0e-5f); // let the step from oscillating drive to fixed bias settle first
        float previous = c.voltage(plate);
        for (int i = 1; i < silentSamples; ++i) {
            c.processSample(40, 1.0e-5f);
            const auto stats = c.lastStats();
            const float y = c.voltage(plate);
            healthy &= !stats.singular && std::isfinite(y);
            unconverged += stats.converged ? 0 : 1;
            const double delta = static_cast<double>(y) - static_cast<double>(previous);
            sumSquaredJitter += delta * delta;
            previous = y;
        }
        const double jitterRms = std::sqrt(sumSquaredJitter / static_cast<double>(silentSamples - 1));
        std::cout << "DIAG pentode-silence jitter_rms=" << jitterRms
                  << " unconverged=" << unconverged << '\n';
        ok &= require(healthy, "driven pentode stage stays finite and nonsingular decaying into silence");
        ok &= require(unconverged < silentSamples / 50,
                      "driven pentode stage converges on nearly every silent sample");
        ok &= require(jitterRms < 0.1,
                      "driven pentode stage settles into silence without a Newton limit cycle");
    }

    {
        circuit::MnaCircuitEngine c;
        const auto in = c.addNode();
        const auto out = c.addNode();
        hq::InductorSpec l{};
        l.inductanceHenries = 10.0e-3f;
        l.seriesResistanceOhms = 2.0f;
        hq::ResistorSpec load{};
        load.resistanceOhms = 1000.0f;
        c.addVoltageSource(in, circuit::ground, 1.0f);
        const auto inductor = c.addInductor(in, out, l);
        c.addResistor(out, circuit::ground, load);
        c.prepare(48000.0);
        for (int i = 0; i < 2000; ++i) c.processSample();
        ok &= require(c.inductorCurrent(0) > 0.0009f && c.inductorCurrent(0) < 0.0011f,
                      "trapezoidal inductor reaches expected DC current");
        ok &= require(c.setInductance(inductor, 20.0e-3f),
                      "inductor handle accepts inductance edit");
    }

    return ok ? 0 : 1;
}
