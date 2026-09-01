#include "guitardsp/hq/AdditionalDeviceStages.h"
#include "guitardsp/hq/ComponentCatalog.h"

#include <cmath>
#include <iostream>

using namespace guitardsp;

namespace {
bool require(bool condition, const char* name) {
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    return condition;
}
}

int main() {
    bool ok = true;

    {
        auto cap = hq::component_presets::film22n();
        cap.capacitanceFarads = 47.0e-9f;
        cap.voltageRatingVolts = 250.0f;
        ok &= require(std::abs(cap.capacitanceFarads - 47.0e-9f) < 1.0e-12f,
                      "capacitor nominal value is editable");
        ok &= require(cap.voltageRatingVolts == 250.0f,
                      "capacitor voltage rating is independently editable");
    }

    {
        auto diode = hq::component_presets::oneN4148();
        const auto silicon = diode.toModel();
        diode.nominalForwardVoltage = 0.52f;
        diode.saturationCurrent *= 50.0f;
        const auto modified = diode.toModel();
        ok &= require(modified.current(0.55f) > silicon.current(0.55f),
                      "diode physical parameters alter nonlinear current");
    }

    {
        hq::PotentiometerSpec pot;
        pot.totalResistanceOhms = 100000.0f;
        pot.position = 0.5f;
        pot.taper = hq::PotTaper::linear;
        const float linear = pot.resistanceToWiperLow();
        pot.taper = hq::PotTaper::audio;
        const float audio = pot.resistanceToWiperLow();
        ok &= require(std::abs(linear - 50000.0f) < 1.0f,
                      "linear pot midpoint is half total resistance");
        ok &= require(audio < linear,
                      "audio taper differs from linear taper");
    }

    {
        hq::JFETCommonSourceStage stage;
        hq::JFETCommonSourceStage::Config config;
        config.device = hq::component_presets::j201();
        stage.prepare(48000.0, config);
        float peak = 0.0f;
        for (int i = 0; i < 2048; ++i) {
            const float x = 0.25f * std::sin(6.28318530718f * 220.0f * static_cast<float>(i) / 48000.0f);
            const float y = stage.process(x);
            ok &= require(std::isfinite(y), "JFET stage stays finite");
            peak = std::max(peak, std::abs(y));
            if (!ok) break;
        }
        ok &= require(peak > 1.0e-5f, "JFET common-source stage produces signal");
    }

    {
        hq::MOSFETCommonSourceStage stage;
        hq::MOSFETCommonSourceStage::Config config;
        config.device = hq::component_presets::bs170();
        stage.prepare(48000.0, config);
        float sum = 0.0f;
        for (int i = 0; i < 1024; ++i)
            sum += std::abs(stage.process(0.15f * std::sin(6.28318530718f * 330.0f * static_cast<float>(i) / 48000.0f)));
        ok &= require(std::isfinite(sum) && sum > 1.0e-5f,
                      "MOSFET common-source stage produces finite signal");
    }

    {
        hq::OpAmpStage stage;
        hq::OpAmpStage::Config config;
        config.device = hq::component_presets::jrc4558();
        config.closedLoopGain = 20.0f;
        stage.prepare(48000.0, config);
        float y = 0.0f;
        for (int i = 0; i < 256; ++i) y = stage.process(0.5f);
        ok &= require(y <= 3.1f && y >= -3.1f,
                      "op-amp stage respects output rail headroom");
    }

    {
        hq::OptocouplerLDR ldr;
        ldr.prepare(48000.0);
        const float dark = ldr.resistanceOhms();
        for (int i = 0; i < 4800; ++i) ldr.processLedDrive(1.0f);
        const float lit = ldr.resistanceOhms();
        ok &= require(lit < dark, "optocoupler LDR resistance falls under illumination");
    }

    return ok ? 0 : 1;
}
