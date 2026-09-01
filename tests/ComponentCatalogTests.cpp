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

    {
        const auto el34 = hq::PentodeModel::el34();
        ok &= require(el34.plateCurrent(-4.0f, 0.0f, 250.0f) == 0.0f,
                      "pentode plate current is zero at Vpk == 0 (plate-voltage knee floor)");
        const float lowPlate = el34.plateCurrent(-4.0f, 20.0f, 250.0f);
        const float highPlate = el34.plateCurrent(-4.0f, 300.0f, 250.0f);
        const float veryHighPlate = el34.plateCurrent(-4.0f, 3000.0f, 250.0f);
        ok &= require(highPlate > lowPlate,
                      "pentode plate current rises with plate voltage through the knee region");
        ok &= require(std::abs(veryHighPlate - highPlate) < 0.10f * highPlate,
                      "pentode plate current saturates and grows only weakly with plate voltage "
                      "far beyond the knee (defining pentode trait: a 10x plate-voltage increase "
                      "changes current by well under 10%)");

        const float screenCurrent = el34.screenCurrent(-4.0f, 250.0f);
        ok &= require(screenCurrent > 0.0f && screenCurrent < highPlate,
                      "pentode screen current is positive and a minority fraction of plate current "
                      "at a typical bias point");
        const float strongerDriveScreen = el34.screenCurrent(-2.0f, 250.0f);
        ok &= require(strongerDriveScreen > screenCurrent,
                      "pentode screen current rises with stronger grid drive, same as plate current");

        const float cutoffCurrent = el34.plateCurrent(-150.0f, 300.0f, 250.0f);
        ok &= require(cutoffCurrent >= 0.0f && cutoffCurrent < 1.0e-6f,
                      "deeply negative grid drive cuts off pentode plate current");

        const float lowScreenDrive = el34.plateCurrent(-4.0f, 300.0f, 0.0f);
        ok &= require(std::isfinite(lowScreenDrive) && lowScreenDrive >= 0.0f,
                      "pentode drive term stays finite as screen voltage collapses toward zero");
    }

    return ok ? 0 : 1;
}
