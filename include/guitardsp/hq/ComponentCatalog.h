#pragma once

#include "Components.h"
#include "PowerTubeModels.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace guitardsp::hq {

// ComponentCatalog separates three concepts that should not be conflated:
//  1) nominal electrical value (for example 22 nF or 100 kOhm),
//  2) physical rating/tolerance (for example 50 V, 1 %, 0.25 W), and
//  3) nonlinear model parameters used by the DSP solver.
//
// The factory presets below are engineering starting points for circuit modelling.
// They are intentionally editable and are not claims of exact manufacturer models.

enum class ComponentCategory : std::uint8_t {
    resistor,
    capacitor,
    inductor,
    potentiometer,
    diode,
    bjt,
    jfet,
    mosfet,
    opAmp,
    triode,
    powerTube,
    transformer,
    optocoupler
};

enum class CapacitorTechnology : std::uint8_t {
    generic,
    ceramic,
    film,
    electrolytic,
    tantalum
};

enum class DiodeTechnology : std::uint8_t {
    silicon,
    germanium,
    led
};

enum class TransistorPolarity : std::uint8_t {
    npn,
    pnp,
    nChannel,
    pChannel
};

enum class PotTaper : std::uint8_t {
    linear,
    audio,
    reverseAudio
};

struct ResistorSpec {
    float resistanceOhms = 1000.0f;
    float tolerancePercent = 5.0f;
    float powerRatingWatts = 0.25f;
    float temperatureCoefficientPpm = 100.0f;
    float excessNoiseFactor = 0.0f; // reserved for later noise modelling
};

struct CapacitorSpec {
    float capacitanceFarads = 22.0e-9f;
    float tolerancePercent = 10.0f;
    float voltageRatingVolts = 50.0f;
    float esrOhms = 0.05f;
    float leakageResistanceOhms = 1.0e9f;
    float dielectricAbsorption = 0.0f;
    CapacitorTechnology technology = CapacitorTechnology::film;
};

struct InductorSpec {
    float inductanceHenries = 1.0e-3f;
    float tolerancePercent = 10.0f;
    float currentRatingAmps = 0.10f;
    float seriesResistanceOhms = 1.0f;
    float parasiticCapacitanceFarads = 10.0e-12f;
    float saturationCurrentAmps = 0.20f;
};

struct PotentiometerSpec {
    float totalResistanceOhms = 100000.0f;
    float tolerancePercent = 20.0f;
    float powerRatingWatts = 0.25f;
    PotTaper taper = PotTaper::audio;
    float position = 0.5f; // normalized mechanical position, 0..1

    float normalizedElectricalPosition() const noexcept {
        const float p = std::clamp(position, 0.0f, 1.0f);
        switch (taper) {
            case PotTaper::audio:
                // Simple 10 % audio-taper approximation. A later part-specific
                // curve can replace this without changing the component contract.
                return (std::pow(10.0f, p) - 1.0f) / 9.0f;
            case PotTaper::reverseAudio:
                return 1.0f - (std::pow(10.0f, 1.0f - p) - 1.0f) / 9.0f;
            case PotTaper::linear:
            default:
                return p;
        }
    }

    float resistanceToWiperLow() const noexcept {
        return totalResistanceOhms * normalizedElectricalPosition();
    }
    float resistanceToWiperHigh() const noexcept {
        return totalResistanceOhms - resistanceToWiperLow();
    }
};

struct DiodeSpec {
    std::string_view name = "Generic Silicon";
    DiodeTechnology technology = DiodeTechnology::silicon;
    float nominalForwardVoltage = 0.65f; // metadata/UI reference, not a hard clamp
    float saturationCurrent = 2.0e-9f;
    float emissionCoefficient = 1.9f;
    float thermalVoltage = 0.02585f;
    float seriesResistanceOhms = 2.0f;
    float junctionCapacitanceFarads = 2.0e-12f;
    float reverseVoltageRating = 75.0f;
    float currentRatingAmps = 0.15f;

    DiodeModel toModel() const noexcept {
        return {saturationCurrent, emissionCoefficient, thermalVoltage};
    }
};

struct BJTSpec {
    std::string_view name = "Generic NPN";
    TransistorPolarity polarity = TransistorPolarity::npn;
    float beta = 180.0f;
    float nominalVbe = 0.62f;
    float saturationVoltage = 0.18f;
    float thermalVoltage = 0.02585f;
    float maxCollectorVoltage = 40.0f;
    float maxCollectorCurrentAmps = 0.20f;
    float inputCapacitanceFarads = 10.0e-12f;

    BJTModel toModel() const noexcept {
        return {beta, nominalVbe, thermalVoltage, saturationVoltage};
    }
};

struct JFETSpec {
    std::string_view name = "Generic N-JFET";
    TransistorPolarity polarity = TransistorPolarity::nChannel;
    float idssAmps = 2.0e-3f;
    float pinchOffVoltage = -1.5f;
    float lambda = 0.01f;
    float gateSourceCapacitanceFarads = 5.0e-12f;
    float maxDrainSourceVoltage = 25.0f;
};

struct MOSFETSpec {
    std::string_view name = "Generic N-MOSFET";
    TransistorPolarity polarity = TransistorPolarity::nChannel;
    float thresholdVoltage = 2.0f;
    float transconductance = 0.10f;
    float lambda = 0.02f;
    float bodyDiodeForwardVoltage = 0.75f;
    float gateCapacitanceFarads = 30.0e-12f;
    float maxDrainSourceVoltage = 60.0f;
};

struct OpAmpSpec {
    std::string_view name = "Generic Audio Op-Amp";
    float openLoopGainDb = 100.0f;
    float gainBandwidthHz = 3.0e6f;
    float slewRateVoltsPerSecond = 5.0e6f;
    float inputBiasCurrentAmps = 50.0e-9f;
    float inputOffsetVoltage = 2.0e-3f;
    float inputNoiseVoltsPerRootHz = 18.0e-9f;
    float outputCurrentLimitAmps = 0.025f;
    float positiveRailHeadroomVolts = 1.5f;
    float negativeRailHeadroomVolts = 1.5f;
};

struct TriodeSpec {
    std::string_view name = "12AX7";
    TriodeModel model = TriodeModel::twelveAX7();
    float heaterVoltage = 6.3f;
    float nominalPlateVoltage = 250.0f;
    float maxPlateVoltage = 300.0f;
    float maxPlateDissipationWatts = 1.0f;
};

struct PowerTubeSpec {
    std::string_view name = "EL34";
    PowerTubeType type = PowerTubeType::el34;
    float heaterVoltage = 6.3f;
    float nominalPlateVoltage = 425.0f;
    float maxPlateVoltage = 800.0f;
    float maxPlateDissipationWatts = 25.0f;

    PowerTubeModel toModel() const noexcept { return PowerTubeModel::forType(type); }
};

struct TransformerSpec {
    std::string_view name = "Generic Output Transformer";
    float primaryInductanceHenries = 20.0f;
    float leakageInductanceHenries = 25.0e-3f;
    float primaryResistanceOhms = 120.0f;
    float secondaryResistanceOhms = 0.5f;
    float turnsRatio = 25.0f;
    float interwindingCapacitanceFarads = 1.0e-9f;
    float saturationFluxNormalized = 1.0f;
};

struct OptocouplerSpec {
    std::string_view name = "Generic LED/LDR";
    float ledForwardVoltage = 1.7f;
    float darkResistanceOhms = 5.0e6f;
    float lightResistanceOhms = 500.0f;
    float attackMs = 8.0f;
    float releaseMs = 80.0f;
    float gamma = 0.7f;
};

namespace component_presets {

inline constexpr ResistorSpec carbonFilm100k() noexcept {
    return {100000.0f, 5.0f, 0.25f, 200.0f, 0.2f};
}
inline constexpr ResistorSpec metalFilm100k() noexcept {
    return {100000.0f, 1.0f, 0.25f, 50.0f, 0.02f};
}
inline constexpr CapacitorSpec film22n() noexcept {
    return {22.0e-9f, 5.0f, 100.0f, 0.03f, 2.0e9f, 0.0005f, CapacitorTechnology::film};
}
inline constexpr CapacitorSpec electrolytic10u() noexcept {
    return {10.0e-6f, 20.0f, 25.0f, 0.8f, 5.0e6f, 0.02f, CapacitorTechnology::electrolytic};
}
inline constexpr InductorSpec inductor1mH() noexcept {
    return {1.0e-3f, 10.0f, 0.15f, 2.0f, 15.0e-12f, 0.25f};
}

inline constexpr DiodeSpec oneN4148() noexcept {
    return {"1N4148-style", DiodeTechnology::silicon, 0.65f, 2.0e-9f, 1.9f, 0.02585f,
            1.0f, 2.0e-12f, 100.0f, 0.20f};
}
inline constexpr DiodeSpec oneN34A() noexcept {
    return {"1N34A-style", DiodeTechnology::germanium, 0.28f, 2.0e-6f, 1.1f, 0.02585f,
            3.0f, 2.5e-12f, 60.0f, 0.05f};
}
inline constexpr DiodeSpec redLed() noexcept {
    return {"Red LED", DiodeTechnology::led, 1.75f, 1.0e-12f, 2.2f, 0.02585f,
            8.0f, 25.0e-12f, 5.0f, 0.02f};
}

inline constexpr BJTSpec twoN3904() noexcept {
    return {"2N3904-style", TransistorPolarity::npn, 180.0f, 0.64f, 0.18f, 0.02585f,
            40.0f, 0.20f, 8.0e-12f};
}
inline constexpr BJTSpec twoN5088() noexcept {
    return {"2N5088-style", TransistorPolarity::npn, 450.0f, 0.62f, 0.16f, 0.02585f,
            30.0f, 0.10f, 12.0e-12f};
}

inline constexpr JFETSpec j201() noexcept {
    return {"J201-style", TransistorPolarity::nChannel, 0.8e-3f, -0.8f, 0.015f, 5.0e-12f, 40.0f};
}
inline constexpr JFETSpec twoN5457() noexcept {
    return {"2N5457-style", TransistorPolarity::nChannel, 3.0e-3f, -2.5f, 0.012f, 4.0e-12f, 25.0f};
}
inline constexpr MOSFETSpec bs170() noexcept {
    return {"BS170-style", TransistorPolarity::nChannel, 2.1f, 0.12f, 0.02f, 0.75f, 35.0e-12f, 60.0f};
}

inline constexpr OpAmpSpec jrc4558() noexcept {
    return {"JRC4558-style", 100.0f, 3.0e6f, 1.0e6f, 100.0e-9f, 2.0e-3f, 18.0e-9f, 0.025f, 1.5f, 1.5f};
}
inline constexpr OpAmpSpec tl072() noexcept {
    return {"TL072-style", 106.0f, 3.0e6f, 13.0e6f, 65.0e-12f, 3.0e-3f, 18.0e-9f, 0.010f, 1.5f, 1.5f};
}

inline TriodeSpec twelveAX7() noexcept {
    return {"12AX7", TriodeModel::twelveAX7(), 6.3f, 250.0f, 300.0f, 1.0f};
}
inline TriodeSpec twelveAT7() noexcept {
    return {"12AT7", TriodeModel::twelveAT7(), 6.3f, 250.0f, 300.0f, 2.5f};
}

inline constexpr PowerTubeSpec el34() noexcept {
    return {"EL34", PowerTubeType::el34, 6.3f, 425.0f, 800.0f, 25.0f};
}
inline constexpr PowerTubeSpec sixL6GC() noexcept {
    return {"6L6GC", PowerTubeType::sixL6GC, 6.3f, 450.0f, 500.0f, 30.0f};
}
inline constexpr PowerTubeSpec kt88() noexcept {
    return {"KT88", PowerTubeType::kt88, 6.3f, 500.0f, 800.0f, 42.0f};
}

} // namespace component_presets

} // namespace guitardsp::hq
