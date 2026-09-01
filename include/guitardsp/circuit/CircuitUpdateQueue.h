#pragma once

#include "MnaCircuitEngine.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <variant>

namespace guitardsp::circuit {

enum class CircuitUpdateKind : std::uint8_t {
    voltageSource,
    resistance,
    resistorSpec,
    capacitance,
    capacitorSpec,
    inductance,
    inductorSpec,
    potentiometerPosition,
    potentiometerSpec,
    vccsTransconductance,
    vcvsGain,
    cccsGain,
    ccvsTransresistance,
    diodeSpec,
    bjtSpec,
    jfetSpec,
    mosfetSpec,
    opAmpSpec,
    triodeSpec
};

using CircuitUpdatePayload = std::variant<
    float,
    hq::ResistorSpec,
    hq::CapacitorSpec,
    hq::InductorSpec,
    hq::PotentiometerSpec,
    hq::DiodeSpec,
    hq::BJTSpec,
    hq::JFETSpec,
    hq::MOSFETSpec,
    hq::OpAmpSpec,
    hq::TriodeSpec>;

struct CircuitUpdateCommand {
    CircuitUpdateKind kind = CircuitUpdateKind::resistance;
    std::uint16_t handle = 0;
    CircuitUpdatePayload payload = 0.0f;
};

// Single-producer/single-consumer queue intended for control-thread -> audio-thread
// block-boundary circuit edits. Storage is fixed at compile time and push/pop do
// not allocate. The producer must be unique and the consumer must be unique.
template <std::size_t Capacity>
class CircuitUpdateQueue {
public:
    static_assert(Capacity > 0U);

    bool tryPush(const CircuitUpdateCommand& command) noexcept {
        const std::size_t write = writeIndex_.load(std::memory_order_relaxed);
        const std::size_t read = readIndex_.load(std::memory_order_acquire);
        if (write - read >= Capacity) return false;
        storage_[write % Capacity] = command;
        writeIndex_.store(write + 1U, std::memory_order_release);
        return true;
    }

    bool tryPop(CircuitUpdateCommand& command) noexcept {
        const std::size_t read = readIndex_.load(std::memory_order_relaxed);
        const std::size_t write = writeIndex_.load(std::memory_order_acquire);
        if (read == write) return false;
        command = storage_[read % Capacity];
        readIndex_.store(read + 1U, std::memory_order_release);
        return true;
    }

    std::size_t pendingApprox() const noexcept {
        const std::size_t write = writeIndex_.load(std::memory_order_acquire);
        const std::size_t read = readIndex_.load(std::memory_order_acquire);
        return write - read;
    }

private:
    std::array<CircuitUpdateCommand, Capacity> storage_{};
    alignas(64) std::atomic<std::size_t> writeIndex_{0};
    alignas(64) std::atomic<std::size_t> readIndex_{0};
};

inline bool applyCircuitUpdate(MnaCircuitEngine& engine,
                               const CircuitUpdateCommand& command) noexcept {
    const auto scalar = std::get_if<float>(&command.payload);
    switch (command.kind) {
        case CircuitUpdateKind::voltageSource:
            return scalar != nullptr && engine.setVoltageSource(static_cast<SourceHandle>(command.handle), *scalar);
        case CircuitUpdateKind::resistance:
            return scalar != nullptr && engine.setResistance(static_cast<ResistorHandle>(command.handle), *scalar);
        case CircuitUpdateKind::resistorSpec: {
            const auto value = std::get_if<hq::ResistorSpec>(&command.payload);
            return value != nullptr && engine.setResistorSpec(static_cast<ResistorHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::capacitance:
            return scalar != nullptr && engine.setCapacitance(static_cast<CapacitorHandle>(command.handle), *scalar);
        case CircuitUpdateKind::capacitorSpec: {
            const auto value = std::get_if<hq::CapacitorSpec>(&command.payload);
            return value != nullptr && engine.setCapacitorSpec(static_cast<CapacitorHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::inductance:
            return scalar != nullptr && engine.setInductance(static_cast<InductorHandle>(command.handle), *scalar);
        case CircuitUpdateKind::inductorSpec: {
            const auto value = std::get_if<hq::InductorSpec>(&command.payload);
            return value != nullptr && engine.setInductorSpec(static_cast<InductorHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::potentiometerPosition:
            return scalar != nullptr && engine.setPotentiometerPosition(static_cast<PotHandle>(command.handle), *scalar);
        case CircuitUpdateKind::potentiometerSpec: {
            const auto value = std::get_if<hq::PotentiometerSpec>(&command.payload);
            return value != nullptr && engine.setPotentiometerSpec(static_cast<PotHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::vccsTransconductance:
            return scalar != nullptr && engine.setVccsTransconductance(static_cast<ControlledSourceHandle>(command.handle), *scalar);
        case CircuitUpdateKind::vcvsGain:
            return scalar != nullptr && engine.setVcvsGain(static_cast<ControlledSourceHandle>(command.handle), *scalar);
        case CircuitUpdateKind::cccsGain:
            return scalar != nullptr && engine.setCccsGain(static_cast<ControlledSourceHandle>(command.handle), *scalar);
        case CircuitUpdateKind::ccvsTransresistance:
            return scalar != nullptr && engine.setCcvsTransresistance(static_cast<ControlledSourceHandle>(command.handle), *scalar);
        case CircuitUpdateKind::diodeSpec: {
            const auto value = std::get_if<hq::DiodeSpec>(&command.payload);
            return value != nullptr && engine.setDiodeSpec(static_cast<DiodeHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::bjtSpec: {
            const auto value = std::get_if<hq::BJTSpec>(&command.payload);
            return value != nullptr && engine.setBjtSpec(static_cast<BjtHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::jfetSpec: {
            const auto value = std::get_if<hq::JFETSpec>(&command.payload);
            return value != nullptr && engine.setJfetSpec(static_cast<JfetHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::mosfetSpec: {
            const auto value = std::get_if<hq::MOSFETSpec>(&command.payload);
            return value != nullptr && engine.setMosfetSpec(static_cast<MosfetHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::opAmpSpec: {
            const auto value = std::get_if<hq::OpAmpSpec>(&command.payload);
            return value != nullptr && engine.setOpAmpSpec(static_cast<OpAmpHandle>(command.handle), *value);
        }
        case CircuitUpdateKind::triodeSpec: {
            const auto value = std::get_if<hq::TriodeSpec>(&command.payload);
            return value != nullptr && engine.setTriodeSpec(static_cast<TriodeHandle>(command.handle), *value);
        }
    }
    return false;
}

template <std::size_t Capacity>
std::size_t applyPendingCircuitUpdates(MnaCircuitEngine& engine,
                                       CircuitUpdateQueue<Capacity>& queue,
                                       std::size_t maximumCommands = Capacity) noexcept {
    CircuitUpdateCommand command{};
    std::size_t applied = 0;
    while (applied < maximumCommands && queue.tryPop(command)) {
        applyCircuitUpdate(engine, command);
        ++applied;
    }
    return applied;
}

} // namespace guitardsp::circuit
