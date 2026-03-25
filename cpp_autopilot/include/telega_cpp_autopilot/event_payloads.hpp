#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace telega::autopilot {

// Generic pair used by the original diagrams. The exact semantics of A/K will be
// defined later by the real math blocks, so the stub preserves the same shape.
struct FPair {
    float A = 0.0F;
    float K = 0.0F;
};

// Raw ingress packet accepted from the external world before the payload is
// expanded into typed velocity/pose topics.
struct datIngressPacket {
    std::uint32_t timestamp = 0;
    std::uint32_t pc_time_ms = 0;
};

// Placeholder output of the future mixer/filter block.
struct datVelocity {
    std::uint32_t timestamp = 0;
    FPair vX {};
    FPair vY {};
    FPair vZ {};
    FPair vPhi {};
    FPair vPsi {};
    FPair vTheta {};

    std::uint16_t getWeight() const {
        return 10U;
    }
};

// Placeholder output of the future integrator block.
struct datPosition {
    std::uint32_t timestamp = 0;
    FPair X {};
    FPair Y {};
    FPair Z {};
    FPair Phi {};
    FPair Psi {};
    FPair Theta {};

    std::uint16_t getWeight() const {
        return 10U;
    }
};

// Generic scheduling tick used by trigger-like logic.
struct datTick {
    std::uint32_t timestamp = 0;
    std::uint32_t dt_us = 0;

    std::uint16_t getWeight() const {
        return 1U;
    }
};

// Event emitted when the trigger decides that downstream control logic should run.
struct datTriggerEvent {
    std::uint32_t timestamp = 0;
    std::uint64_t cause_mask = 0;
    std::uint16_t total_weight = 0;
    std::uint16_t threshold = 0;
};

// Current stage-1 control output mirrored back to the GUI.
struct datPwmCommand {
    std::uint32_t timestamp = 0;
    std::int16_t left_pwm = 1500;
    std::int16_t right_pwm = 1500;
};

// Reserved status topic for future watchdog/error reporting.
struct datHealthStatus {
    std::uint32_t timestamp = 0;
    std::uint32_t error_mask = 0;
    std::uint32_t stale_topics_mask_low = 0;
    std::uint32_t stale_topics_mask_high = 0;
};

// The bus uses one fixed-size double buffer per topic. The slot size therefore
// must fit the largest registered payload type at compile time.
constexpr std::size_t kEventBusSlotSize = std::max({
    sizeof(datVelocity),
    sizeof(datPosition),
    sizeof(datTick),
    sizeof(datTriggerEvent),
    sizeof(datPwmCommand),
    sizeof(datHealthStatus),
});

// Every payload copied through the bus must also fit the common alignment.
constexpr std::size_t kEventBusSlotAlignment = std::max({
    alignof(datVelocity),
    alignof(datPosition),
    alignof(datTick),
    alignof(datTriggerEvent),
    alignof(datPwmCommand),
    alignof(datHealthStatus),
});

static_assert(std::is_trivially_copyable_v<FPair>, "Event bus payload fields must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datIngressPacket>, "Ingress payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datVelocity>, "Velocity payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datPosition>, "Position payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datTick>, "Tick payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datTriggerEvent>, "Trigger payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datPwmCommand>, "PWM payload must be trivially copyable.");
static_assert(std::is_trivially_copyable_v<datHealthStatus>, "Health payload must be trivially copyable.");

}  // namespace telega::autopilot
