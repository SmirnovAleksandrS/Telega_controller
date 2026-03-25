#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "telega_cpp_autopilot/event_payloads.hpp"

namespace telega::autopilot {

// Minimal bundle returned to the outer runtime bridge after one graph step.
// At this stage the bridge only needs pose and PWM to talk back to the GUI.
struct RuntimeOutputs {
    std::optional<datPosition> pose;
    std::optional<datPwmCommand> pwm_command;
};

// Public façade for the current autopilot graph skeleton. The implementation is
// hidden behind `Impl` so node layout can evolve without leaking dependencies
// into the GUI/runtime bridge code.
class RuntimeGraph {
public:
    RuntimeGraph();
    ~RuntimeGraph();
    RuntimeGraph(RuntimeGraph&&) noexcept;
    RuntimeGraph& operator=(RuntimeGraph&&) noexcept;
    RuntimeGraph(const RuntimeGraph&) = delete;
    RuntimeGraph& operator=(const RuntimeGraph&) = delete;

    void reset();
    RuntimeOutputs processTelemetryEvent(std::uint32_t timestamp_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace telega::autopilot
