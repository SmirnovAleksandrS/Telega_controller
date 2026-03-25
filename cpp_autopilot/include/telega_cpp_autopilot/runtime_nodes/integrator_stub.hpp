#pragma once

#include "telega_cpp_autopilot/runtime_node_interfaces.hpp"

namespace telega::autopilot {

class EventBus;

// Stage-1 placeholder for the future integrator. It accepts velocity, emits a
// synthetic pose sample, and mirrors that pose into the event bus.
class IntegratorStub final : public VelocityConsumer {
public:
    void bind(EventBus* bus);
    void receiveNewData(const datVelocity& velocity) override;

private:
    EventBus* bus_ = nullptr;
};

}  // namespace telega::autopilot
