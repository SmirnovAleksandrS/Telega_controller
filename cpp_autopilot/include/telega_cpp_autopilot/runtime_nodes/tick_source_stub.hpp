#pragma once

#include "telega_cpp_autopilot/runtime_node_interfaces.hpp"

namespace telega::autopilot {

class EventBus;

// Simple tick source used by the graph skeleton. The real system can later swap
// this out for a hardware timer or scheduler-driven publisher.
class TickSourceStub final : public TickPublisher {
public:
    void bind(EventBus* bus);
    void publish(std::uint32_t timestamp) override;

private:
    EventBus* bus_ = nullptr;
};

}  // namespace telega::autopilot
