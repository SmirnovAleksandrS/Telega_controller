#include "telega_cpp_autopilot/runtime_nodes/tick_source_stub.hpp"

#include "telega_cpp_autopilot/event_bus.hpp"

namespace telega::autopilot {

void TickSourceStub::bind(EventBus* bus) {
    bus_ = bus;
}

void TickSourceStub::publish(std::uint32_t timestamp) {
    if (bus_ == nullptr) {
        return;
    }

    datTick tick;
    tick.timestamp = timestamp;
    tick.dt_us = 1000U;
    bus_->publish<TopicId::kTick>(tick, timestamp);
}

}  // namespace telega::autopilot
