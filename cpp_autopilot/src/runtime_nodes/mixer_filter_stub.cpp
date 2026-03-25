#include "telega_cpp_autopilot/runtime_nodes/mixer_filter_stub.hpp"

#include "telega_cpp_autopilot/event_bus.hpp"

namespace telega::autopilot {

void MixerFilterStub::bind(EventBus* bus, VelocityConsumer* next) {
    bus_ = bus;
    next_ = next;
}

void MixerFilterStub::receiveNewData(const datIngressPacket& packet) {
    // The stub keeps the shape of the future block without implementing real
    // filtering yet. It simply stamps a deterministic sample for downstream
    // plumbing and integration tests.
    datVelocity velocity;
    velocity.timestamp = packet.timestamp;
    velocity.vX = {0.0F, 1.0F};
    velocity.vY = {0.0F, 0.0F};
    velocity.vZ = {0.0F, 0.0F};
    velocity.vPhi = {0.0F, 0.0F};
    velocity.vPsi = {0.0F, 0.0F};
    velocity.vTheta = {0.0F, 1.0F};

    if (bus_ != nullptr) {
        bus_->publish<TopicId::kVelocity>(velocity, packet.timestamp);
    }
    if (next_ != nullptr) {
        next_->receiveNewData(velocity);
    }
}

}  // namespace telega::autopilot
