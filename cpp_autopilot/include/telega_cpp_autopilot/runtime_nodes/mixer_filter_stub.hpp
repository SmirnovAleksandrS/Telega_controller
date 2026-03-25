#pragma once

#include "telega_cpp_autopilot/runtime_node_interfaces.hpp"

namespace telega::autopilot {

class EventBus;

// Stage-1 placeholder for the future mixer/filter block from the diagrams.
// It receives a raw ingress packet, synthesizes a velocity sample, publishes it
// into the bus, and forwards the typed sample down the hot path chain.
class MixerFilterStub final : public IngressPacketConsumer {
public:
    void bind(EventBus* bus, VelocityConsumer* next);
    void receiveNewData(const datIngressPacket& packet) override;

private:
    EventBus* bus_ = nullptr;
    VelocityConsumer* next_ = nullptr;
};

}  // namespace telega::autopilot
