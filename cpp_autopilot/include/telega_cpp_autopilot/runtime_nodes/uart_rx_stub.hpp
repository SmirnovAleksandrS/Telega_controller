#pragma once

#include "telega_cpp_autopilot/runtime_node_interfaces.hpp"

namespace telega::autopilot {

class EventBus;

// UART-like ingress stub. It stores one pending packet as if it arrived from an
// ISR and then flushes that packet through the hot path and event bus on demand.
class UartRxStub final {
public:
    void bind(EventBus* bus, IngressPacketConsumer* pipeline_entry, TickPublisher* tick_source);

    void publishFromISR(const datIngressPacket& packet);
    void process();

private:
    EventBus* bus_ = nullptr;
    IngressPacketConsumer* pipeline_entry_ = nullptr;
    TickPublisher* tick_source_ = nullptr;
    datIngressPacket inbox_ {};
    bool have_packet_ = false;
};

}  // namespace telega::autopilot
