#include "telega_cpp_autopilot/runtime_nodes/uart_rx_stub.hpp"

#include "telega_cpp_autopilot/event_bus.hpp"

namespace telega::autopilot {

void UartRxStub::bind(EventBus* bus, IngressPacketConsumer* pipeline_entry, TickPublisher* tick_source) {
    bus_ = bus;
    pipeline_entry_ = pipeline_entry;
    tick_source_ = tick_source;
}

void UartRxStub::publishFromISR(const datIngressPacket& packet) {
    // Stage-1 skeleton keeps a single inbox slot. That matches the "interrupt
    // arrives, main loop drains later" mental model from the diagrams.
    inbox_ = packet;
    have_packet_ = true;
}

void UartRxStub::process() {
    if (!have_packet_) {
        return;
    }

    const datIngressPacket packet = inbox_;
    have_packet_ = false;

    // The graph uses a hybrid flow: direct hot-path calls for deterministic
    // upstream chaining, then event bus dispatch for fan-out subscribers.
    if (pipeline_entry_ != nullptr) {
        pipeline_entry_->receiveNewData(packet);
    }
    if (tick_source_ != nullptr) {
        tick_source_->publish(packet.timestamp);
    }
    if (bus_ != nullptr) {
        bus_->dispatchPending();
    }
}

}  // namespace telega::autopilot
