#pragma once

#include <cstdint>

#include "telega_cpp_autopilot/event_payloads.hpp"

namespace telega::autopilot {

// Consumer of the raw ingress packet. This is the first hot-path handoff after
// a UART/ISR-style source accepts new external data.
class IngressPacketConsumer {
public:
    virtual ~IngressPacketConsumer() = default;
    virtual void receiveNewData(const datIngressPacket& packet) = 0;
};

// Consumer of velocity data produced by the mixer/filter stage.
class VelocityConsumer {
public:
    virtual ~VelocityConsumer() = default;
    virtual void receiveNewData(const datVelocity& velocity) = 0;
};

// Minimal interface for components that can emit a scheduling/control tick.
class TickPublisher {
public:
    virtual ~TickPublisher() = default;
    virtual void publish(std::uint32_t timestamp) = 0;
};

}  // namespace telega::autopilot
