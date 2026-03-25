#include "telega_cpp_autopilot/runtime_nodes/integrator_stub.hpp"

#include "telega_cpp_autopilot/event_bus.hpp"

namespace telega::autopilot {

void IntegratorStub::bind(EventBus* bus) {
    bus_ = bus;
}

void IntegratorStub::receiveNewData(const datVelocity& velocity) {
    // The real integrator will convert velocity into position/orientation. The
    // stub only preserves the interface and emits a predictable placeholder pose.
    datPosition position;
    position.timestamp = velocity.timestamp;
    position.X = {0.0F, 1.0F};
    position.Y = {0.0F, 0.0F};
    position.Z = {0.0F, 0.0F};
    position.Phi = {0.0F, 0.0F};
    position.Psi = {0.0F, 0.0F};
    position.Theta = {0.0F, 1.0F};

    if (bus_ != nullptr) {
        bus_->publish<TopicId::kPose>(position, velocity.timestamp);
    }
}

}  // namespace telega::autopilot
