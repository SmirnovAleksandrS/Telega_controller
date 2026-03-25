#include "telega_cpp_autopilot/runtime_nodes/autopilot_logic_stub.hpp"

namespace telega::autopilot {

AutopilotLogicStub::AutopilotLogicStub(EventBus* bus)
    : bus_(bus) {
}

void AutopilotLogicStub::bind(EventBus* bus) {
    bus_ = bus;
}

void AutopilotLogicStub::reset() {
    last_command_.reset();
}

const std::optional<datPwmCommand>& AutopilotLogicStub::lastCommand() const {
    return last_command_;
}

void AutopilotLogicStub::HandleDecisionTrigger(void* ctx, const TopicSampleRef& sample) {
    static_cast<AutopilotLogicStub*>(ctx)->onDecisionTrigger(sample);
}

void AutopilotLogicStub::onDecisionTrigger(const TopicSampleRef& sample) {
    if (bus_ == nullptr) {
        return;
    }

    // The real controller will calculate meaningful actuator commands. For now
    // the stub publishes a neutral PWM pair so the GUI bridge can be exercised.
    datPwmCommand command;
    command.timestamp = sample.publish_timestamp;
    command.left_pwm = 1500;
    command.right_pwm = 1500;
    last_command_ = command;
    bus_->publish<TopicId::kPwmCommand>(command, sample.publish_timestamp);
}

}  // namespace telega::autopilot
