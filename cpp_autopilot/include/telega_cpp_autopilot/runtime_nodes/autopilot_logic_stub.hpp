#pragma once

#include <optional>

#include "telega_cpp_autopilot/event_bus.hpp"
#include "telega_cpp_autopilot/event_payloads.hpp"

namespace telega::autopilot {

class EventBus;

// Placeholder for the future main autopilot logic. At this stage it only reacts
// to the trigger topic and publishes a neutral PWM command.
class AutopilotLogicStub final {
public:
    explicit AutopilotLogicStub(EventBus* bus = nullptr);

    void bind(EventBus* bus);
    void reset();

    const std::optional<datPwmCommand>& lastCommand() const;

    static void HandleDecisionTrigger(void* ctx, const TopicSampleRef& sample);

private:
    void onDecisionTrigger(const TopicSampleRef& sample);

    EventBus* bus_ = nullptr;
    std::optional<datPwmCommand> last_command_;
};

}  // namespace telega::autopilot
