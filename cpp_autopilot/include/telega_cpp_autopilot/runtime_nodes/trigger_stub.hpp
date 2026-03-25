#pragma once

#include <cstdint>

#include "telega_cpp_autopilot/event_bus.hpp"

namespace telega::autopilot {

class EventBus;

// Stub trigger block. It watches velocity/pose/tick topics and emits a
// decision-trigger event when the accumulated placeholder weight crosses the
// current threshold.
class TriggerStub final {
public:
    explicit TriggerStub(EventBus* bus = nullptr);

    void bind(EventBus* bus);
    void reset();

    static void HandleVelocity(void* ctx, const TopicSampleRef& sample);
    static void HandlePose(void* ctx, const TopicSampleRef& sample);
    static void HandleTick(void* ctx, const TopicSampleRef& sample);

private:
    void onVelocity(const TopicSampleRef& sample);
    void onPose(const TopicSampleRef& sample);
    void onTick(const TopicSampleRef& sample);

    EventBus* bus_ = nullptr;
    std::uint16_t velocity_weight_ = 0U;
    std::uint16_t pose_weight_ = 0U;
    bool seen_velocity_ = false;
    bool seen_pose_ = false;
};

}  // namespace telega::autopilot
