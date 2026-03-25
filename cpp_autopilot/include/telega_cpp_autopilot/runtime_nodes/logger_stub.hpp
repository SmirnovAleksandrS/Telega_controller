#pragma once

#include <optional>

#include "telega_cpp_autopilot/event_bus.hpp"
#include "telega_cpp_autopilot/event_payloads.hpp"

namespace telega::autopilot {

// Temporary logger/cache node used by the runtime bridge. It keeps the latest
// bus samples so `RuntimeGraph` can expose them back to the GUI layer.
class LoggerStub final {
public:
    void reset();

    const std::optional<datPosition>& lastPose() const;

    static void HandleVelocity(void* ctx, const TopicSampleRef& sample);
    static void HandlePose(void* ctx, const TopicSampleRef& sample);

private:
    void onVelocity(const TopicSampleRef& sample);
    void onPose(const TopicSampleRef& sample);

    std::optional<datVelocity> last_velocity_;
    std::optional<datPosition> last_pose_;
};

}  // namespace telega::autopilot
