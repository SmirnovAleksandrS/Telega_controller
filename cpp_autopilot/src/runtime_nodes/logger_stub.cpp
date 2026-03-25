#include "telega_cpp_autopilot/runtime_nodes/logger_stub.hpp"

namespace telega::autopilot {

void LoggerStub::reset() {
    last_velocity_.reset();
    last_pose_.reset();
}

const std::optional<datPosition>& LoggerStub::lastPose() const {
    return last_pose_;
}

void LoggerStub::HandleVelocity(void* ctx, const TopicSampleRef& sample) {
    static_cast<LoggerStub*>(ctx)->onVelocity(sample);
}

void LoggerStub::HandlePose(void* ctx, const TopicSampleRef& sample) {
    static_cast<LoggerStub*>(ctx)->onPose(sample);
}

void LoggerStub::onVelocity(const TopicSampleRef& sample) {
    const auto* velocity = static_cast<const datVelocity*>(sample.payload);
    if (velocity != nullptr) {
        last_velocity_ = *velocity;
    }
}

void LoggerStub::onPose(const TopicSampleRef& sample) {
    const auto* pose = static_cast<const datPosition*>(sample.payload);
    if (pose != nullptr) {
        last_pose_ = *pose;
    }
}

}  // namespace telega::autopilot
