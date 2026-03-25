#include "telega_cpp_autopilot/runtime_nodes/trigger_stub.hpp"

namespace telega::autopilot {
namespace {

constexpr std::uint16_t kTriggerThreshold = 21U;
constexpr std::uint64_t kVelocityCauseBit = std::uint64_t {1} << TopicIndex(TopicId::kVelocity);
constexpr std::uint64_t kPoseCauseBit = std::uint64_t {1} << TopicIndex(TopicId::kPose);
constexpr std::uint64_t kTickCauseBit = std::uint64_t {1} << TopicIndex(TopicId::kTick);

}  // namespace

TriggerStub::TriggerStub(EventBus* bus)
    : bus_(bus) {
}

void TriggerStub::bind(EventBus* bus) {
    bus_ = bus;
}

void TriggerStub::reset() {
    velocity_weight_ = 0U;
    pose_weight_ = 0U;
    seen_velocity_ = false;
    seen_pose_ = false;
}

void TriggerStub::HandleVelocity(void* ctx, const TopicSampleRef& sample) {
    static_cast<TriggerStub*>(ctx)->onVelocity(sample);
}

void TriggerStub::HandlePose(void* ctx, const TopicSampleRef& sample) {
    static_cast<TriggerStub*>(ctx)->onPose(sample);
}

void TriggerStub::HandleTick(void* ctx, const TopicSampleRef& sample) {
    static_cast<TriggerStub*>(ctx)->onTick(sample);
}

void TriggerStub::onVelocity(const TopicSampleRef& sample) {
    const auto* velocity = static_cast<const datVelocity*>(sample.payload);
    if (velocity == nullptr) {
        return;
    }

    velocity_weight_ = velocity->getWeight();
    seen_velocity_ = true;
}

void TriggerStub::onPose(const TopicSampleRef& sample) {
    const auto* pose = static_cast<const datPosition*>(sample.payload);
    if (pose == nullptr) {
        return;
    }

    pose_weight_ = pose->getWeight();
    seen_pose_ = true;
}

void TriggerStub::onTick(const TopicSampleRef& sample) {
    if (bus_ == nullptr || !seen_velocity_ || !seen_pose_) {
        return;
    }

    // This keeps the intended trigger seam visible in the skeleton: the block
    // observes several upstream topics, aggregates a score, and conditionally
    // emits a single event for downstream control logic.
    datTriggerEvent event;
    event.timestamp = sample.publish_timestamp;
    event.cause_mask = kVelocityCauseBit | kPoseCauseBit | kTickCauseBit;
    event.total_weight = static_cast<std::uint16_t>(velocity_weight_ + pose_weight_);
    event.threshold = kTriggerThreshold;
    if (event.total_weight >= event.threshold) {
        bus_->publish<TopicId::kDecisionTrigger>(event, sample.publish_timestamp);
    }

    seen_velocity_ = false;
    seen_pose_ = false;
}

}  // namespace telega::autopilot
