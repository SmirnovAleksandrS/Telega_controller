#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "telega_cpp_autopilot/event_payloads.hpp"

namespace telega::autopilot {

// Stable identifiers for all stage-1 topics. The numeric ordering is also the
// dispatch priority because the bus walks the bitMask from low bit to high bit.
enum class TopicId : std::uint8_t {
    kVelocity = 0,
    kPose = 1,
    kTick = 2,
    kDecisionTrigger = 3,
    kPwmCommand = 4,
    kHealth = 5,
    kTopicCount = 6,
};

enum class SubscriberMode : std::uint8_t {
    kInline,
    kDeferredQueue,
};

// Topic behavior flags used by the runtime graph when it registers the topology.
enum TopicFlags : std::uint16_t {
    kTopicLatestWins = 1U << 0,
    kTopicAllowISRPublish = 1U << 1,
    kTopicTraceEnabled = 1U << 2,
    kTopicHotPathMirror = 1U << 3,
};

// Lightweight view passed to subscribers. The bus keeps ownership of the payload
// memory, so handlers must not retain the raw pointer beyond the callback.
struct TopicSampleRef {
    TopicId topic = TopicId::kVelocity;
    const void* payload = nullptr;
    std::uint32_t sequence = 0;
    std::uint32_t publish_timestamp = 0;
};

using SubscriberFn = void (*)(void* ctx, const TopicSampleRef& sample);

// One subscriber entry inside a topic descriptor. `context` is intentionally
// untyped so the bus stays free of higher-level node dependencies.
struct SubscriberBinding {
    const char* name = "";
    SubscriberMode mode = SubscriberMode::kInline;
    SubscriberFn callback = nullptr;
    void* context = nullptr;
    std::uint8_t queue_id = 0xFFU;
};

// Static metadata for a topic. RuntimeGraph builds one registry table and then
// hands it to EventBus during initialization.
struct TopicDescriptor {
    TopicId id = TopicId::kVelocity;
    const char* name = "";
    std::uint8_t priority = 0;
    std::uint16_t payload_size = 0;
    std::uint16_t payload_align = 0;
    std::uint16_t flags = 0;
    const SubscriberBinding* subscribers = nullptr;
    std::uint8_t subscriber_count = 0;
};

// Per-topic runtime state that changes on every publish/dispatch cycle.
struct TopicRuntimeState {
    std::uint32_t sequence = 0;
    std::uint32_t last_publish_timestamp = 0;
    std::uint8_t active_slot = 0;
    bool pending = false;
};

// One storage block holds two slots so the publisher can write the next sample
// without invalidating the currently active slot until the swap is complete.
struct TopicStorageBlock {
    alignas(kEventBusSlotAlignment) std::array<std::byte, kEventBusSlotSize * 2U> bytes {};
};

// Result returned by publish APIs so callers can detect rejected samples and
// "latest wins" overwrites while a previous sample is still pending.
struct PublishResult {
    bool accepted = false;
    bool overwritten_previous = false;
    std::uint32_t new_sequence = 0;
};

// Small runtime counter bundle that makes it easy to inspect how much work a
// dispatch cycle performed.
struct DispatchStats {
    std::uint32_t topics_dispatched = 0;
    std::uint32_t subscriber_calls = 0;
    std::uint32_t deferred_enqueues = 0;
};

constexpr std::uint8_t TopicIndex(TopicId id) {
    return static_cast<std::uint8_t>(id);
}

static_assert(TopicIndex(TopicId::kTopicCount) <= 64U, "EventBus bitMask supports at most 64 topics.");

class EventBus {
public:
    // `registry`, `runtime`, and `storage` are owned by the caller. EventBus only
    // stores raw pointers to those buffers and never reallocates them.
    void init(
        const TopicDescriptor* registry,
        TopicRuntimeState* runtime,
        TopicStorageBlock* storage,
        std::uint8_t topic_count
    );

    void clear();

    // Untyped publish entry point used by the typed wrappers and by any future
    // dynamic tooling. The payload must match the descriptor exactly.
    PublishResult publish(TopicId topic, const void* payload, std::uint16_t payload_size, std::uint32_t publish_timestamp);
    PublishResult publishFromISR(
        TopicId topic,
        const void* payload,
        std::uint16_t payload_size,
        std::uint32_t publish_timestamp
    );

    DispatchStats dispatchPending(std::uint8_t max_topics = 0xFFU);
    bool dispatchOne(TopicId topic, DispatchStats* stats = nullptr);

    TopicSampleRef getLatest(TopicId topic) const;

    template <TopicId Id, typename Payload>
    PublishResult publish(const Payload& payload, std::uint32_t publish_timestamp) {
        static_assert(std::is_trivially_copyable_v<Payload>, "EventBus payloads must be trivially copyable.");
        static_assert(std::is_standard_layout_v<Payload>, "EventBus payloads must use standard layout.");
        static_assert(sizeof(Payload) <= kEventBusSlotSize, "Payload does not fit in the event bus slot.");
        static_assert(alignof(Payload) <= kEventBusSlotAlignment, "Payload alignment exceeds event bus storage.");
        return publish(Id, &payload, static_cast<std::uint16_t>(sizeof(Payload)), publish_timestamp);
    }

    template <TopicId Id, typename Payload>
    PublishResult publishFromISR(const Payload& payload, std::uint32_t publish_timestamp) {
        static_assert(std::is_trivially_copyable_v<Payload>, "EventBus payloads must be trivially copyable.");
        static_assert(std::is_standard_layout_v<Payload>, "EventBus payloads must use standard layout.");
        static_assert(sizeof(Payload) <= kEventBusSlotSize, "Payload does not fit in the event bus slot.");
        static_assert(alignof(Payload) <= kEventBusSlotAlignment, "Payload alignment exceeds event bus storage.");
        return publishFromISR(Id, &payload, static_cast<std::uint16_t>(sizeof(Payload)), publish_timestamp);
    }

    template <typename Payload>
    const Payload* getLatestTyped(TopicId topic) const {
        static_assert(std::is_trivially_copyable_v<Payload>, "EventBus payloads must be trivially copyable.");
        static_assert(std::is_standard_layout_v<Payload>, "EventBus payloads must use standard layout.");
        const TopicSampleRef sample = getLatest(topic);
        return static_cast<const Payload*>(sample.payload);
    }

    template <TopicId Id, typename Payload>
    const Payload* getLatestTyped() const {
        return getLatestTyped<Payload>(Id);
    }

    bool isPending(TopicId topic) const;
    std::uint64_t bitMask() const;

private:
    const TopicDescriptor& descriptor(TopicId topic) const;
    TopicRuntimeState& runtimeState(TopicId topic);
    const TopicRuntimeState& runtimeState(TopicId topic) const;
    void* slotPointer(TopicId topic, std::uint8_t slot_index);
    const void* slotPointer(TopicId topic, std::uint8_t slot_index) const;
    std::uint32_t publishImpl(
        TopicId topic,
        const void* payload,
        std::uint16_t payload_size,
        std::uint32_t publish_timestamp,
        bool from_isr
    );

    const TopicDescriptor* registry_ = nullptr;
    TopicRuntimeState* runtime_ = nullptr;
    TopicStorageBlock* storage_ = nullptr;
    std::uint8_t topic_count_ = 0;
    std::uint64_t bitMask_ = 0;
};

}  // namespace telega::autopilot
