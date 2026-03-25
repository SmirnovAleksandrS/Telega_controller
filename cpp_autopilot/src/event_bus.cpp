#include "telega_cpp_autopilot/event_bus.hpp"

#include <cstdint>

namespace telega::autopilot {
namespace {

constexpr std::uint64_t TopicBit(TopicId topic) {
    return std::uint64_t {1} << TopicIndex(topic);
}

// Stage-1 bus keeps the pending set inside one bit mask. The lowest set bit
// corresponds to the highest-priority pending topic because TopicId order is
// also the dispatch order.
std::uint8_t LowestPendingTopic(std::uint64_t bit_mask) {
    std::uint8_t index = 0;
    while ((bit_mask & std::uint64_t {1}) == 0U) {
        bit_mask >>= 1U;
        ++index;
    }
    return index;
}

}  // namespace

void EventBus::init(
    const TopicDescriptor* registry,
    TopicRuntimeState* runtime,
    TopicStorageBlock* storage,
    std::uint8_t topic_count
) {
    registry_ = registry;
    runtime_ = runtime;
    storage_ = storage;
    topic_count_ = topic_count;
    // `clear()` resets both runtime metadata and storage bytes so the graph can
    // start from a deterministic state after every init/reset cycle.
    clear();
}

void EventBus::clear() {
    bitMask_ = 0;
    if (runtime_ == nullptr || storage_ == nullptr) {
        return;
    }
    for (std::uint8_t i = 0; i < topic_count_; ++i) {
        runtime_[i] = TopicRuntimeState {};
        storage_[i] = TopicStorageBlock {};
    }
}

PublishResult EventBus::publish(
    TopicId topic,
    const void* payload,
    std::uint16_t payload_size,
    std::uint32_t publish_timestamp
) {
    PublishResult result;
    // If the topic is already pending, the new sample will replace the previous
    // undelivered one under the "latest wins" rule used by the current skeleton.
    const bool had_pending_sample = isPending(topic);
    result.new_sequence = publishImpl(topic, payload, payload_size, publish_timestamp, false);
    result.accepted = result.new_sequence != 0U;
    result.overwritten_previous = result.accepted && had_pending_sample;
    return result;
}

PublishResult EventBus::publishFromISR(
    TopicId topic,
    const void* payload,
    std::uint16_t payload_size,
    std::uint32_t publish_timestamp
) {
    PublishResult result;
    const bool had_pending_sample = isPending(topic);
    result.new_sequence = publishImpl(topic, payload, payload_size, publish_timestamp, true);
    result.accepted = result.new_sequence != 0U;
    result.overwritten_previous = result.accepted && had_pending_sample;
    return result;
}

DispatchStats EventBus::dispatchPending(std::uint8_t max_topics) {
    DispatchStats stats;

    // The loop always re-reads `bitMask_` so subscribers are allowed to publish
    // new topics while dispatch is in progress. Those new bits are then picked
    // up in the same pass as long as `max_topics` still allows it.
    while (bitMask_ != 0U && stats.topics_dispatched < max_topics) {
        const TopicId topic = static_cast<TopicId>(LowestPendingTopic(bitMask_));
        if (dispatchOne(topic, &stats)) {
            ++stats.topics_dispatched;
        }
    }

    return stats;
}

bool EventBus::dispatchOne(TopicId topic, DispatchStats* stats) {
    if (registry_ == nullptr || runtime_ == nullptr || storage_ == nullptr || TopicIndex(topic) >= topic_count_) {
        return false;
    }
    if (!isPending(topic)) {
        return false;
    }

    TopicRuntimeState& topic_state = runtimeState(topic);
    const TopicDescriptor& topic_descriptor = descriptor(topic);
    const TopicSampleRef sample {
        topic,
        slotPointer(topic, topic_state.active_slot),
        topic_state.sequence,
        topic_state.last_publish_timestamp,
    };

    topic_state.pending = false;
    bitMask_ &= ~TopicBit(topic);

    // Inline subscribers are executed immediately. Deferred subscribers are only
    // counted for now; a real queue can be plugged in later without changing the
    // descriptor format or the bus/public API.
    for (std::uint8_t i = 0; i < topic_descriptor.subscriber_count; ++i) {
        const SubscriberBinding& subscriber = topic_descriptor.subscribers[i];
        if (subscriber.mode == SubscriberMode::kDeferredQueue) {
            if (stats != nullptr) {
                ++stats->deferred_enqueues;
            }
            continue;
        }
        if (subscriber.callback != nullptr) {
            subscriber.callback(subscriber.context, sample);
            if (stats != nullptr) {
                ++stats->subscriber_calls;
            }
        }
    }

    return true;
}

TopicSampleRef EventBus::getLatest(TopicId topic) const {
    if (registry_ == nullptr || runtime_ == nullptr || storage_ == nullptr || TopicIndex(topic) >= topic_count_) {
        return TopicSampleRef {};
    }

    const TopicRuntimeState& topic_state = runtimeState(topic);
    return TopicSampleRef {
        topic,
        slotPointer(topic, topic_state.active_slot),
        topic_state.sequence,
        topic_state.last_publish_timestamp,
    };
}

bool EventBus::isPending(TopicId topic) const {
    if (TopicIndex(topic) >= topic_count_) {
        return false;
    }
    return (bitMask_ & TopicBit(topic)) != 0U;
}

std::uint64_t EventBus::bitMask() const {
    return bitMask_;
}

const TopicDescriptor& EventBus::descriptor(TopicId topic) const {
    return registry_[TopicIndex(topic)];
}

TopicRuntimeState& EventBus::runtimeState(TopicId topic) {
    return runtime_[TopicIndex(topic)];
}

const TopicRuntimeState& EventBus::runtimeState(TopicId topic) const {
    return runtime_[TopicIndex(topic)];
}

void* EventBus::slotPointer(TopicId topic, std::uint8_t slot_index) {
    return storage_[TopicIndex(topic)].bytes.data() + (slot_index * kEventBusSlotSize);
}

const void* EventBus::slotPointer(TopicId topic, std::uint8_t slot_index) const {
    return storage_[TopicIndex(topic)].bytes.data() + (slot_index * kEventBusSlotSize);
}

std::uint32_t EventBus::publishImpl(
    TopicId topic,
    const void* payload,
    std::uint16_t payload_size,
    std::uint32_t publish_timestamp,
    bool from_isr
) {
    if (registry_ == nullptr || runtime_ == nullptr || storage_ == nullptr || payload == nullptr) {
        return 0U;
    }
    if (TopicIndex(topic) >= topic_count_) {
        return 0U;
    }

    const TopicDescriptor& topic_descriptor = descriptor(topic);
    if (payload_size != topic_descriptor.payload_size) {
        return 0U;
    }
    if (payload_size > kEventBusSlotSize) {
        return 0U;
    }
    if (from_isr && (topic_descriptor.flags & kTopicAllowISRPublish) == 0U) {
        return 0U;
    }

    TopicRuntimeState& topic_state = runtimeState(topic);
    const std::uint8_t next_slot = static_cast<std::uint8_t>(1U - topic_state.active_slot);
    // Copy into the inactive slot first, then swap the active index. This keeps
    // the visible sample coherent even if the caller asks for the latest payload
    // immediately after publish.
    std::memcpy(slotPointer(topic, next_slot), payload, payload_size);
    topic_state.active_slot = next_slot;
    topic_state.last_publish_timestamp = publish_timestamp;
    ++topic_state.sequence;
    topic_state.pending = true;
    bitMask_ |= TopicBit(topic);
    return topic_state.sequence;
}

}  // namespace telega::autopilot
