#include "telega_cpp_autopilot/runtime_graph.hpp"

#include <array>
#include <cstdint>
#include <memory>

#include "telega_cpp_autopilot/event_bus.hpp"
#include "telega_cpp_autopilot/runtime_nodes/autopilot_logic_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/integrator_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/logger_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/mixer_filter_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/tick_source_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/trigger_stub.hpp"
#include "telega_cpp_autopilot/runtime_nodes/uart_rx_stub.hpp"

namespace telega::autopilot {
namespace {

constexpr std::size_t kTopicCount = static_cast<std::size_t>(TopicIndex(TopicId::kTopicCount));

// Small helper that keeps topic registration readable inside the graph builder.
TopicDescriptor MakeTopicDescriptor(
    TopicId id,
    const char* name,
    std::uint16_t payload_size,
    std::uint16_t payload_align,
    std::uint16_t flags,
    const SubscriberBinding* subscribers,
    std::uint8_t subscriber_count
) {
    return TopicDescriptor {
        id,
        name,
        TopicIndex(id),
        payload_size,
        payload_align,
        flags,
        subscribers,
        subscriber_count,
    };
}

}  // namespace

struct RuntimeGraph::Impl {
    Impl() {
        // Hot path wiring: raw packet -> mixer/filter -> integrator.
        mixer_filter.bind(&bus, &integrator);
        integrator.bind(&bus);

        // Tick and ingress are separate because the diagrams model them as
        // different sources that converge later in the trigger/bus layers.
        tick_source.bind(&bus);
        trigger.bind(&bus);
        autopilot_logic.bind(&bus);
        uart_rx.bind(&bus, &mixer_filter, &tick_source);

        // Subscriber arrays stay local to the graph so the event bus remains a
        // generic data mover and knows nothing about concrete node classes.
        velocity_subscribers[0] = SubscriberBinding {
            "TriggerStub",
            SubscriberMode::kInline,
            &TriggerStub::HandleVelocity,
            &trigger,
        };
        velocity_subscribers[1] = SubscriberBinding {
            "LoggerStub",
            SubscriberMode::kInline,
            &LoggerStub::HandleVelocity,
            &logger,
        };
        pose_subscribers[0] = SubscriberBinding {
            "TriggerStub",
            SubscriberMode::kInline,
            &TriggerStub::HandlePose,
            &trigger,
        };
        pose_subscribers[1] = SubscriberBinding {
            "LoggerStub",
            SubscriberMode::kInline,
            &LoggerStub::HandlePose,
            &logger,
        };
        tick_subscribers[0] = SubscriberBinding {
            "TriggerStub",
            SubscriberMode::kInline,
            &TriggerStub::HandleTick,
            &trigger,
        };
        decision_trigger_subscribers[0] = SubscriberBinding {
            "AutopilotLogicStub",
            SubscriberMode::kInline,
            &AutopilotLogicStub::HandleDecisionTrigger,
            &autopilot_logic,
        };

        // Topic registration is the single source of truth for payload layout,
        // dispatch order, flags, and subscriber fan-out.
        registry[TopicIndex(TopicId::kVelocity)] = MakeTopicDescriptor(
            TopicId::kVelocity,
            "Velocity",
            static_cast<std::uint16_t>(sizeof(datVelocity)),
            static_cast<std::uint16_t>(alignof(datVelocity)),
            static_cast<std::uint16_t>(kTopicLatestWins | kTopicHotPathMirror),
            velocity_subscribers.data(),
            static_cast<std::uint8_t>(velocity_subscribers.size())
        );
        registry[TopicIndex(TopicId::kPose)] = MakeTopicDescriptor(
            TopicId::kPose,
            "Pose",
            static_cast<std::uint16_t>(sizeof(datPosition)),
            static_cast<std::uint16_t>(alignof(datPosition)),
            static_cast<std::uint16_t>(kTopicLatestWins | kTopicHotPathMirror),
            pose_subscribers.data(),
            static_cast<std::uint8_t>(pose_subscribers.size())
        );
        registry[TopicIndex(TopicId::kTick)] = MakeTopicDescriptor(
            TopicId::kTick,
            "Tick",
            static_cast<std::uint16_t>(sizeof(datTick)),
            static_cast<std::uint16_t>(alignof(datTick)),
            static_cast<std::uint16_t>(kTopicLatestWins),
            tick_subscribers.data(),
            static_cast<std::uint8_t>(tick_subscribers.size())
        );
        registry[TopicIndex(TopicId::kDecisionTrigger)] = MakeTopicDescriptor(
            TopicId::kDecisionTrigger,
            "DecisionTrigger",
            static_cast<std::uint16_t>(sizeof(datTriggerEvent)),
            static_cast<std::uint16_t>(alignof(datTriggerEvent)),
            static_cast<std::uint16_t>(kTopicLatestWins),
            decision_trigger_subscribers.data(),
            static_cast<std::uint8_t>(decision_trigger_subscribers.size())
        );
        registry[TopicIndex(TopicId::kPwmCommand)] = MakeTopicDescriptor(
            TopicId::kPwmCommand,
            "PwmCommand",
            static_cast<std::uint16_t>(sizeof(datPwmCommand)),
            static_cast<std::uint16_t>(alignof(datPwmCommand)),
            static_cast<std::uint16_t>(kTopicLatestWins),
            nullptr,
            0U
        );
        registry[TopicIndex(TopicId::kHealth)] = MakeTopicDescriptor(
            TopicId::kHealth,
            "Health",
            static_cast<std::uint16_t>(sizeof(datHealthStatus)),
            static_cast<std::uint16_t>(alignof(datHealthStatus)),
            static_cast<std::uint16_t>(kTopicLatestWins | kTopicAllowISRPublish),
            nullptr,
            0U
        );

        // Runtime buffers live beside the registry so the whole stage-1 graph is
        // self-contained and cheap to reset.
        bus.init(
            registry.data(),
            runtime_state.data(),
            storage.data(),
            static_cast<std::uint8_t>(runtime_state.size())
        );
    }

    void reset() {
        bus.clear();
        trigger.reset();
        autopilot_logic.reset();
        logger.reset();
    }

    RuntimeOutputs processTelemetryEvent(std::uint32_t timestamp_ms) {
        // The public bridge currently advances the whole graph with a single
        // telemetry event. Later this can split into several explicit entry
        // points without changing the external RuntimeGraph API.
        datIngressPacket packet;
        packet.timestamp = timestamp_ms;
        packet.pc_time_ms = timestamp_ms;
        uart_rx.publishFromISR(packet);
        uart_rx.process();

        RuntimeOutputs outputs;
        outputs.pose = logger.lastPose();
        outputs.pwm_command = autopilot_logic.lastCommand();
        return outputs;
    }

    EventBus bus {};
    std::array<TopicRuntimeState, kTopicCount> runtime_state {};
    std::array<TopicStorageBlock, kTopicCount> storage {};

    MixerFilterStub mixer_filter {};
    IntegratorStub integrator {};
    TickSourceStub tick_source {};
    TriggerStub trigger {};
    AutopilotLogicStub autopilot_logic {};
    LoggerStub logger {};
    UartRxStub uart_rx {};

    std::array<SubscriberBinding, 2> velocity_subscribers {};
    std::array<SubscriberBinding, 2> pose_subscribers {};
    std::array<SubscriberBinding, 1> tick_subscribers {};
    std::array<SubscriberBinding, 1> decision_trigger_subscribers {};
    std::array<TopicDescriptor, kTopicCount> registry {};
};

RuntimeGraph::RuntimeGraph()
    : impl_(std::make_unique<Impl>()) {
}

RuntimeGraph::~RuntimeGraph() = default;

RuntimeGraph::RuntimeGraph(RuntimeGraph&&) noexcept = default;

RuntimeGraph& RuntimeGraph::operator=(RuntimeGraph&&) noexcept = default;

void RuntimeGraph::reset() {
    impl_->reset();
}

RuntimeOutputs RuntimeGraph::processTelemetryEvent(std::uint32_t timestamp_ms) {
    return impl_->processTelemetryEvent(timestamp_ms);
}

}  // namespace telega::autopilot
