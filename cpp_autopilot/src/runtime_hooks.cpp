#include "telega_cpp_autopilot/runtime_hooks.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <sys/socket.h>
#include <unistd.h>

#include "telega_cpp_autopilot/runtime_graph.hpp"

namespace telega::autopilot {
namespace {

bool SendAll(int socket_fd, const std::string& payload) {
    const char* data = payload.c_str();
    std::size_t total = 0;
    while (total < payload.size()) {
        const ssize_t sent = ::send(socket_fd, data + total, payload.size() - total, 0);
        if (sent <= 0) {
            return false;
        }
        total += static_cast<std::size_t>(sent);
    }
    return true;
}

std::string JsonQuoted(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return "\"" + escaped + "\"";
}

void EnsureRuntimeGraph(RuntimeState& state) {
    if (state.graph == nullptr) {
        // The bridge keeps lazy construction so startup stays cheap and a custom
        // replacement graph can later be injected without changing message flow.
        state.graph = std::make_unique<RuntimeGraph>();
    }
}

}  // namespace

RuntimeState::RuntimeState() = default;

RuntimeState::~RuntimeState() = default;

RuntimeState::RuntimeState(RuntimeState&&) noexcept = default;

RuntimeState& RuntimeState::operator=(RuntimeState&&) noexcept = default;

ClientConnection::ClientConnection(int socket_fd)
    : socket_fd_(socket_fd) {
}

bool ClientConnection::SendJsonLine(const std::string& json_line) const {
    if (socket_fd_ < 0) {
        return false;
    }
    return SendAll(socket_fd_, json_line + "\n");
}

int ClientConnection::socket_fd() const {
    return socket_fd_;
}

bool SendPoseUpdate(const ClientConnection& client, const PoseUpdate& pose) {
    std::ostringstream payload;
    payload
        << "{\"type\":\"pose_update\",\"pose\":{\"x_m\":" << pose.x_m
        << ",\"y_m\":" << pose.y_m
        << ",\"theta_rad\":" << pose.theta_rad
        << ",\"source\":" << JsonQuoted(pose.source)
        << ",\"pc_time_ms\":" << pose.pc_time_ms
        << ",\"mcu_time_ms\":";
    if (pose.mcu_time_ms.has_value()) {
        payload << *pose.mcu_time_ms;
    } else {
        payload << "null";
    }
    payload << "},\"source\":" << JsonQuoted(pose.logger_source) << "}";

    if (!client.SendJsonLine(payload.str())) {
        return false;
    }

    std::cout << "[tx] " << payload.str() << std::endl;
    return true;
}

bool SendPwmCommand(const ClientConnection& client, const PwmCommand& command) {
    std::ostringstream payload;
    payload
        << "{\"type\":\"control_command\",\"left_pwm\":" << command.left_pwm
        << ",\"right_pwm\":" << command.right_pwm
        << ",\"duration_ms\":" << command.duration_ms
        << ",\"finished\":" << (command.finished ? "true" : "false")
        << ",\"source\":" << JsonQuoted(command.source) << "}";

    if (!client.SendJsonLine(payload.str())) {
        return false;
    }

    std::cout << "[tx] " << payload.str() << std::endl;
    return true;
}

GuiInterruptResult DefaultOnGuiMessageInterrupt(
    const GuiMessage& message,
    RuntimeState& state,
    const ClientConnection& client
) {
    GuiInterruptResult result;

    switch (message.kind) {
        case GuiMessageKind::kHello:
            std::cout << "[state] hello received" << std::endl;
            break;

        case GuiMessageKind::kMission:
            state.stored_mission_json = message.raw_json;
            state.mission_loaded = true;
            EnsureRuntimeGraph(state);
            // Reset-on-mission-load keeps the stage-1 graph deterministic until
            // real mission parsing/configuration is introduced.
            state.graph->reset();
            std::cout << "[state] mission stored, bytes=" << state.stored_mission_json.size() << std::endl;
            break;

        case GuiMessageKind::kReset:
            EnsureRuntimeGraph(state);
            state.graph->reset();
            std::cout << "[state] runtime reset" << std::endl;
            break;

        case GuiMessageKind::kTelemetryEvent:
        case GuiMessageKind::kControlTick: {
            EnsureRuntimeGraph(state);
            const std::uint32_t timestamp_ms = message.pc_time_ms.has_value()
                ? static_cast<std::uint32_t>(std::lround(*message.pc_time_ms))
                : 0U;
            // For now both telemetry and control-tick messages advance the same
            // graph step. The distinction remains available at protocol level.
            const RuntimeOutputs outputs = state.graph->processTelemetryEvent(timestamp_ms);

            if (outputs.pose.has_value()) {
                const datPosition& position = *outputs.pose;
                PoseUpdate pose;
                pose.x_m = position.X.A;
                pose.y_m = position.Y.A;
                pose.theta_rad = position.Theta.A;
                pose.pc_time_ms = static_cast<int>(position.timestamp);
                if (!SendPoseUpdate(client, pose)) {
                    result.close_session = true;
                    break;
                }
            }

            if (outputs.pwm_command.has_value()) {
                const datPwmCommand& stub_command = *outputs.pwm_command;
                PwmCommand command;
                command.left_pwm = stub_command.left_pwm;
                command.right_pwm = stub_command.right_pwm;
                command.duration_ms = message.duration_ms;
                if (!SendPwmCommand(client, command)) {
                    result.close_session = true;
                }
            }
            break;
        }

        case GuiMessageKind::kStop:
            std::cout << "[state] stop received, mission_loaded="
                      << (state.mission_loaded ? "true" : "false")
                      << ", mission_bytes=" << state.stored_mission_json.size() << std::endl;
            state.stored_mission_json.clear();
            state.mission_loaded = false;
            if (state.graph != nullptr) {
                state.graph->reset();
            }
            result.close_session = true;
            break;

        case GuiMessageKind::kShutdown:
            std::cout << "[state] shutdown received, closing server, mission_loaded="
                      << (state.mission_loaded ? "true" : "false")
                      << ", mission_bytes=" << state.stored_mission_json.size() << std::endl;
            state.stored_mission_json.clear();
            state.mission_loaded = false;
            if (state.graph != nullptr) {
                state.graph->reset();
            }
            result.close_session = true;
            result.shutdown_server = true;
            break;

        case GuiMessageKind::kUnknown:
            std::cout << "[state] unsupported message type=" << JsonQuoted(message.type_name) << std::endl;
            break;
    }

    return result;
}

GuiInterruptResult OnGuiMessageInterrupt(
    const GuiMessage& message,
    RuntimeState& state,
    const ClientConnection& client
) {
    return DefaultOnGuiMessageInterrupt(message, state, client);
}

}  // namespace telega::autopilot
