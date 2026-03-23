#pragma once

#include <optional>
#include <string>

#include "telega_cpp_autopilot/runtime_protocol.hpp"

namespace telega::autopilot {

struct RuntimeState {
    std::string stored_mission_json;
    bool mission_loaded = false;
};

struct PoseUpdate {
    double x_m = 0.0;
    double y_m = 0.0;
    double theta_rad = 0.0;
    std::string source = "external";
    int pc_time_ms = 0;
    std::optional<int> mcu_time_ms;
    std::string logger_source = "cpp_stub_logger";
};

struct PwmCommand {
    int left_pwm = 1500;
    int right_pwm = 1500;
    int duration_ms = 100;
    bool finished = false;
    std::string source = "cpp_stub_controller";
};

class ClientConnection {
public:
    explicit ClientConnection(int socket_fd);

    bool SendJsonLine(const std::string& json_line) const;
    int socket_fd() const;

private:
    int socket_fd_ = -1;
};

struct GuiInterruptResult {
    bool close_session = false;
    bool shutdown_server = false;
};

#if defined(__GNUC__) || defined(__clang__)
#define TELEGA_RUNTIME_WEAK __attribute__((weak))
#else
#define TELEGA_RUNTIME_WEAK
#endif

bool SendPoseUpdate(const ClientConnection& client, const PoseUpdate& pose);
bool SendPwmCommand(const ClientConnection& client, const PwmCommand& command);

// Default stub behavior used by the current project.
GuiInterruptResult DefaultOnGuiMessageInterrupt(
    const GuiMessage& message,
    RuntimeState& state,
    const ClientConnection& client
);

// Interrupt-like hook for every inbound GUI message.
// You can provide a strong definition with the same signature in another
// translation unit and the linker will use it instead of the weak default.
GuiInterruptResult OnGuiMessageInterrupt(
    const GuiMessage& message,
    RuntimeState& state,
    const ClientConnection& client
) TELEGA_RUNTIME_WEAK;

}  // namespace telega::autopilot
