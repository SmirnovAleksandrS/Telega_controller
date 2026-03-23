#pragma once

#include <optional>
#include <string>

namespace telega::autopilot {

enum class GuiMessageKind {
    kHello,
    kMission,
    kReset,
    kTelemetryEvent,
    kControlTick,
    kStop,
    kShutdown,
    kUnknown,
};

struct GuiMessage {
    GuiMessageKind kind = GuiMessageKind::kUnknown;
    std::string type_name;
    std::string raw_json;
    std::optional<double> pc_time_ms;
    int duration_ms = 100;
};

GuiMessage ParseGuiMessage(const std::string& json_line);
const char* ToString(GuiMessageKind kind);

}  // namespace telega::autopilot
