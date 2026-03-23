#include "telega_cpp_autopilot/runtime_protocol.hpp"

#include <cctype>
#include <cmath>
#include <optional>

namespace telega::autopilot {
namespace {

std::optional<std::string> ExtractJsonStringField(const std::string& json, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const std::size_t key_pos = json.find(needle);
    if (key_pos == std::string::npos) {
        return std::nullopt;
    }

    std::size_t pos = json.find(':', key_pos + needle.size());
    if (pos == std::string::npos) {
        return std::nullopt;
    }
    ++pos;

    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    if (pos >= json.size() || json[pos] != '"') {
        return std::nullopt;
    }
    ++pos;

    std::string value;
    for (; pos < json.size(); ++pos) {
        const char ch = json[pos];
        if (ch == '\\') {
            if (pos + 1 < json.size()) {
                value.push_back(json[pos + 1]);
                ++pos;
            }
            continue;
        }
        if (ch == '"') {
            return value;
        }
        value.push_back(ch);
    }
    return std::nullopt;
}

std::optional<double> ExtractJsonNumberField(const std::string& json, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const std::size_t key_pos = json.find(needle);
    if (key_pos == std::string::npos) {
        return std::nullopt;
    }

    std::size_t pos = json.find(':', key_pos + needle.size());
    if (pos == std::string::npos) {
        return std::nullopt;
    }
    ++pos;

    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }

    std::size_t end = pos;
    while (end < json.size()) {
        const char ch = json[end];
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') {
            ++end;
            continue;
        }
        break;
    }
    if (end == pos) {
        return std::nullopt;
    }

    try {
        return std::stod(json.substr(pos, end - pos));
    } catch (...) {
        return std::nullopt;
    }
}

int DurationMsFromTick(const std::string& json) {
    const auto dt_s = ExtractJsonNumberField(json, "dt_s");
    if (!dt_s.has_value()) {
        return 100;
    }
    return std::max(1, static_cast<int>(std::lround(*dt_s * 1000.0)));
}

GuiMessageKind ParseMessageKind(const std::string& type_name) {
    if (type_name == "hello") {
        return GuiMessageKind::kHello;
    }
    if (type_name == "mission") {
        return GuiMessageKind::kMission;
    }
    if (type_name == "reset") {
        return GuiMessageKind::kReset;
    }
    if (type_name == "telemetry_event") {
        return GuiMessageKind::kTelemetryEvent;
    }
    if (type_name == "control_tick") {
        return GuiMessageKind::kControlTick;
    }
    if (type_name == "stop") {
        return GuiMessageKind::kStop;
    }
    if (type_name == "shutdown" || type_name == "kill" || type_name == "terminate" || type_name == "exit") {
        return GuiMessageKind::kShutdown;
    }
    return GuiMessageKind::kUnknown;
}

}  // namespace

GuiMessage ParseGuiMessage(const std::string& json_line) {
    GuiMessage message;
    message.raw_json = json_line;
    message.type_name = ExtractJsonStringField(json_line, "type").value_or("");
    message.kind = ParseMessageKind(message.type_name);
    message.pc_time_ms = ExtractJsonNumberField(json_line, "pc_time_ms");
    if (message.kind == GuiMessageKind::kControlTick) {
        message.duration_ms = DurationMsFromTick(json_line);
    }
    return message;
}

const char* ToString(GuiMessageKind kind) {
    switch (kind) {
        case GuiMessageKind::kHello:
            return "hello";
        case GuiMessageKind::kMission:
            return "mission";
        case GuiMessageKind::kReset:
            return "reset";
        case GuiMessageKind::kTelemetryEvent:
            return "telemetry_event";
        case GuiMessageKind::kControlTick:
            return "control_tick";
        case GuiMessageKind::kStop:
            return "stop";
        case GuiMessageKind::kShutdown:
            return "shutdown";
        case GuiMessageKind::kUnknown:
            return "unknown";
    }
    return "unknown";
}

}  // namespace telega::autopilot
