#pragma once

#include <functional>
#include <optional>
#include <string>

namespace telega::autopilot {

struct ServerConfig {
    std::string host = "127.0.0.1";
    int port = 8765;
    int socket_timeout_ms = 250;
};

std::optional<ServerConfig> ParseServerConfig(int argc, char** argv);
void PrintUsage(const char* program_name);

int RunServer(const ServerConfig& config, const std::function<bool()>& keep_running);

}  // namespace telega::autopilot
