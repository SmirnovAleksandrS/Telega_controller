#include <csignal>

#include "telega_cpp_autopilot/runtime_server.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;

void HandleSignal(int) {
    g_running = 0;
}

bool KeepRunning() {
    return g_running != 0;
}

}  // namespace

int main(int argc, char** argv) {
    const auto config = telega::autopilot::ParseServerConfig(argc, argv);
    if (!config.has_value()) {
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    return telega::autopilot::RunServer(*config, KeepRunning);
}
