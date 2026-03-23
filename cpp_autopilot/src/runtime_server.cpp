#include "telega_cpp_autopilot/runtime_server.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "telega_cpp_autopilot/runtime_hooks.hpp"
#include "telega_cpp_autopilot/runtime_protocol.hpp"

namespace telega::autopilot {
namespace {

bool SetRecvTimeout(int fd, int timeout_ms) {
    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    return ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
}

void CloseSocket(int& fd) {
    if (fd < 0) {
        return;
    }
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
    fd = -1;
}

bool ReadLine(int fd, const std::function<bool()>& keep_running, std::string& line) {
    line.clear();
    while (keep_running()) {
        char ch = '\0';
        const ssize_t received = ::recv(fd, &ch, 1, 0);
        if (received == 0) {
            return !line.empty();
        }
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        if (ch == '\n') {
            return true;
        }
        if (ch != '\r') {
            line.push_back(ch);
        }
    }
    return false;
}

int CreateListenSocket(const ServerConfig& config) {
    int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::perror("socket");
        return -1;
    }

    int reuse_addr = 1;
    if (::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) != 0) {
        std::perror("setsockopt");
        CloseSocket(server_fd);
        return -1;
    }

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config.port));
    if (::inet_pton(AF_INET, config.host.c_str(), &address.sin_addr) != 1) {
        std::cerr << "Invalid IPv4 address: " << config.host << '\n';
        CloseSocket(server_fd);
        return -1;
    }

    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        std::perror("bind");
        CloseSocket(server_fd);
        return -1;
    }
    if (::listen(server_fd, 1) != 0) {
        std::perror("listen");
        CloseSocket(server_fd);
        return -1;
    }
    if (!SetRecvTimeout(server_fd, config.socket_timeout_ms)) {
        std::perror("setsockopt(SO_RCVTIMEO)");
        CloseSocket(server_fd);
        return -1;
    }

    return server_fd;
}

std::string DescribeClient(const sockaddr_in& client_address) {
    char client_ip[INET_ADDRSTRLEN] = {};
    if (::inet_ntop(AF_INET, &client_address.sin_addr, client_ip, sizeof(client_ip)) == nullptr) {
        std::strncpy(client_ip, "unknown", sizeof(client_ip) - 1);
    }
    return std::string(client_ip) + ":" + std::to_string(ntohs(client_address.sin_port));
}

}  // namespace

std::optional<ServerConfig> ParseServerConfig(int argc, char** argv) {
    ServerConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            config.host = argv[++i];
            continue;
        }
        if (arg == "--port" && i + 1 < argc) {
            config.port = std::stoi(argv[++i]);
            continue;
        }
        PrintUsage(argv[0]);
        return std::nullopt;
    }

    return config;
}

void PrintUsage(const char* program_name) {
    std::cerr << "Usage: " << program_name << " [--host 127.0.0.1] [--port 8765]\n";
}

int RunServer(const ServerConfig& config, const std::function<bool()>& keep_running) {
    int server_fd = CreateListenSocket(config);
    if (server_fd < 0) {
        return 1;
    }

    std::cout << "C++ runtime stub listening on " << config.host << ':' << config.port << std::endl;

    RuntimeState state;
    bool shutdown_requested = false;

    while (keep_running() && !shutdown_requested) {
        sockaddr_in client_address {};
        socklen_t client_length = sizeof(client_address);
        int client_fd = ::accept(server_fd, reinterpret_cast<sockaddr*>(&client_address), &client_length);
        if (client_fd < 0) {
            if ((errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) && keep_running()) {
                continue;
            }
            if (!keep_running()) {
                break;
            }
            std::perror("accept");
            continue;
        }

        if (!SetRecvTimeout(client_fd, config.socket_timeout_ms)) {
            std::perror("setsockopt(SO_RCVTIMEO)");
            CloseSocket(client_fd);
            continue;
        }

        std::cout << "Client connected from " << DescribeClient(client_address) << std::endl;

        ClientConnection client(client_fd);
        bool session_closed_by_command = false;
        std::string line;

        while (keep_running() && ReadLine(client_fd, keep_running, line)) {
            if (line.empty()) {
                continue;
            }

            std::cout << "[rx] " << line << std::endl;
            const GuiMessage message = ParseGuiMessage(line);
            const GuiInterruptResult action = OnGuiMessageInterrupt(message, state, client);

            if (action.shutdown_server) {
                shutdown_requested = true;
            }
            if (action.close_session || action.shutdown_server) {
                session_closed_by_command = true;
                break;
            }
        }

        CloseSocket(client_fd);
        if (session_closed_by_command) {
            std::cout << "Client session closed by command" << std::endl;
        } else {
            std::cout << "Client disconnected" << std::endl;
        }
    }

    CloseSocket(server_fd);
    std::cout << "C++ runtime stub stopped" << std::endl;
    return 0;
}

}  // namespace telega::autopilot
