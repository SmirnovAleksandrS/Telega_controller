"""
Backward-compatible alias for the unified socket runtime bridge.
"""

from runtime.socket_runtime import SocketExternalRuntime

SocketAutopilotClient = SocketExternalRuntime

__all__ = ["SocketAutopilotClient", "SocketExternalRuntime"]
