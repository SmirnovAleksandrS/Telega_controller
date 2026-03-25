# C++ runtime stub

First-stage standalone bridge for the future external business-logic runtime.

The binary runs as a separate process, accepts TCP connections from the GUI, prints every received JSON message, stores the incoming mission, and answers each `telemetry_event` with:

- a zero pose via `pose_update`;
- a neutral PWM command via `control_command`.

## Build

```bash
cmake -S cpp_autopilot -B cpp_autopilot/build
cmake --build cpp_autopilot/build
```

## Run in a separate terminal

```bash
./cpp_autopilot/build/telega_cpp_autopilot
```

Optional endpoint override:

```bash
./cpp_autopilot/build/telega_cpp_autopilot --host 127.0.0.1 --port 8765
```

The GUI uses the same default endpoint. If needed, it can be redirected through environment variables:

```bash
export TELEGA_CPP_RUNTIME_HOST=127.0.0.1
export TELEGA_CPP_RUNTIME_PORT=8765
python main.py
```

Backward-compatible environment variables are still accepted:

```bash
export TELEGA_CPP_AUTOPILOT_HOST=127.0.0.1
export TELEGA_CPP_AUTOPILOT_PORT=8765
```

## Current protocol

Transport: newline-delimited JSON over TCP.

GUI -> C++:

- `hello`: protocol/version handshake.
- `mission`: full `MissionConfig`.
- `reset`: reset controller state.
- `telemetry_event`: current `TelemetrySnapshot` triggered by a speed sample.
- `stop`: terminate the active session and close the current client connection.
- `shutdown`: stop the stub process and release the listening port.

The stub also accepts `kill`, `terminate`, and `exit` as aliases for `shutdown`.

C++ -> GUI:

- `pose_update`: currently returns zero pose.
- `control_command`: currently returns `left_pwm = 1500` and `right_pwm = 1500`.

Example command reply:

```json
{"type":"control_command","left_pwm":1500,"right_pwm":1500,"duration_ms":100,"finished":false,"source":"cpp_stub_controller"}
```

Example pose reply:

```json
{"type":"pose_update","pose":{"x_m":0.0,"y_m":0.0,"theta_rad":0.0,"source":"external","pc_time_ms":0,"mcu_time_ms":null},"source":"cpp_stub_logger"}
```

At this stage the stub is intentionally simple: it validates mission loading, the telemetry-triggered processing seam, and the async output flow back into the GUI.

Shutdown behavior is split intentionally:

- mission stop from the GUI uses `stop`, which closes only the current TCP session so the next mission can reconnect;
- full GUI shutdown uses `shutdown`, which terminates the stub process and frees the port for the next launch.

## Source layout

- `src/main.cpp`: only signal handling and `RunServer(...)`.
- `src/event_bus.cpp`: stage-1 event bus core with `bitMask`, double-buffer topic storage, and inline dispatch.
- `src/runtime_graph.cpp`: graph assembly, topic registry, and topology wiring.
- `src/runtime_nodes/*.cpp`: one stub node per file (`MixerFilter`, `Integrator`, `Trigger`, `Logger`, and so on).
- `src/runtime_server.cpp`: TCP server lifecycle, accept loop, start/stop.
- `src/runtime_protocol.cpp`: parsing inbound GUI messages into typed `GuiMessage`.
- `src/runtime_hooks.cpp`: default runtime logic and outbound helpers.
- `include/telega_cpp_autopilot/runtime_hooks.hpp`: the main extension seam.
- `include/telega_cpp_autopilot/event_bus.hpp`: bus API, topic descriptors, subscriber bindings, and dispatch state.
- `include/telega_cpp_autopilot/event_payloads.hpp`: stage-1 payload types for bus topics.
- `include/telega_cpp_autopilot/runtime_graph.hpp`: public stub graph entry point used by the runtime hook.
- `include/telega_cpp_autopilot/runtime_node_interfaces.hpp`: small direct-call interfaces for hot-path node chaining.
- `include/telega_cpp_autopilot/runtime_nodes/*.hpp`: per-node contracts and stub implementations.

## Main hook and send functions

The interrupt-like receive hook is:

```cpp
telega::autopilot::GuiInterruptResult OnGuiMessageInterrupt(
    const telega::autopilot::GuiMessage& message,
    telega::autopilot::RuntimeState& state,
    const telega::autopilot::ClientConnection& client
);
```

By default it is implemented in `src/runtime_hooks.cpp` as a weak symbol. You can define the same function in another `.cpp` file and the linker will use your version instead of the stub one.

Outbound helpers are:

```cpp
bool SendPoseUpdate(const ClientConnection& client, const PoseUpdate& pose);
bool SendPwmCommand(const ClientConnection& client, const PwmCommand& command);
```

At the moment they cover the two GUI-facing outputs you asked for:

- current trolley pose via `pose_update`;
- left/right PWM command via `control_command`.
