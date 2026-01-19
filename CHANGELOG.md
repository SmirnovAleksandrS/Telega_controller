# Changelog — Virtual Controller

Framework/UI: Tkinter (stdlib)
UART: pyserial
Log format: .log (JSON Lines)

## Implemented
- UART stream parser: SOF 0x7E, type, len, payload, CRC32 (IEEE, over type+len+payload).
- Strong parsing of D0/D1/D2 and F0.
- Outgoing builders for B0 (sync req), C0 (control), A0/A1 (admin).
- NTP-like time sync on PC:
  - initial multi-round sampling, K-best by minimal delta, estimate a,b
  - periodic re-sync + EMA smoothing
  - wrap-safe u32(ms) arithmetic for timestamps
- GUI:
  - Tabs: Manual (implemented), Coordinate (placeholder)
  - Virtual joystick on Canvas (mouse drag)
  - Left/Right output values, per-side Shift and Linear coefficients
  - Right panel: Critical Parameters table, MCU Time, Radio Quality, Deviation Settings
  - Menu: Files (Log path / Start / Stop), Settings (COM Settings)
  - Log Running (green) / Log Stopped (red)
- Logging:
  - saves parsed messages to .log as JSONL, includes PC receive time and time model snapshot

## Unreleased
- Fix crash in right panel init by initializing value label registry before creating rows.
- Update manual joystick math to center at 1500 with range 1000..2000, applying shift add and linear multiplier.
- Add bottom Log/RPM tabs, move joystick to make room, and stream parsed logs to UI + console.
- Redraw RPM graph on a timer using joystick commands even without MCU data.
- Add log filter dropdown with checkboxes grouped by message type and hover tooltips; filtering affects UI only.
- Keep RPM graph values steady between new data points by reusing last samples.
- Add Time Sync button to trigger initial synchronization procedure.
- Log outgoing PC messages (B0/C0) to file/console/GUI with dark-blue color.
- Log all outgoing frames in decoded, human-readable form (no raw hex).
- Avoid dumping raw payload bytes for unknown frames; log type and length only.
- Show both received MCU timestamp and estimated MCU time in the GUI.
- Display MCU times on separate lines and log rx_time alongside parsed payload timestamps.
- Add TX status counters and last write info in GUI to confirm serial writes.
- Remove TX status from GUI after debugging.
- Persist COM settings, deviation config, and joystick shifts/linear; add quick Connect/Disconnect button.

## Notes/Ideas
- F0 payload in PDF lacks seq; implemented best-effort matching to last pending request.
- Radio Quality implemented as recent RX success ratio (0..10).
- If needed: implement A0/A1 UI to enable/disable periodic D messages.
