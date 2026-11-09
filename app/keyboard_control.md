> SRC: keyboard_control.hpp, keyboard_input.hpp, keyboard_input.cpp, keyboard_desktop.hpp, keyboard_desktop.cpp, keyboard_channel.hpp, app_peripherals_keyboard_bridge.cpp
> FLOW: Local X11 held keys / terminal steps → control worker → ServoRelativeP / ServoJ → cabinet planner

# Keyboard teleoperation

- `DesktopKeys`: local X11 via bounded XCB queries, no window/grab/evdev;
  terminal Space binds focus, physical releases and toggle latches suppress
  autorepeat. Focus/tab/modifier loss pauses; SSH/Wayland fall back to steps.
- `Channel`: short snapshot mutex, sticky focus-loss and 150ms input freshness.
  Worker owns Session at 1kHz; terminal I/O and XCB remain on main thread.
- `UpdateHeld`: rate-limited continuous directions from held-key snapshots,
  simultaneous arms, opposing keys cancel, Cartesian diagonal norm bounded.
- `Input`: explicit `--terminal` fallback, noncanonical stdin with ISIG.
  RAII restores terminal; startup input discarded. Parses CSI/SS3 arrows,
  PageUp/Down, fragmented sequences and Esc. Paste/focus-out/flood pauses.
- `Control::Update`: Space toggle, paused startup; one pending step/axis,
  1mm or 0.5deg, rate-limited at 1kHz without catch-up after a loop stall.
  Terminal repeat is not a key-release or simultaneous-held-key signal.
- `TrySwitch`: Tab pauses both arms, clears increments; fresh stable position/
  velocity observations for 100ms switch Cartesian/Joint, remaining paused.
- `Check`: local feedback age ≤250ms, SDK ownership/Kinematics and fixed frames.
- `Session`: Single or Left/Right Robot instances; both checked before Enable.
  Switch seeds measured joints; ServoJ targets clamp to cabinet limits.
  Shutdown holds with the active command type, then independently disables and
  checks enabled=false for both arms; no Move-only StopMotion on Servo streams.
  Partial startup/send/disable failures still clean up both arms.
- CLI: Linux SDK build optionally uses libxcb; current-terminal local held keys or
  bounded `--terminal`, dry-run or explicit motion/local/IP. Worker joined
  through Servo hold/disable on SIGINT, Esc or input failure.
