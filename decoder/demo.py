#!/usr/bin/env python3
"""
demo.py — Live telemetry feed from Uncrashed FPV Drone Simulator.

Run:  python decoder/demo.py        (from the project root)
  or: cd decoder && python demo.py

Requires the TelemetryCrX Lua mod loaded in Uncrashed via UE4SS.
Press Ctrl+C to stop.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow running from either the project root or the decoder/ directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from telemetry_decoder import TelemetryDecoder  # noqa: E402


# ANSI helpers (Windows 10+ terminals support VT100)
CLEAR_LINE = "\033[2K"
BOLD       = "\033[1m"
RED        = "\033[91m"
GREEN      = "\033[92m"
YELLOW     = "\033[93m"
CYAN       = "\033[96m"
RESET_CLR  = "\033[0m"


def enable_ansi_on_windows() -> None:
    """Enable VT100 escape sequences on older Windows terminals."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        k32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
        handle = k32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_ulong()
        k32.GetConsoleMode(handle, ctypes.byref(mode))
        k32.SetConsoleMode(handle, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass


def main() -> None:
    enable_ansi_on_windows()

    decoder = TelemetryDecoder()
    print(f"{BOLD}TelemetryCrX Decoder — Demo{RESET_CLR}")
    print(f"Reading from: {CYAN}{decoder.path}{RESET_CLR}")
    print("Waiting for game data…  (start Uncrashed with the mod loaded)\n")

    prev_crashes = 0
    prev_resets  = 0
    last_print   = 0.0
    ticks        = 0

    try:
        while True:
            state = decoder.wait_for_update(timeout=2.0)

            if state is None:
                status = f"{YELLOW}no data{RESET_CLR}" if not decoder.is_live else "waiting…"
                print(f"\r{CLEAR_LINE}  [{status}]  file exists: {decoder.path.exists()}", end="", flush=True)
                continue

            ticks += 1

            # ---------- events ----------
            if state.crash_count > prev_crashes:
                n = state.crash_count - prev_crashes
                prev_crashes = state.crash_count
                print(f"\n  {RED}{BOLD}CRASH #{state.crash_count}{RESET_CLR}"
                      + (f" ({n} in burst)" if n > 1 else ""))

            if state.reset_count > prev_resets:
                prev_resets = state.reset_count
                print(f"\n  {YELLOW}{BOLD}RESET #{state.reset_count}{RESET_CLR}")

            # ---------- throttled status line (~4 Hz) ----------
            now = time.monotonic()
            if now - last_print < 0.25:
                continue
            last_print = now

            armed_str = f"{GREEN}ARMED{RESET_CLR}" if state.armed else f"{RED}DISARMED{RESET_CLR}"

            line = (
                f"  Spd {state.speed:7.1f}  |  "
                f"Pos ({state.x:8.0f},{state.y:8.0f},{state.z:8.0f})  |  "
                f"Rot (P{state.pitch:+7.1f} Y{state.yaw:+7.1f} R{state.roll:+7.1f})  |  "
                f"Gyro ({state.gyro_x:+7.1f},{state.gyro_y:+7.1f},{state.gyro_z:+7.1f})  |  "
                f"Stk T{state.throttle:+.2f} P{state.pitch_input:+.2f} "
                f"R{state.roll_input:+.2f} Y{state.yaw_input:+.2f}  |  "
                f"{armed_str}  |  "
                f"seq {state.seq}"
            )
            print(f"\r{CLEAR_LINE}{line}", end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n\n{BOLD}Stopped.{RESET_CLR}  "
              f"{ticks} ticks received, "
              f"{decoder.failed_reads} failed reads, "
              f"{decoder.stale_reads} stale reads.")


if __name__ == "__main__":
    main()
