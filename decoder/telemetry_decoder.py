"""
TelemetryCrX Decoder
====================
Pure-Python telemetry reader for the Uncrashed FPV Drone Simulator.

Reads the JSON state file written by the TelemetryCrX Lua mod (via UE4SS)
and exposes it as an immutable DroneState dataclass.

Transport
---------
The Lua mod writes ``%TEMP%\\TelemetryCrX\\state.json`` at ~40 Hz.
This module polls that file.  Partial / corrupt reads are handled
gracefully (the last good state is returned).

Usage
-----
    from decoder import TelemetryDecoder

    dec = TelemetryDecoder()
    while True:
        state = dec.wait_for_update()
        if state:
            print(state.speed, state.x, state.y, state.z)
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class DroneState:
    """Immutable snapshot of drone telemetry at one instant."""

    # Sequence / timing
    seq: int = 0
    timestamp: float = 0.0          # os.clock() on the game side

    # Dynamics
    speed: float = 0.0              # magnitude of velocity (uu/s)
    x: float = 0.0                  # world position  (Unreal units)
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0                 # velocity components (uu/s)
    vy: float = 0.0
    vz: float = 0.0

    # Orientation  (degrees, Unreal conventions)
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    # Angular rates (deg/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    gyro_source: str = "none"       # "derived" | "physics" | "none"

    # Event counters (monotonically increasing per session)
    crash_count: int = 0
    reset_count: int = 0

    # Per-tick event flags (true for the single tick when it happened)
    crash_event: bool = False
    reset_event: bool = False

    # Game state
    armed: bool = False
    is_3d: bool = False

    # Stick inputs  (raw axis values from the game)
    throttle: float = 0.0
    pitch_input: float = 0.0
    roll_input: float = 0.0
    yaw_input: float = 0.0


# ---------------------------------------------------------------------------
# Decoder
# ---------------------------------------------------------------------------

class TelemetryDecoder:
    """
    Reads and decodes drone telemetry from the JSON state file
    written by the TelemetryCrX Lua mod.

    Parameters
    ----------
    path : str or Path, optional
        Override the telemetry file location.
        Defaults to ``%TEMP%\\TelemetryCrX\\state.json``.
    """

    DEFAULT_PATH = os.path.join(
        os.environ.get("TEMP",
                       os.path.join(os.path.expanduser("~"),
                                    "AppData", "Local", "Temp")),
        "TelemetryCrX", "state.json",
    )

    def __init__(self, path: Optional[str | Path] = None) -> None:
        self.path = Path(path or self.DEFAULT_PATH)
        self._last_seq: int = -1
        self._last_state: Optional[DroneState] = None

        # Diagnostics
        self.total_reads: int = 0
        self.failed_reads: int = 0
        self.stale_reads: int = 0

    # ---- public API -------------------------------------------------------

    def read_state(self) -> Optional[DroneState]:
        """
        Non-blocking read of the current drone state.

        * Returns a *new* ``DroneState`` whenever the Lua side has written
          a fresh tick (detected via the ``seq`` counter).
        * Returns the **previous** state on a stale or failed read.
        * Returns ``None`` only if no valid state has ever been read.
        """
        try:
            raw = self.path.read_text(encoding="utf-8")
            data: dict = json.loads(raw)
        except FileNotFoundError:
            return self._last_state
        except (PermissionError, json.JSONDecodeError, UnicodeDecodeError):
            self.failed_reads += 1
            return self._last_state

        self.total_reads += 1
        seq = data.get("seq", 0)

        if seq == self._last_seq:
            self.stale_reads += 1
            return self._last_state

        self._last_seq = seq
        state = self._parse(data)
        self._last_state = state
        return state

    def wait_for_update(self, timeout: float = 1.0,
                        poll_interval: float = 0.005) -> Optional[DroneState]:
        """
        Block until a new state tick arrives (``seq`` changes).

        Parameters
        ----------
        timeout : float
            Max seconds to wait.
        poll_interval : float
            Seconds between file reads (default 5 ms).

        Returns the new ``DroneState``, or ``None`` on timeout.
        """
        start_seq = self._last_seq
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.read_state()
            if state is not None and state.seq != start_seq:
                return state
            time.sleep(poll_interval)
        return None

    @property
    def is_live(self) -> bool:
        """``True`` if the telemetry file was modified in the last second."""
        try:
            return time.time() - self.path.stat().st_mtime < 1.0
        except FileNotFoundError:
            return False

    @property
    def last_state(self) -> Optional[DroneState]:
        """The most recent successfully parsed state (or ``None``)."""
        return self._last_state

    # ---- internals --------------------------------------------------------

    @staticmethod
    def _parse(d: dict) -> DroneState:
        return DroneState(
            seq=int(d.get("seq", 0)),
            timestamp=float(d.get("t", 0.0)),
            speed=float(d.get("speed", 0.0)),
            x=float(d.get("x", 0.0)),
            y=float(d.get("y", 0.0)),
            z=float(d.get("z", 0.0)),
            vx=float(d.get("vx", 0.0)),
            vy=float(d.get("vy", 0.0)),
            vz=float(d.get("vz", 0.0)),
            pitch=float(d.get("pitch", 0.0)),
            yaw=float(d.get("yaw", 0.0)),
            roll=float(d.get("roll", 0.0)),
            gyro_x=float(d.get("gyro_x", 0.0)),
            gyro_y=float(d.get("gyro_y", 0.0)),
            gyro_z=float(d.get("gyro_z", 0.0)),
            gyro_source=str(d.get("gyro_src", "none")),
            crash_count=int(d.get("crashes", 0)),
            reset_count=int(d.get("resets", 0)),
            crash_event=bool(d.get("crash_ev", False)),
            reset_event=bool(d.get("reset_ev", False)),
            armed=bool(d.get("armed", False)),
            is_3d=bool(d.get("is_3d", False)),
            throttle=float(d.get("throttle", 0.0)),
            pitch_input=float(d.get("pitch_in", 0.0)),
            roll_input=float(d.get("roll_in", 0.0)),
            yaw_input=float(d.get("yaw_in", 0.0)),
        )
