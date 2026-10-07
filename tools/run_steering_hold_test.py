#!/usr/bin/env python3
"""Run a bounded steering-angle hold while recording synchronized telemetry."""

from __future__ import annotations

import argparse
import math
import sys
import time

from control_station import ControlStation


FALL_DISARM = 1 << 20


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--offset", type=float, required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--abort-roll", type=float, default=6.0)
    args = parser.parse_args()
    if not -150.0 <= args.offset <= 150.0:
        parser.error("--offset must be between -150 and +150 us")
    if 0.0 < abs(args.offset) < 50.0:
        parser.error("--offset must be 0 or outside the +/-50 us deadband")
    if not math.isclose(args.offset / 5.0, round(args.offset / 5.0), abs_tol=1e-6):
        parser.error("--offset must use 5 us steps")

    station = ControlStation(args.port, history_seconds=max(120, int(args.duration) + 30))
    station.start()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if station.status()["uart_connected"]:
            break
        time.sleep(0.1)
    else:
        station.close()
        raise RuntimeError("UART7 did not connect")

    recording = station.start_recording()
    print(f"recording={recording['path']}", flush=True)
    started = time.monotonic()
    next_command = 0.0
    next_report = 0.0
    abort_reason = ""
    result: dict[str, object] | None = None
    try:
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now >= next_command:
                station.send_tuning("STEER", "ANGLE", f"{args.offset:.3f}", wait=False)
                next_command = now + 0.35

            status = station.status()
            balance = status.get("balance") or {}
            steering = status.get("steering") or {}
            odrive = status.get("odrive") or {}
            roll_error = float(balance.get("roll_error_deg", 0.0))
            balance_flags = int(balance.get("flags", 0))
            odrive_fault = any(
                int(odrive.get(name, 0)) != 0
                for name in ("axis_error", "motor_error", "encoder_error", "controller_error")
            )
            if abs(roll_error) >= args.abort_roll:
                abort_reason = f"roll error {roll_error:+.3f} deg exceeded limit"
                break
            if balance_flags & FALL_DISARM:
                abort_reason = "firmware FALL_DISARM asserted"
                break
            if odrive_fault:
                abort_reason = "ODrive fault detected"
                break
            if now >= next_report:
                print(
                    f"t={now-started:5.1f}s "
                    f"roll_err={roll_error:+.3f}deg "
                    f"wheel={float(balance.get('wheel_tps', 0.0)):+.2f}tps "
                    f"bias={float(balance.get('wheel_bias_deg', 0.0)):+.3f}deg "
                    f"accel={float(balance.get('accel_cmd_tps2', 0.0)):+.1f}tps2 "
                    f"Iq={float(odrive.get('iq_measured_a', 0.0)):+.1f}A "
                    f"steer={float(steering.get('commanded_offset_us', 0.0)):+.1f}us "
                    f"pulse={int(steering.get('pulse_us', 0))}us",
                    flush=True,
                )
                next_report = now + 1.0
            time.sleep(0.02)
    except KeyboardInterrupt:
        abort_reason = "operator interrupted"
    finally:
        try:
            station.send_tuning("STEER", "DISABLE", wait=False)
            time.sleep(0.1)
        finally:
            try:
                result = station.stop_recording()
            finally:
                station.close()

    if abort_reason:
        print(f"ABORT: {abort_reason}", file=sys.stderr, flush=True)
    if result is not None:
        print(f"saved={result.get('path')}", flush=True)
        print(f"analysis={result.get('analysis')}", flush=True)
    return 2 if abort_reason else 0


if __name__ == "__main__":
    raise SystemExit(main())
