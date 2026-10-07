#!/usr/bin/env python3
"""Repeat a symmetric steering sequence with arrival-based dwell timing and safety aborts."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from control_station import ControlStation


STEER_MODE_ANGLE = 3
FALL_DISARM = 1 << 20
ODRIVE_TIMEOUT = 1 << 15
ODRIVE_FAULT = 1 << 16


def finite(value: object, fallback: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--amplitude", type=float, required=True)
    parser.add_argument("--groups", type=int, default=10)
    parser.add_argument("--dwell", type=float, default=10.0)
    parser.add_argument("--abort-roll", type=float, default=4.0)
    parser.add_argument("--abort-wheel", type=float, default=30.0)
    args = parser.parse_args()
    if not 0.0 < args.amplitude <= 150.0:
        parser.error("--amplitude must be in (0, 150] us")
    if args.amplitude < 50.0:
        parser.error("--amplitude must be outside the 50 us deadband")
    if not math.isclose(args.amplitude / 5.0, round(args.amplitude / 5.0), abs_tol=1e-6):
        parser.error("--amplitude must use 5 us steps")
    if args.groups <= 0 or args.dwell <= 0.0:
        parser.error("--groups and --dwell must be positive")

    station = ControlStation(args.port, history_seconds=3600)
    station.start()
    startup_deadline = time.monotonic() + 10.0
    while time.monotonic() < startup_deadline:
        status = station.status()
        if status["uart_connected"] and status.get("balance") and status.get("steering"):
            break
        time.sleep(0.1)
    else:
        station.close()
        raise RuntimeError("UART7 telemetry did not become ready")

    recording = station.start_recording()
    metadata_path = Path(str(recording["path"])).with_suffix(".stages.json")
    print(f"recording={recording['path']}", flush=True)
    stages: list[dict[str, object]] = []
    aborted = ""
    test_started = time.monotonic()

    def check_safety(status: dict[str, object]) -> tuple[float, float]:
        nonlocal aborted
        balance = status.get("balance") or {}
        roll_error = finite(balance.get("roll_error_deg"))
        wheel = finite(balance.get("wheel_tps"))
        flags = int(balance.get("flags", 0))
        uart_age_ms = status.get("uart_age_ms")
        if uart_age_ms is None or finite(uart_age_ms, 1e9) > 500.0:
            aborted = f"UART telemetry stale ({uart_age_ms} ms)"
        elif abs(roll_error) >= args.abort_roll:
            aborted = f"roll error {roll_error:+.3f} deg exceeded limit"
        elif abs(wheel) >= args.abort_wheel:
            aborted = f"wheel speed {wheel:+.3f} tps exceeded limit"
        elif flags & FALL_DISARM:
            aborted = "firmware FALL_DISARM asserted"
        elif flags & ODRIVE_TIMEOUT:
            aborted = "firmware ODRIVE_TIMEOUT asserted"
        elif flags & ODRIVE_FAULT:
            aborted = "firmware ODRIVE_FAULT asserted"
        return roll_error, wheel

    def run_stage(group: int, name: str, target: float) -> bool:
        nonlocal aborted
        stage_started = time.monotonic()
        reached_at: float | None = None
        stable_since: float | None = None
        next_command = 0.0
        next_report = 0.0
        peak_roll = 0.0
        peak_wheel = 0.0
        peak_accel = 0.0
        sat_samples = 0
        samples = 0
        while not aborted:
            now = time.monotonic()
            if now - stage_started > 35.0:
                aborted = f"G{group} {name}: target {target:+.0f} us arrival timeout"
                break
            if now >= next_command:
                station.send_tuning("STEER", "ANGLE", f"{target:.3f}", wait=False)
                next_command = now + 0.35
            status = station.status()
            balance = status.get("balance") or {}
            steering = status.get("steering") or {}
            roll_error, wheel = check_safety(status)
            accel = finite(balance.get("accel_cmd_tps2"))
            flags = int(balance.get("flags", 0))
            peak_roll = max(peak_roll, abs(roll_error))
            peak_wheel = max(peak_wheel, abs(wheel))
            peak_accel = max(peak_accel, abs(accel))
            sat_samples += int(bool(flags & ((1 << 3) | (1 << 4) | (1 << 19))))
            samples += 1

            mode = int(steering.get("mode", -1))
            actual = finite(steering.get("commanded_offset_us"), 9999.0)
            at_target = mode == STEER_MODE_ANGLE and abs(actual - target) <= 0.6
            if at_target:
                if stable_since is None:
                    stable_since = now
                elif reached_at is None and now - stable_since >= 0.5:
                    reached_at = now
                    print(
                        f"G{group:02d} {name:<18} reached target={target:+.0f}us "
                        f"move={reached_at-stage_started:.2f}s; dwell {args.dwell:.1f}s",
                        flush=True,
                    )
            else:
                stable_since = None
            if reached_at is not None and now - reached_at >= args.dwell:
                ended = time.monotonic()
                stages.append(
                    {
                        "group": group,
                        "name": name,
                        "target_us": target,
                        "start_s": stage_started - test_started,
                        "reached_s": reached_at - test_started,
                        "end_s": ended - test_started,
                        "move_s": reached_at - stage_started,
                        "dwell_s": ended - reached_at,
                        "peak_abs_roll_deg": peak_roll,
                        "peak_abs_wheel_tps": peak_wheel,
                        "peak_abs_accel_tps2": peak_accel,
                        "saturation_pct": 100.0 * sat_samples / max(samples, 1),
                    }
                )
                print(
                    f"G{group:02d} {name:<18} done "
                    f"roll_peak={peak_roll:.3f}deg wheel_peak={peak_wheel:.2f}tps "
                    f"accel_peak={peak_accel:.1f}tps2 sat={100.0*sat_samples/max(samples,1):.1f}%",
                    flush=True,
                )
                return True
            if now >= next_report:
                print(
                    f"  live G{group:02d}/{args.groups} {name}: offset={actual:+.1f}us "
                    f"roll={roll_error:+.3f}deg wheel={wheel:+.2f}tps accel={accel:+.1f}tps2",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        return False

    sequence = (
        ("left", args.amplitude),
        ("center_after_left", 0.0),
        ("right", -args.amplitude),
        ("center_after_right", 0.0),
        ("left_again", args.amplitude),
        ("direct_left_to_right", -args.amplitude),
        ("final_center", 0.0),
    )

    result: dict[str, object] | None = None
    try:
        for group in range(1, args.groups + 1):
            print(f"=== GROUP {group}/{args.groups}, amplitude ±{args.amplitude:.0f} us ===", flush=True)
            for name, target in sequence:
                if not run_stage(group, name, target):
                    break
            if aborted:
                break
    except KeyboardInterrupt:
        aborted = "operator interrupted"
    finally:
        try:
            station.send_tuning("STEER", "DISABLE", wait=False)
            time.sleep(0.2)
        finally:
            try:
                result = station.stop_recording()
            finally:
                station.close()
        metadata_path.write_text(
            json.dumps(
                {
                    "amplitude_us": args.amplitude,
                    "requested_groups": args.groups,
                    "completed_stages": stages,
                    "aborted": aborted or None,
                    "csv_path": result.get("path") if result else None,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"stages={metadata_path}", flush=True)
        if result is not None:
            print(f"saved={result.get('path')}", flush=True)
    if aborted:
        print(f"ABORT: {aborted}", flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
