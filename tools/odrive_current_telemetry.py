#!/usr/bin/env python3
"""Record ODrive Axis 0 current and velocity feedback over USB."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import odrive


FIELDS = (
    "host_monotonic_s",
    "vbus_v",
    "ibus_a",
    "wheel_tps",
    "input_vel_tps",
    "vel_setpoint_tps",
    "iq_setpoint_a",
    "iq_measured_a",
    "axis_state",
    "axis_error",
    "motor_error",
    "encoder_error",
    "controller_error",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("duration and rate must be positive")

    device = odrive.find_any(timeout=5)
    if device is None:
        raise RuntimeError("ODrive not found")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    period_s = 1.0 / args.rate
    started = time.monotonic()
    next_sample = started
    samples = 0
    with args.output.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(FIELDS)
        while True:
            now = time.monotonic()
            if now - started >= args.duration:
                break
            if now < next_sample:
                time.sleep(next_sample - now)
            now = time.monotonic()
            axis = device.axis0
            writer.writerow(
                (
                    f"{now:.9f}",
                    f"{device.vbus_voltage:.6f}",
                    f"{device.ibus:.6f}",
                    f"{axis.encoder.vel_estimate:.6f}",
                    f"{axis.controller.input_vel:.6f}",
                    f"{axis.controller.vel_setpoint:.6f}",
                    f"{axis.motor.current_control.Iq_setpoint:.6f}",
                    f"{axis.motor.current_control.Iq_measured:.6f}",
                    int(axis.current_state),
                    int(axis.error),
                    int(axis.motor.error),
                    int(axis.encoder.error),
                    int(axis.controller.error),
                )
            )
            samples += 1
            next_sample += period_s
        output_file.flush()

    print(f"Captured {samples} ODrive samples to {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
