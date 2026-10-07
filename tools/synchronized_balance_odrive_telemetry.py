#!/usr/bin/env python3
"""Record STM32 balance telemetry and ODrive Axis 0 USB data on one timebase."""

from __future__ import annotations

import argparse
import bisect
import csv
import os
import select
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import odrive

from balance_telemetry import (
    CSV_FIELDS,
    Sample,
    Uart7StreamDecoder,
    configure_115200,
    parse_sample,
)


ODRIVE_FIELDS = (
    "odrive_host_monotonic_s",
    "odrive_age_ms",
    "odrive_poll_span_ms",
    "odrive_vbus_v",
    "odrive_ibus_a",
    "odrive_wheel_tps",
    "odrive_input_vel_tps",
    "odrive_vel_setpoint_tps",
    "odrive_iq_setpoint_a",
    "odrive_iq_measured_a",
    "odrive_axis_state",
    "odrive_axis_error",
    "odrive_motor_error",
    "odrive_encoder_error",
    "odrive_controller_error",
)


@dataclass(frozen=True)
class TimedBalanceSample:
    host_monotonic_s: float
    sample: Sample


@dataclass(frozen=True)
class ODriveSample:
    host_monotonic_s: float
    poll_span_ms: float
    vbus_v: float
    ibus_a: float
    wheel_tps: float
    input_vel_tps: float
    vel_setpoint_tps: float
    iq_setpoint_a: float
    iq_measured_a: float
    axis_state: int
    axis_error: int
    motor_error: int
    encoder_error: int
    controller_error: int


def poll_odrive(
    device: object,
    rate_hz: float,
    stop_event: threading.Event,
    samples: list[ODriveSample],
    error_count: list[int],
) -> None:
    period_s = 1.0 / rate_hz
    next_sample_s = time.monotonic()
    while not stop_event.is_set():
        started_s = time.monotonic()
        try:
            axis = device.axis0
            vbus_v = float(device.vbus_voltage)
            ibus_a = float(device.ibus)
            wheel_tps = float(axis.encoder.vel_estimate)
            input_vel_tps = float(axis.controller.input_vel)
            vel_setpoint_tps = float(axis.controller.vel_setpoint)
            iq_setpoint_a = float(axis.motor.current_control.Iq_setpoint)
            iq_measured_a = float(axis.motor.current_control.Iq_measured)
            axis_state = int(axis.current_state)
            axis_error = int(axis.error)
            motor_error = int(axis.motor.error)
            encoder_error = int(axis.encoder.error)
            controller_error = int(axis.controller.error)
            finished_s = time.monotonic()
            samples.append(
                ODriveSample(
                    host_monotonic_s=(started_s + finished_s) * 0.5,
                    poll_span_ms=(finished_s - started_s) * 1000.0,
                    vbus_v=vbus_v,
                    ibus_a=ibus_a,
                    wheel_tps=wheel_tps,
                    input_vel_tps=input_vel_tps,
                    vel_setpoint_tps=vel_setpoint_tps,
                    iq_setpoint_a=iq_setpoint_a,
                    iq_measured_a=iq_measured_a,
                    axis_state=axis_state,
                    axis_error=axis_error,
                    motor_error=motor_error,
                    encoder_error=encoder_error,
                    controller_error=controller_error,
                )
            )
        except Exception:
            error_count[0] += 1
            finished_s = time.monotonic()

        next_sample_s += period_s
        wait_s = next_sample_s - finished_s
        if wait_s <= 0.0:
            next_sample_s = finished_s
        else:
            stop_event.wait(wait_s)


def nearest_odrive_sample(
    timestamps: list[float], samples: list[ODriveSample], target_s: float
) -> ODriveSample:
    index = bisect.bisect_left(timestamps, target_s)
    if index <= 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before = samples[index - 1]
    after = samples[index]
    if target_s - before.host_monotonic_s <= after.host_monotonic_s - target_s:
        return before
    return after


def odrive_csv_row(sample: ODriveSample, balance_time_s: float) -> tuple[object, ...]:
    return (
        f"{sample.host_monotonic_s:.9f}",
        f"{abs(sample.host_monotonic_s - balance_time_s) * 1000.0:.3f}",
        f"{sample.poll_span_ms:.3f}",
        f"{sample.vbus_v:.6f}",
        f"{sample.ibus_a:.6f}",
        f"{sample.wheel_tps:.6f}",
        f"{sample.input_vel_tps:.6f}",
        f"{sample.vel_setpoint_tps:.6f}",
        f"{sample.iq_setpoint_a:.6f}",
        f"{sample.iq_measured_a:.6f}",
        sample.axis_state,
        f"0x{sample.axis_error:X}",
        f"0x{sample.motor_error:X}",
        f"0x{sample.encoder_error:X}",
        f"0x{sample.controller_error:X}",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="STM32 telemetry port, e.g. /dev/cu.usbserial-120")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--odrive-rate", type=float, default=20.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.odrive_rate <= 0.0:
        parser.error("duration and odrive-rate must be positive")

    device = odrive.find_any(timeout=10)
    if device is None:
        raise RuntimeError("ODrive not found")

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_115200(fd)
    balance_samples: list[TimedBalanceSample] = []
    odrive_samples: list[ODriveSample] = []
    odrive_errors = [0]
    malformed_lines = 0
    decoder = Uart7StreamDecoder()
    stop_event = threading.Event()
    poll_thread = threading.Thread(
        target=poll_odrive,
        args=(device, args.odrive_rate, stop_event, odrive_samples, odrive_errors),
        daemon=True,
    )
    started_s = time.monotonic()
    poll_thread.start()

    try:
        while time.monotonic() - started_s < args.duration:
            readable, _, _ = select.select([fd], [], [], 0.1)
            if not readable:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                continue
            for event_type, payload in decoder.feed(chunk):
                sample = payload if event_type == "balance" else None
                if event_type == "error":
                    malformed_lines += 1
                    continue
                if event_type == "line":
                    try:
                        sample = parse_sample(str(payload))
                    except (ValueError, OverflowError):
                        malformed_lines += 1
                        continue
                if sample is not None:
                    balance_samples.append(
                        TimedBalanceSample(time.monotonic(), sample)
                    )
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        poll_thread.join(timeout=3.0)
        os.close(fd)

    if not balance_samples:
        raise RuntimeError("no STM32 balance telemetry received")
    if not odrive_samples:
        raise RuntimeError("no ODrive USB telemetry received")

    odrive_timestamps = [sample.host_monotonic_s for sample in odrive_samples]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ages_ms: list[float] = []
    with args.output.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(("balance_host_monotonic_s", *CSV_FIELDS, *ODRIVE_FIELDS))
        for timed_balance in balance_samples:
            nearest = nearest_odrive_sample(
                odrive_timestamps, odrive_samples, timed_balance.host_monotonic_s
            )
            age_ms = abs(nearest.host_monotonic_s - timed_balance.host_monotonic_s) * 1000.0
            ages_ms.append(age_ms)
            writer.writerow(
                (
                    f"{timed_balance.host_monotonic_s:.9f}",
                    *timed_balance.sample.csv_row(),
                    *odrive_csv_row(nearest, timed_balance.host_monotonic_s),
                )
            )

    ages_ms.sort()
    p95_age_ms = ages_ms[min(len(ages_ms) - 1, int(len(ages_ms) * 0.95))]
    print(
        f"Captured {len(balance_samples)} STM32 and {len(odrive_samples)} ODrive samples; "
        f"nearest-sample age median {ages_ms[len(ages_ms) // 2]:.1f} ms, "
        f"p95 {p95_age_ms:.1f} ms; malformed UART lines {malformed_lines}; "
        f"ODrive poll errors {odrive_errors[0]}."
    )
    print(f"Merged log: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
