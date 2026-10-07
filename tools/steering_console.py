#!/usr/bin/env python3
"""Send safe UART7 steering commands and display the steering telemetry stream."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import select
import sys
try:
    import termios
except ImportError:  # Web control station uses pyserial on Windows.
    termios = None
import time
from pathlib import Path

try:
    from balance_telemetry import Uart7StreamDecoder
except ModuleNotFoundError:  # Also support `python -m tools.steering_console`.
    from tools.balance_telemetry import Uart7StreamDecoder


FIELDS = (
    "time_ms",
    "mode",
    "yaw_deg",
    "yaw_rate_dps",
    "target_heading_deg",
    "heading_error_deg",
    "rear_wheel_tps",
    "kp",
    "ki",
    "kd",
    "integral_deg_s",
    "raw_offset_us",
    "commanded_offset_us",
    "pulse_us",
    "flags",
)

FLAG_NAMES = (
    (1 << 0, "OUTPUT"),
    (1 << 1, "TIMEOUT"),
    (1 << 2, "GATED"),
    (1 << 3, "SAT"),
    (1 << 4, "SLEW"),
    (1 << 5, "ROLL_DERATE"),
    (1 << 6, "BALANCE"),
    (1 << 7, "IMU"),
    (1 << 8, "REAR_READY"),
    (1 << 9, "BACKLASH_CENTER"),
)

RAMP_COMMAND_PERIOD_S = 0.02

MODE_NAMES = {0: "DISABLED", 1: "CAL", 2: "HOLD", 3: "ANGLE"}


def configure_serial(fd: int) -> None:
    if termios is None:
        raise RuntimeError("POSIX serial configuration is unavailable on this platform")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def command(*parts: str) -> bytes:
    sequence = int(time.monotonic() * 1000) & 0xFFFFFFFF
    payload = ",".join(("P", str(sequence), *parts))
    return f"@{payload},{crc16_ccitt(payload.encode('ascii')):04X}\n".encode("ascii")


def parse_steering(line: str) -> dict[str, object] | None:
    if not line.startswith("S,"):
        return None
    values = line.split(",")
    if len(values) != 16:
        raise ValueError(f"expected 16 steering fields, received {len(values)}")
    sample: dict[str, object] = {
        "time_ms": int(values[1]),
        "mode": int(values[2]),
        "yaw_deg": float(values[3]),
        "yaw_rate_dps": float(values[4]),
        "target_heading_deg": float(values[5]),
        "heading_error_deg": float(values[6]),
        "rear_wheel_tps": float(values[7]),
        "kp": float(values[8]),
        "ki": float(values[9]),
        "kd": float(values[10]),
        "integral_deg_s": float(values[11]),
        "raw_offset_us": float(values[12]),
        "commanded_offset_us": float(values[13]),
        "pulse_us": int(values[14]),
        "flags": int(values[15], 0),
    }
    floats = [value for key, value in sample.items() if isinstance(value, float)]
    if not all(math.isfinite(value) for value in floats):
        raise ValueError("non-finite steering telemetry")
    return sample


def flag_text(flags: int) -> str:
    return "|".join(name for mask, name in FLAG_NAMES if flags & mask) or "NONE"


def default_output() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"steering_{stamp}.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="UART7 serial port, e.g. /dev/cu.usbserial-120")
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument(
        "--cal-offset",
        type=float,
        metavar="MICROSECONDS",
        help="command a repeated calibration offset; firmware applies its configured safety limits",
    )
    actions.add_argument(
        "--cal-ramp-offset",
        type=float,
        metavar="MICROSECONDS",
        help="linearly ramp from center to a calibration offset",
    )
    actions.add_argument(
        "--angle-offset",
        type=float,
        metavar="MICROSECONDS",
        help="command the formal static/moving steering-angle mode (range +/-150 us)",
    )
    actions.add_argument("--hold", type=float, metavar="DEGREES", help="hold an absolute heading")
    actions.add_argument(
        "--capture",
        action="store_true",
        help="capture the first received yaw and repeatedly hold that fixed heading",
    )
    actions.add_argument("--disable", action="store_true", help="disable steering and slew to center")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--ramp-duration", type=float, default=10.0)
    parser.add_argument(
        "--ramp-start-offset",
        type=float,
        default=0.0,
        metavar="MICROSECONDS",
        help="starting offset for --cal-ramp-offset (default: 0)",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-log", action="store_true")
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.ramp_duration <= 0:
        parser.error("--ramp-duration must be greater than zero")
    if args.no_log and args.output is not None:
        parser.error("--no-log and --output cannot be combined")
    if args.cal_offset is not None and not -650.0 <= args.cal_offset <= 1350.0:
        parser.error("--cal-offset must be between -650 and 1350 us")
    if args.cal_ramp_offset is not None and not -650.0 <= args.cal_ramp_offset <= 1350.0:
        parser.error("--cal-ramp-offset must be between -650 and 1350 us")
    if args.angle_offset is not None and not -150.0 <= args.angle_offset <= 150.0:
        parser.error("--angle-offset must be between -150 and 150 us")
    if args.angle_offset is not None and 0.0 < abs(args.angle_offset) < 50.0:
        parser.error("--angle-offset must be 0 or outside the +/-50 us deadband")
    if args.angle_offset is not None and not math.isclose(
        args.angle_offset / 5.0, round(args.angle_offset / 5.0), abs_tol=1e-6
    ):
        parser.error("--angle-offset must use 5 us steps")
    if not -650.0 <= args.ramp_start_offset <= 1350.0:
        parser.error("--ramp-start-offset must be between -650 and 1350 us")
    if args.cal_ramp_offset is None and args.ramp_start_offset != 0.0:
        parser.error("--ramp-start-offset requires --cal-ramp-offset")
    if args.cal_ramp_offset is not None and args.duration < args.ramp_duration:
        parser.error("--duration must be at least --ramp-duration")

    output = None if args.no_log else (args.output or default_output())
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_serial(fd)
    log_file = None
    writer = None
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        log_file = output.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(log_file, fieldnames=FIELDS)
        writer.writeheader()

    repeated_command: bytes | None = None
    ramp_target: float | None = None
    if args.cal_offset is not None:
        repeated_command = command("STEER", "CAL", f"{args.cal_offset:.3f}")
    elif args.cal_ramp_offset is not None:
        ramp_target = args.cal_ramp_offset
    elif args.angle_offset is not None:
        repeated_command = command("STEER", "ANGLE", f"{args.angle_offset:.3f}")
    elif args.hold is not None:
        repeated_command = command("STEER", "HOLD", f"{args.hold:.3f}")
    elif args.disable:
        os.write(fd, command("STEER", "DISABLE"))

    started = time.monotonic()
    last_command = 0.0
    decoder = Uart7StreamDecoder()
    sample_count = 0
    captured_heading: float | None = None
    try:
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if ramp_target is not None and now - last_command >= RAMP_COMMAND_PERIOD_S:
                progress = min(1.0, (now - started) / args.ramp_duration)
                offset = args.ramp_start_offset + (
                    ramp_target - args.ramp_start_offset
                ) * progress
                os.write(fd, command("STEER", "CAL", f"{offset:.3f}"))
                last_command = now
            elif repeated_command is not None and now - last_command >= 0.4:
                os.write(fd, repeated_command)
                last_command = now

            readable, _, _ = select.select([fd], [], [], 0.02)
            if not readable:
                continue
            for event_type, payload in decoder.feed(os.read(fd, 4096)):
                sample = payload if event_type == "steering" else None
                if event_type == "line":
                    line = str(payload)
                    if line.startswith("#P,"):
                        continue
                    try:
                        sample = parse_steering(line)
                    except (ValueError, OverflowError):
                        continue
                if sample is None:
                    continue
                if args.capture and captured_heading is None:
                    captured_heading = float(sample["yaw_deg"])
                    repeated_command = command("STEER", "HOLD", f"{captured_heading:.3f}")
                    last_command = 0.0
                    print(f"Captured heading {captured_heading:.3f} deg")
                sample_count += 1
                if writer is not None:
                    writer.writerow(sample)
                    if sample_count % 20 == 0:
                        log_file.flush()
                mode = MODE_NAMES.get(int(sample["mode"]), str(sample["mode"]))
                flags = flag_text(int(sample["flags"]))
                print(
                    "\r"
                    f"{mode:<8} yaw {float(sample['yaw_deg']):+7.2f} deg  "
                    f"rate {float(sample['yaw_rate_dps']):+7.2f} dps  "
                    f"target {float(sample['target_heading_deg']):+7.2f}  "
                    f"error {float(sample['heading_error_deg']):+7.2f}  "
                    f"rear {float(sample['rear_wheel_tps']):+6.2f} tps  "
                    f"offset {float(sample['commanded_offset_us']):+6.1f} us  "
                    f"pulse {int(sample['pulse_us']):4d} us  {flags:<32}",
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        if repeated_command is not None or ramp_target is not None:
            os.write(fd, command("STEER", "DISABLE"))
        print()
        if log_file is not None:
            log_file.close()
        os.close(fd)

    if output is not None:
        print(f"Recorded {sample_count} samples to {output.resolve()}")
    else:
        print(f"Received {sample_count} steering samples")
    return 0 if sample_count else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except OSError as error:
        print(f"Serial error: {error}", file=sys.stderr)
        raise SystemExit(2)
