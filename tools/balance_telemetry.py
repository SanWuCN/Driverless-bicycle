#!/usr/bin/env python3
"""Monitor and record the bicycle balance telemetry stream on UART7."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import select
import sys
import termios
import time
from dataclasses import dataclass
from pathlib import Path


CSV_FIELDS = (
    "time_ms",
    "seq",
    "roll_deg",
    "roll_rate_dps",
    "wheel_tps",
    "vel_cmd_tps",
    "accel_cmd_tps2",
    "zero_base_deg",
    "zero_control_deg",
    "zero_candidate_deg",
    "zero_persisted_deg",
    "zero_persist_seq",
    "zero_rate_dps",
    "wheel_bias_deg",
    "flags",
)

FLAG_NAMES = (
    (1 << 0, "ZERO_GATE"),
    (1 << 1, "ZERO_LEARNING"),
    (1 << 2, "BIAS_SAT"),
    (1 << 3, "VELOCITY_SAT"),
    (1 << 4, "ACCEL_SAT"),
    (1 << 5, "ZERO_APPLIED"),
    (1 << 6, "ZERO_LIMIT"),
    (1 << 7, "CONTROL_ARMED"),
    (1 << 8, "ARM_WINDOW"),
    (1 << 9, "IMU_READY"),
    (1 << 10, "ZERO_PERSIST_VALID"),
    (1 << 11, "ZERO_PERSIST_SAVED"),
    (1 << 12, "ZERO_PERSIST_ERROR"),
)


@dataclass(frozen=True)
class Sample:
    time_ms: int
    seq: int
    roll_deg: float
    roll_rate_dps: float
    wheel_tps: float
    vel_cmd_tps: float
    accel_cmd_tps2: float
    zero_base_deg: float
    zero_control_deg: float
    zero_candidate_deg: float
    zero_persisted_deg: float
    zero_persist_seq: int
    zero_rate_dps: float
    wheel_bias_deg: float
    flags: int

    def csv_row(self) -> tuple[object, ...]:
        return (
            self.time_ms,
            self.seq,
            self.roll_deg,
            self.roll_rate_dps,
            self.wheel_tps,
            self.vel_cmd_tps,
            self.accel_cmd_tps2,
            self.zero_base_deg,
            self.zero_control_deg,
            self.zero_candidate_deg,
            self.zero_persisted_deg,
            self.zero_persist_seq,
            self.zero_rate_dps,
            self.wheel_bias_deg,
            f"0x{self.flags:02X}",
        )


def parse_sample(line: str) -> Sample | None:
    if not line.startswith("B,"):
        return None

    fields = line.split(",")
    if len(fields) != 16:
        raise ValueError(f"expected 16 fields, received {len(fields)}")

    sample = Sample(
        time_ms=int(fields[1]),
        seq=int(fields[2]),
        roll_deg=float(fields[3]),
        roll_rate_dps=float(fields[4]),
        wheel_tps=float(fields[5]),
        vel_cmd_tps=float(fields[6]),
        accel_cmd_tps2=float(fields[7]),
        zero_base_deg=float(fields[8]),
        zero_control_deg=float(fields[9]),
        zero_candidate_deg=float(fields[10]),
        zero_persisted_deg=float(fields[11]),
        zero_persist_seq=int(fields[12]),
        zero_rate_dps=float(fields[13]),
        wheel_bias_deg=float(fields[14]),
        flags=int(fields[15], 0),
    )
    numeric_values = (
        sample.roll_deg,
        sample.roll_rate_dps,
        sample.wheel_tps,
        sample.vel_cmd_tps,
        sample.accel_cmd_tps2,
        sample.zero_base_deg,
        sample.zero_control_deg,
        sample.zero_candidate_deg,
        sample.zero_persisted_deg,
        sample.zero_rate_dps,
        sample.wheel_bias_deg,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise ValueError("sample contains a non-finite value")
    return sample


def configure_115200(fd: int) -> None:
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


def flag_text(flags: int) -> str:
    names = [name for mask, name in FLAG_NAMES if flags & mask]
    known_mask = sum(mask for mask, _ in FLAG_NAMES)
    if flags & ~known_mask:
        names.append(f"UNKNOWN_0x{flags & ~known_mask:X}")
    return "|".join(names) if names else "NONE"


def default_log_path() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"balance_{timestamp}.csv"


def monitor(port: str, output: Path | None, duration_s: float | None) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    log_file = None
    writer = None
    sample_count = 0
    malformed_count = 0
    last_sequence: int | None = None
    dropped_samples = 0
    received = bytearray()
    started = time.monotonic()
    last_valid = started

    try:
        configure_115200(fd)
        if output is not None:
            output.parent.mkdir(parents=True, exist_ok=True)
            log_file = output.open("w", newline="", encoding="utf-8")
            writer = csv.writer(log_file)
            writer.writerow(CSV_FIELDS)

        print(f"Listening on {port} at 115200 8N1")
        if output is not None:
            print(f"Logging to {output.resolve()}")
        print("Press Ctrl-C to stop.")

        while duration_s is None or (time.monotonic() - started) < duration_s:
            readable, _, _ = select.select([fd], [], [], 0.25)
            if not readable:
                if sample_count == 0 and (time.monotonic() - started) > 3.0:
                    print(
                        "\rNo telemetry yet: check UART7 PE8 TX -> adapter RX and common GND.",
                        end="",
                        flush=True,
                    )
                continue

            chunk = os.read(fd, 4096)
            if not chunk:
                continue
            received.extend(chunk)

            while b"\n" in received:
                raw_line, _, received = received.partition(b"\n")
                line = raw_line.rstrip(b"\r").decode("ascii", errors="replace")
                if line.startswith("#") or not line:
                    continue
                try:
                    sample = parse_sample(line)
                except (ValueError, OverflowError):
                    malformed_count += 1
                    continue
                if sample is None:
                    continue

                if last_sequence is not None:
                    sequence_delta = (sample.seq - last_sequence) & 0xFFFFFFFF
                    if sequence_delta > 25:
                        dropped_samples += max(0, (sequence_delta // 25) - 1)
                last_sequence = sample.seq
                last_valid = time.monotonic()
                sample_count += 1

                if writer is not None:
                    writer.writerow(sample.csv_row())
                    if sample_count % 20 == 0:
                        log_file.flush()

                delta_zero = sample.zero_control_deg - sample.zero_base_deg
                print(
                    "\r"
                    f"roll {sample.roll_deg:+7.3f} deg  "
                    f"rate {sample.roll_rate_dps:+8.3f} dps  "
                    f"wheel {sample.wheel_tps:+7.3f} tps  "
                    f"cmd {sample.vel_cmd_tps:+7.3f} tps  "
                    f"accel {sample.accel_cmd_tps2:+9.1f} tps2  "
                    f"zero {sample.zero_base_deg:+6.3f}->{sample.zero_control_deg:+6.3f} "
                    f"(d {delta_zero:+6.3f})  "
                    f"saved {sample.zero_persisted_deg:+6.3f}#{sample.zero_persist_seq}  "
                    f"{flag_text(sample.flags):<38}",
                    end="",
                    flush=True,
                )

        if sample_count and (time.monotonic() - last_valid) > 2.0:
            print("\nWarning: telemetry stopped before the monitor duration ended.")
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if log_file is not None:
            log_file.close()
        os.close(fd)

    print(
        f"Captured {sample_count} samples; estimated gaps {dropped_samples}; "
        f"malformed lines {malformed_count}."
    )
    return 0 if sample_count else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Display and record STM32 UART7 balance telemetry."
    )
    parser.add_argument("port", help="macOS serial device, e.g. /dev/cu.usbserial-1140")
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV output path (default: timestamped file under logs/)",
    )
    parser.add_argument(
        "--no-log", action="store_true", help="display telemetry without creating a CSV file"
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="stop automatically after this many seconds",
    )
    args = parser.parse_args()

    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.no_log and args.output is not None:
        parser.error("--no-log and --output cannot be used together")

    output = None if args.no_log else (args.output or default_log_path())
    try:
        return monitor(args.port, output, args.duration)
    except OSError as error:
        print(f"Unable to open {args.port}: {error}", file=sys.stderr)
        print("Available serial ports:", file=sys.stderr)
        for candidate in sorted(Path("/dev").glob("cu.*")):
            print(f"  {candidate}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
