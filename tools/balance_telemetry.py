#!/usr/bin/env python3
"""Monitor and record the bicycle balance telemetry stream on UART7."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import select
import struct
import sys
try:
    import termios
except ImportError:  # Windows uses pyserial in the web control station.
    termios = None
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
    "rate_target_dps",
    "rate_error_dps",
    "rate_p_tps",
    "rate_i_tps",
    "rate_d_tps",
    "vel_raw_tps",
    "vel_slew_error_tps",
    "accel_raw_tps2",
    "accel_limit_error_tps2",
    "current_cmd_a",
    "torque_cmd_nm",
    "rate_kp",
    "rate_ki",
    "rate_kd",
    "angle_kp",
    "angle_ki",
    "angle_kd",
    "wheel_kp",
    "wheel_ki",
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
    (1 << 13, "RATE_SLEW"),
    (1 << 14, "ODRIVE_READY"),
    (1 << 15, "ODRIVE_TIMEOUT"),
    (1 << 16, "ODRIVE_FAULT"),
    (1 << 17, "UART7_RX_SEEN"),
    (1 << 18, "UART7_RX_OVERFLOW"),
    (1 << 19, "RATE_TARGET_SAT"),
    (1 << 20, "FALL_DISARM"),
    (1 << 21, "ZERO_STEER_BLOCK"),
    (1 << 22, "ACCEL_BOOST_120"),
    (1 << 23, "SPEED_ENVELOPE"),
    (1 << 24, "TORQUE_CONTROL"),
    (1 << 25, "ODRIVE_RECOVERY_PENDING"),
    (1 << 26, "ODRIVE_RECOVERY_FAILED"),
)

TUNABLE_PARAMETERS = {
    "RATE_KP",
    "RATE_KI",
    "RATE_KD",
    "ANGLE_KP",
    "ANGLE_KI",
    "ANGLE_KD",
    "WHEEL_KP",
    "WHEEL_KI",
    "STEER_KP",
    "STEER_KI",
    "STEER_KD",
}


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
    rate_target_dps: float
    rate_error_dps: float
    rate_p_tps: float
    rate_i_tps: float
    rate_d_tps: float
    vel_raw_tps: float
    vel_slew_error_tps: float
    accel_raw_tps2: float
    accel_limit_error_tps2: float
    current_cmd_a: float
    torque_cmd_nm: float
    rate_kp: float
    rate_ki: float
    rate_kd: float
    angle_kp: float
    angle_ki: float
    angle_kd: float
    wheel_kp: float
    wheel_ki: float
    flags: int
    compact: bool = False

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
            self.rate_target_dps,
            self.rate_error_dps,
            self.rate_p_tps,
            self.rate_i_tps,
            self.rate_d_tps,
            self.vel_raw_tps,
            self.vel_slew_error_tps,
            self.accel_raw_tps2,
            self.accel_limit_error_tps2,
            self.current_cmd_a,
            self.torque_cmd_nm,
            self.rate_kp,
            self.rate_ki,
            self.rate_kd,
            self.angle_kp,
            self.angle_ki,
            self.angle_kd,
            self.wheel_kp,
            self.wheel_ki,
            f"0x{self.flags:02X}",
        )


def parse_sample(line: str) -> Sample | None:
    if not line.startswith("B,"):
        return None

    fields = line.split(",")
    if len(fields) not in (16, 23, 31, 33, 35):
        raise ValueError(
            f"expected 16, 23, 31, 33, or 35 fields, received {len(fields)}"
        )

    is_extended = len(fields) >= 23
    has_direct_accel = len(fields) in (33, 35)
    has_torque_command = len(fields) == 35
    has_tuning = len(fields) in (31, 33, 35)
    tuning_offset = (2 if has_direct_accel else 0) + (
        2 if has_torque_command else 0
    )

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
        rate_target_dps=float(fields[15]) if is_extended else 0.0,
        rate_error_dps=float(fields[16]) if is_extended else 0.0,
        rate_p_tps=float(fields[17]) if is_extended else 0.0,
        rate_i_tps=float(fields[18]) if is_extended else 0.0,
        rate_d_tps=float(fields[19]) if is_extended else 0.0,
        vel_raw_tps=float(fields[20]) if is_extended else float(fields[6]),
        vel_slew_error_tps=float(fields[21]) if is_extended else 0.0,
        accel_raw_tps2=float(fields[22]) if has_direct_accel else float(fields[7]),
        accel_limit_error_tps2=float(fields[23]) if has_direct_accel else 0.0,
        current_cmd_a=float(fields[24]) if has_torque_command else 0.0,
        torque_cmd_nm=float(fields[25]) if has_torque_command else 0.0,
        rate_kp=float(fields[22 + tuning_offset]) if has_tuning else 0.0,
        rate_ki=float(fields[23 + tuning_offset]) if has_tuning else 0.0,
        rate_kd=float(fields[24 + tuning_offset]) if has_tuning else 0.0,
        angle_kp=float(fields[25 + tuning_offset]) if has_tuning else 0.0,
        angle_ki=float(fields[26 + tuning_offset]) if has_tuning else 0.0,
        angle_kd=float(fields[27 + tuning_offset]) if has_tuning else 0.0,
        wheel_kp=float(fields[28 + tuning_offset]) if has_tuning else 0.0,
        wheel_ki=float(fields[29 + tuning_offset]) if has_tuning else 0.0,
        flags=int(fields[30 + tuning_offset] if has_tuning else (fields[22] if is_extended else fields[15]), 0),
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
        sample.rate_target_dps,
        sample.rate_error_dps,
        sample.rate_p_tps,
        sample.rate_i_tps,
        sample.rate_d_tps,
        sample.vel_raw_tps,
        sample.vel_slew_error_tps,
        sample.accel_raw_tps2,
        sample.accel_limit_error_tps2,
        sample.current_cmd_a,
        sample.torque_cmd_nm,
        sample.rate_kp,
        sample.rate_ki,
        sample.rate_kd,
        sample.angle_kp,
        sample.angle_ki,
        sample.angle_kd,
        sample.wheel_kp,
        sample.wheel_ki,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise ValueError("sample contains a non-finite value")
    return sample


COMPACT_MAGIC = b"\xA5\x5A"
COMPACT_VERSION = 1
COMPACT_BALANCE_TYPE = 1
COMPACT_STEERING_TYPE = 2
COMPACT_BASIC_TYPE = 3
COMPACT_BALANCE_LENGTH = 65
COMPACT_STEERING_LENGTH = 34
COMPACT_BASIC_LENGTH = 46


def _parse_compact_balance(frame: bytes) -> Sample:
    if len(frame) != COMPACT_BALANCE_LENGTH:
        raise ValueError(f"unexpected compact balance length {len(frame)}")
    values = struct.unpack_from("<III21hI", frame, 5)
    (
        sequence,
        uptime_ms,
        persistence_sequence,
        roll_mdeg,
        roll_rate_cdeg,
        wheel_centi,
        velocity_centi,
        acceleration_deci,
        base_zero_mdeg,
        control_zero_mdeg,
        candidate_zero_mdeg,
        persisted_zero_mdeg,
        zero_rate_micro,
        wheel_bias_mdeg,
        rate_target_cdeg,
        rate_error_cdeg,
        rate_p_centi,
        rate_i_centi,
        rate_d_centi,
        raw_velocity_centi,
        velocity_slew_centi,
        raw_acceleration_deci,
        acceleration_error_deci,
        current_centi,
        flags,
    ) = values
    return Sample(
        time_ms=uptime_ms,
        seq=sequence,
        roll_deg=roll_mdeg / 1000.0,
        roll_rate_dps=roll_rate_cdeg / 100.0,
        wheel_tps=wheel_centi / 100.0,
        vel_cmd_tps=velocity_centi / 100.0,
        accel_cmd_tps2=acceleration_deci / 10.0,
        zero_base_deg=base_zero_mdeg / 1000.0,
        zero_control_deg=control_zero_mdeg / 1000.0,
        zero_candidate_deg=candidate_zero_mdeg / 1000.0,
        zero_persisted_deg=persisted_zero_mdeg / 1000.0,
        zero_persist_seq=persistence_sequence,
        zero_rate_dps=zero_rate_micro / 1_000_000.0,
        wheel_bias_deg=wheel_bias_mdeg / 1000.0,
        rate_target_dps=rate_target_cdeg / 100.0,
        rate_error_dps=rate_error_cdeg / 100.0,
        rate_p_tps=rate_p_centi / 100.0,
        rate_i_tps=rate_i_centi / 100.0,
        rate_d_tps=rate_d_centi / 100.0,
        vel_raw_tps=raw_velocity_centi / 100.0,
        vel_slew_error_tps=velocity_slew_centi / 100.0,
        accel_raw_tps2=raw_acceleration_deci / 10.0,
        accel_limit_error_tps2=acceleration_error_deci / 10.0,
        current_cmd_a=current_centi / 100.0,
        torque_cmd_nm=0.0,
        rate_kp=0.0,
        rate_ki=0.0,
        rate_kd=0.0,
        angle_kp=0.0,
        angle_ki=0.0,
        angle_kd=0.0,
        wheel_kp=0.0,
        wheel_ki=0.0,
        flags=flags,
        compact=True,
    )


def _parse_compact_steering(frame: bytes) -> dict[str, float | int | bool]:
    if len(frame) != COMPACT_STEERING_LENGTH:
        raise ValueError(f"unexpected compact steering length {len(frame)}")
    uptime_ms = struct.unpack_from("<I", frame, 5)[0]
    mode = frame[9]
    (
        yaw_cdeg,
        yaw_rate_cdeg,
        target_cdeg,
        heading_error_cdeg,
        rear_wheel_centi,
        integral_centi,
        raw_offset_deci,
        commanded_offset_deci,
        pulse_us,
        flags,
    ) = struct.unpack_from("<HhHhhhhhHI", frame, 10)
    return {
        "time_ms": uptime_ms,
        "mode": mode,
        "yaw_deg": yaw_cdeg / 100.0,
        "yaw_rate_dps": yaw_rate_cdeg / 100.0,
        "target_heading_deg": target_cdeg / 100.0,
        "heading_error_deg": heading_error_cdeg / 100.0,
        "rear_wheel_tps": rear_wheel_centi / 100.0,
        "kp": 0.0,
        "ki": 0.0,
        "kd": 0.0,
        "integral_deg_s": integral_centi / 100.0,
        "raw_offset_us": raw_offset_deci / 10.0,
        "commanded_offset_us": commanded_offset_deci / 10.0,
        "pulse_us": pulse_us,
        "flags": flags,
        "compact": True,
    }


def _parse_compact_basic(frame: bytes) -> dict[str, object]:
    if len(frame) != COMPACT_BASIC_LENGTH:
        raise ValueError(f"unexpected compact basic length {len(frame)}")
    (
        uptime_ms,
        balance_flags,
        steering_mode,
        steering_raw_deci,
        steering_commanded_deci,
        steering_pulse_us,
        steering_flags,
        drive_mode,
        demo_phase,
        drive_flags,
        requested_milli,
        output_milli,
        actual_milli,
        odometry_mm,
        segment_mm,
        demo_cycle_count,
    ) = struct.unpack_from("<IIBhhHIBBIhhhihH", frame, 5)
    return {
        "balance_flags": balance_flags,
        "steering": {
            "time_ms": uptime_ms,
            "mode": steering_mode,
            "raw_offset_us": steering_raw_deci / 10.0,
            "commanded_offset_us": steering_commanded_deci / 10.0,
            "pulse_us": steering_pulse_us,
            "flags": steering_flags,
            "basic": True,
        },
        "drive": {
            "time_ms": uptime_ms,
            "mode": drive_mode,
            "demo_phase": demo_phase,
            "requested_speed_mps": requested_milli / 1000.0,
            "output_speed_mps": output_milli / 1000.0,
            "actual_speed_mps": actual_milli / 1000.0,
            "odometry_m": odometry_mm / 1000.0,
            "segment_distance_m": segment_mm / 1000.0,
            "demo_cycle_count": demo_cycle_count,
            "flags": drive_flags,
            "basic": True,
        },
    }


class Uart7StreamDecoder:
    """Decode interleaved CRC-protected compact frames and text replies."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def clear(self) -> None:
        self.buffer.clear()

    def feed(self, chunk: bytes) -> list[tuple[str, object]]:
        self.buffer.extend(chunk)
        events: list[tuple[str, object]] = []
        while self.buffer:
            if self.buffer.startswith(COMPACT_MAGIC):
                if len(self.buffer) < 5:
                    break
                frame_length = self.buffer[4]
                if frame_length < 7 or frame_length > 128:
                    del self.buffer[0]
                    events.append(("error", "invalid compact frame length"))
                    continue
                if len(self.buffer) < frame_length:
                    break
                frame = bytes(self.buffer[:frame_length])
                del self.buffer[:frame_length]
                expected_crc = struct.unpack_from("<H", frame, frame_length - 2)[0]
                actual_crc = crc16_ccitt(frame[2:-2])
                if actual_crc != expected_crc:
                    events.append(("error", "compact frame CRC mismatch"))
                    continue
                if frame[2] != COMPACT_VERSION:
                    events.append(("error", f"unsupported compact version {frame[2]}"))
                    continue
                try:
                    if frame[3] == COMPACT_BALANCE_TYPE:
                        events.append(("balance", _parse_compact_balance(frame)))
                    elif frame[3] == COMPACT_STEERING_TYPE:
                        events.append(("steering", _parse_compact_steering(frame)))
                    elif frame[3] == COMPACT_BASIC_TYPE:
                        events.append(("basic", _parse_compact_basic(frame)))
                    else:
                        events.append(("error", f"unknown compact frame type {frame[3]}"))
                except (ValueError, struct.error) as error:
                    events.append(("error", str(error)))
                continue

            newline = self.buffer.find(b"\n")
            magic = self.buffer.find(COMPACT_MAGIC)
            if magic >= 0 and (newline < 0 or magic < newline):
                if magic > 0:
                    del self.buffer[:magic]
                    events.append(("error", "discarded bytes before compact frame"))
                continue
            if newline < 0:
                if len(self.buffer) > 1024:
                    self.buffer.clear()
                    events.append(("error", "oversized unterminated text line"))
                break
            raw = bytes(self.buffer[:newline]).rstrip(b"\r")
            del self.buffer[: newline + 1]
            events.append(("line", raw.decode("ascii", errors="replace")))
        return events


def configure_115200(fd: int) -> None:
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


def tuning_command(action: str, *arguments: str) -> bytes:
    sequence = int(time.monotonic() * 1000) & 0xFFFFFFFF
    payload = ",".join(("P", str(sequence), action, *arguments))
    checksum = crc16_ccitt(payload.encode("ascii"))
    return f"@{payload},{checksum:04X}\n".encode("ascii")


def flag_text(flags: int) -> str:
    names = [name for mask, name in FLAG_NAMES if flags & mask]
    known_mask = sum(mask for mask, _ in FLAG_NAMES)
    if flags & ~known_mask:
        names.append(f"UNKNOWN_0x{flags & ~known_mask:X}")
    return "|".join(names) if names else "NONE"


def default_log_path() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"balance_{timestamp}.csv"


def monitor(
    port: str,
    output: Path | None,
    duration_s: float | None,
    command: bytes | None = None,
) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    log_file = None
    writer = None
    sample_count = 0
    malformed_count = 0
    tuning_reply_count = 0
    first_sequence: int | None = None
    first_time_ms: int | None = None
    last_sequence: int | None = None
    last_time_ms: int | None = None
    dropped_samples = 0
    decoder = Uart7StreamDecoder()
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
        if command is not None:
            os.write(fd, command)
            print(f"Sent tuning command: {command.decode('ascii').strip()}")

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
            for event_type, payload in decoder.feed(chunk):
                sample: Sample | None = None
                if event_type == "error":
                    malformed_count += 1
                    continue
                if event_type == "line":
                    line = str(payload)
                    if line.startswith("#P,"):
                        tuning_reply_count += 1
                        print(f"\n{line}")
                        continue
                    if line.startswith("#") or not line:
                        continue
                    try:
                        sample = parse_sample(line)
                    except (ValueError, OverflowError):
                        malformed_count += 1
                        continue
                elif event_type == "balance":
                    sample = payload if isinstance(payload, Sample) else None
                if sample is None:
                    continue

                if first_sequence is None:
                    first_sequence = sample.seq
                    first_time_ms = sample.time_ms
                last_sequence = sample.seq
                if last_time_ms is not None:
                    telemetry_delta_ms = (sample.time_ms - last_time_ms) & 0xFFFFFFFF
                    if telemetry_delta_ms > 75:
                        dropped_samples += max(
                            0, round(telemetry_delta_ms / 50.0) - 1
                        )
                last_time_ms = sample.time_ms
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
                    f"target {sample.rate_target_dps:+7.3f} dps  "
                    f"P/D {sample.rate_p_tps:+7.2f}/{sample.rate_d_tps:+7.2f}  "
                    f"accel {sample.accel_raw_tps2:+7.1f}->{sample.accel_cmd_tps2:+7.1f} tps2  "
                    f"Iq/T {sample.current_cmd_a:+6.1f} A/{sample.torque_cmd_nm:+5.2f} Nm  "
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

    control_rate_text = "unknown"
    if (
        first_sequence is not None
        and first_time_ms is not None
        and last_sequence is not None
        and last_time_ms is not None
    ):
        elapsed_ms = (last_time_ms - first_time_ms) & 0xFFFFFFFF
        sequence_delta = (last_sequence - first_sequence) & 0xFFFFFFFF
        if elapsed_ms > 0:
            control_rate_text = f"{sequence_delta * 1000.0 / elapsed_ms:.1f} Hz"
    print(
        f"Captured {sample_count} samples; estimated UART gaps {dropped_samples}; "
        f"malformed lines {malformed_count}; tuning replies {tuning_reply_count}; "
        f"control snapshots {control_rate_text}."
    )
    return 0 if sample_count and (command is None or tuning_reply_count > 0) else 1


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
    tuning = parser.add_mutually_exclusive_group()
    tuning.add_argument(
        "--get-params",
        action="store_true",
        help="request the current RAM-only PID parameters",
    )
    tuning.add_argument(
        "--set-param",
        nargs=2,
        metavar=("NAME", "VALUE"),
        help="set one bounded PID parameter in RAM and keep monitoring",
    )
    tuning.add_argument(
        "--revert-params",
        action="store_true",
        help="restore all compile-time PID defaults in RAM",
    )
    args = parser.parse_args()

    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.no_log and args.output is not None:
        parser.error("--no-log and --output cannot be used together")

    command = None
    if args.get_params:
        command = tuning_command("GET")
    elif args.revert_params:
        command = tuning_command("REVERT")
    elif args.set_param is not None:
        name = args.set_param[0].upper()
        if name not in TUNABLE_PARAMETERS:
            parser.error(f"unknown parameter {name}; choose from {sorted(TUNABLE_PARAMETERS)}")
        try:
            value = float(args.set_param[1])
        except ValueError:
            parser.error("parameter VALUE must be a finite number")
        if not math.isfinite(value):
            parser.error("parameter VALUE must be finite")
        command = tuning_command("SET", name, f"{value:.9g}")

    output = None if args.no_log else (args.output or default_log_path())
    try:
        return monitor(args.port, output, args.duration, command)
    except OSError as error:
        print(f"Unable to open {args.port}: {error}", file=sys.stderr)
        print("Available serial ports:", file=sys.stderr)
        for candidate in sorted(Path("/dev").glob("cu.*")):
            print(f"  {candidate}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
