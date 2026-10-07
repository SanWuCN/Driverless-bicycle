#!/usr/bin/env python3
"""Local web control station for bicycle PID tuning and synchronized telemetry.

The server binds to 127.0.0.1 by default. It owns UART7 and the ODrive USB
connection, serves the static dashboard, and exposes a small JSON API.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import io
import json
import math
import secrets
import signal
import statistics
import sys
import threading
import time
import webbrowser
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from balance_telemetry import (
    FLAG_NAMES,
    Sample,
    Uart7StreamDecoder,
    crc16_ccitt,
    parse_sample,
)
from steering_console import parse_steering

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import odrive  # type: ignore
except ImportError:  # The dashboard still works without ODrive USB telemetry.
    odrive = None


ROOT = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[1]))
STATIC_ROOT = ROOT / "tools" / "control_station_web"
LOG_ROOT = (Path.home() / "Documents" / "BikeControlStation" / "logs"
            if getattr(sys, "frozen", False) else ROOT / "logs")

PARAMETER_GROUPS: dict[str, list[dict[str, Any]]] = {
    "rate": [
        {"name": "RATE_KP", "label": "Kp", "min": -40.0, "max": -0.5, "step": 0.1},
        {"name": "RATE_KI", "label": "Ki", "min": -5.0, "max": 0.0, "step": 0.01},
        {"name": "RATE_KD", "label": "Kd", "min": -0.05, "max": 0.0, "step": 0.001},
    ],
    "angle": [
        {"name": "ANGLE_KP", "label": "Kp", "min": -10.0, "max": -0.1, "step": 0.1},
        {"name": "ANGLE_KI", "label": "Ki", "min": -0.1, "max": 0.1, "step": 0.001},
        {"name": "ANGLE_KD", "label": "Kd", "min": -5.0, "max": 0.0, "step": 0.1},
    ],
    "wheel": [
        {"name": "WHEEL_KP", "label": "Kp", "min": 0.0, "max": 0.2, "step": 0.005},
        {"name": "WHEEL_KI", "label": "Ki", "min": 0.0, "max": 0.1, "step": 0.001},
    ],
    "steer": [
        {"name": "STEER_KP", "label": "Kp", "min": -10.0, "max": 10.0, "step": 0.1},
        {"name": "STEER_KI", "label": "Ki", "min": -2.0, "max": 2.0, "step": 0.01},
        {"name": "STEER_KD", "label": "Kd", "min": -5.0, "max": 5.0, "step": 0.1},
    ],
}
PARAMETERS = {
    item["name"]: item
    for group in PARAMETER_GROUPS.values()
    for item in group
}

EXPORT_FIELDS = [
    "host_time_s",
    "time_ms",
    "seq",
    "roll_deg",
    "roll_error_deg",
    "roll_rate_dps",
    "rate_target_dps",
    "rate_error_dps",
    "wheel_tps",
    "vel_cmd_tps",
    "accel_cmd_tps2",
    "accel_raw_tps2",
    "current_cmd_a",
    "torque_cmd_nm",
    "zero_control_deg",
    "zero_candidate_deg",
    "zero_persisted_deg",
    "wheel_bias_deg",
    "rate_p_tps",
    "rate_i_tps",
    "rate_d_tps",
    "rate_kp",
    "rate_ki",
    "rate_kd",
    "angle_kp",
    "angle_ki",
    "angle_kd",
    "wheel_kp",
    "wheel_ki",
    "balance_flags",
    "odrive_vbus_v",
    "odrive_ibus_a",
    "odrive_wheel_tps",
    "odrive_input_vel_tps",
    "odrive_input_torque_nm",
    "odrive_vel_setpoint_tps",
    "odrive_iq_setpoint_a",
    "odrive_iq_measured_a",
    "odrive_axis_state",
    "odrive_control_mode",
    "odrive_axis_error",
    "odrive_motor_error",
    "odrive_encoder_error",
    "odrive_controller_error",
    "steer_mode",
    "steer_heading_error_deg",
    "steer_commanded_offset_us",
    "steer_pulse_us",
    "steer_flags",
    "drive_mode",
    "drive_demo_phase",
    "drive_requested_speed_mps",
    "drive_output_speed_mps",
    "drive_actual_speed_mps",
    "drive_odometry_m",
    "drive_segment_distance_m",
    "drive_demo_cycle_count",
    "drive_flags",
]


def flag_names(flags: int) -> list[str]:
    names = [name for mask, name in FLAG_NAMES if flags & mask]
    known_mask = sum(mask for mask, _ in FLAG_NAMES)
    if flags & ~known_mask:
        names.append(f"UNKNOWN_0x{flags & ~known_mask:X}")
    return names


def build_command(sequence: int, action: str, *arguments: str) -> bytes:
    payload = ",".join(("P", str(sequence), action, *arguments))
    checksum = crc16_ccitt(payload.encode("ascii"))
    return f"@{payload},{checksum:04X}\n".encode("ascii")


def flatten_record(record: dict[str, Any]) -> dict[str, Any]:
    balance = record.get("balance") or {}
    odrive_data = record.get("odrive") or {}
    steering = record.get("steering") or {}
    drive = record.get("drive") or {}
    roll_error = (
        float(balance.get("roll_deg", 0.0))
        - float(balance.get("zero_control_deg", 0.0))
        - float(balance.get("wheel_bias_deg", 0.0))
    )
    row: dict[str, Any] = {
        "host_time_s": record.get("host_time_s"),
        "roll_error_deg": roll_error,
        "balance_flags": balance.get("flags_hex", ""),
    }
    for key in EXPORT_FIELDS:
        if key in row:
            continue
        if key.startswith("odrive_"):
            row[key] = odrive_data.get(key.removeprefix("odrive_"), "")
        elif key.startswith("steer_"):
            row[key] = steering.get(key.removeprefix("steer_"), "")
        elif key.startswith("drive_"):
            row[key] = drive.get(key.removeprefix("drive_"), "")
        else:
            row[key] = balance.get(key, "")
    return row


def _median(values: list[float], fallback: float = 0.0) -> float:
    return statistics.median(values) if values else fallback


def _stable_time(
    records: list[dict[str, Any]],
    deviations: list[float],
    start_index: int,
    threshold_deg: float,
    hold_s: float = 1.0,
) -> tuple[float | None, float | None]:
    """Return (time from peak, absolute host time) after a continuous stable hold."""
    if not records or start_index >= len(records):
        return None, None
    peak_time = float(records[start_index]["host_time_s"])
    for index in range(start_index, len(records)):
        window_start = float(records[index]["host_time_s"])
        end = index
        valid = True
        while end < len(records) and float(records[end]["host_time_s"]) - window_start < hold_s:
            balance = records[end].get("balance") or {}
            rate = abs(float(balance.get("roll_rate_dps", 0.0)))
            if abs(deviations[end]) > threshold_deg or rate > 0.8:
                valid = False
                break
            end += 1
        if valid and end < len(records):
            return window_start - peak_time, window_start
    return None, None


def analyze_records(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Analyze one explicit start/stop interval without changing the controller."""
    if len(records) < 5:
        return {
            "quality": "数据不足",
            "quality_key": "unknown",
            "score": 0,
            "summary": "记录区间过短，无法评估。",
            "recommendations": ["至少记录 3 秒，并包含完整的扰动与恢复过程。"],
            "sample_count": len(records),
        }

    host_times = [float(record["host_time_s"]) for record in records]
    balances = [record.get("balance") or {} for record in records]
    odrives = [record.get("odrive") or {} for record in records]
    errors = [float(balance.get("roll_error_deg", 0.0)) for balance in balances]
    rates = [float(balance.get("roll_rate_dps", 0.0)) for balance in balances]
    duration_s = max(0.0, host_times[-1] - host_times[0])
    baseline_end = host_times[0] + min(1.5, max(0.5, duration_s * 0.2))
    baseline_indices = [i for i, value in enumerate(host_times) if value <= baseline_end]
    baseline_error = _median([errors[i] for i in baseline_indices])
    baseline_deviations = [errors[i] - baseline_error for i in baseline_indices]
    baseline_mad = _median([abs(value) for value in baseline_deviations])
    deviations = [value - baseline_error for value in errors]
    onset_error_threshold = max(0.25, baseline_mad * 6.0)

    onset_index: int | None = None
    for index, (deviation, rate) in enumerate(zip(deviations, rates)):
        if abs(deviation) >= onset_error_threshold or abs(rate) >= 0.8:
            onset_index = index
            break

    search_start = onset_index if onset_index is not None else 0
    peak_index = max(range(search_start, len(records)), key=lambda i: abs(deviations[i]))
    peak_signed = deviations[peak_index]
    peak_abs = abs(peak_signed)
    disturbance_detected = onset_index is not None and peak_abs >= onset_error_threshold

    # Three-sample smoothing rejects isolated IMU noise while preserving recovery peaks.
    smoothed: list[float] = []
    for index in range(len(deviations)):
        low = max(0, index - 1)
        high = min(len(deviations), index + 2)
        smoothed.append(sum(deviations[low:high]) / (high - low))
    extrema: list[int] = []
    for index in range(max(1, search_start), len(smoothed) - 1):
        value = smoothed[index]
        is_peak = (value >= smoothed[index - 1] and value > smoothed[index + 1]) or (
            value <= smoothed[index - 1] and value < smoothed[index + 1]
        )
        if not is_peak or abs(value) < max(0.12, baseline_mad * 4.0):
            continue
        if extrema and host_times[index] - host_times[extrema[-1]] < 0.3:
            if abs(value) > abs(smoothed[extrema[-1]]):
                extrema[-1] = index
        else:
            extrema.append(index)

    peak_sequence = [
        {
            "time_s": host_times[index] - host_times[0],
            "value_deg": deviations[index],
        }
        for index in extrema[:8]
    ]
    opposite_peaks = [
        abs(smoothed[index])
        for index in extrema
        if index > peak_index and smoothed[index] * peak_signed < 0.0
    ]
    opposite_peak = opposite_peaks[0] if opposite_peaks else 0.0
    overshoot_ratio = opposite_peak / peak_abs if peak_abs > 1e-6 else 0.0

    settle_03_s, settle_03_host = _stable_time(records, deviations, peak_index, 0.3)
    settle_02_s, settle_02_host = _stable_time(records, deviations, peak_index, 0.2)
    onset_to_settle_03_s = (
        settle_03_host - host_times[onset_index]
        if settle_03_host is not None and onset_index is not None
        else None
    )

    flags = [int(balance.get("flags", 0)) for balance in balances]
    accel_sat_count = sum(bool(value & (1 << 4)) for value in flags)
    velocity_sat_count = sum(bool(value & (1 << 3)) for value in flags)
    rate_sat_count = sum(bool(value & (1 << 19)) for value in flags)
    fall_count = sum(bool(value & (1 << 20)) for value in flags)
    accel_boost_count = sum(bool(value & (1 << 22)) for value in flags)
    speed_envelope_count = sum(bool(value & (1 << 23)) for value in flags)
    torque_control_count = sum(bool(value & (1 << 24)) for value in flags)
    odrive_fault_count = sum(bool(value & (1 << 16)) for value in flags)
    odrive_timeout_count = sum(bool(value & (1 << 15)) for value in flags)
    count = len(records)

    def peak_balance(key: str) -> float:
        return max(abs(float(balance.get(key, 0.0))) for balance in balances)

    def peak_odrive(key: str) -> float:
        values = [abs(float(value.get(key, 0.0))) for value in odrives if key in value]
        return max(values) if values else 0.0

    wheel_peak = peak_balance("wheel_tps")
    accel_peak = peak_balance("accel_cmd_tps2")
    accel_raw_peak = peak_balance("accel_raw_tps2")
    rate_peak = peak_balance("roll_rate_dps")
    current_peak = peak_odrive("iq_measured_a")
    current_setpoint_peak = peak_odrive("iq_setpoint_a")
    vbus_values = [float(value["vbus_v"]) for value in odrives if "vbus_v" in value]
    current_limits = [float(value["current_lim_a"]) for value in odrives if "current_lim_a" in value]
    current_limit = current_limits[-1] if current_limits else 0.0
    current_saturated = bool(current_limit and current_peak >= current_limit * 0.98)
    critical = bool(fall_count or odrive_fault_count or odrive_timeout_count)
    accel_sat_pct = accel_sat_count * 100.0 / count
    velocity_sat_pct = velocity_sat_count * 100.0 / count
    rate_sat_pct = rate_sat_count * 100.0 / count
    accel_boost_pct = accel_boost_count * 100.0 / count
    speed_envelope_pct = speed_envelope_count * 100.0 / count
    torque_control_pct = torque_control_count * 100.0 / count

    recommendations: list[str] = []
    if not disturbance_detected:
        quality = "未检测到扰动"
        quality_key = "unknown"
        score = 0
        summary = "区间内没有超过噪声门槛的有效扰动，不能评价恢复性能。"
        recommendations.append("重新开始记录，轻推一次后完全松手，稳定后再停止记录。")
    else:
        score = 100.0
        score -= min(25.0, peak_abs * 3.0)
        score -= min(25.0, (settle_03_s if settle_03_s is not None else 8.0) * 4.0)
        score -= min(20.0, overshoot_ratio * 22.0)
        score -= min(15.0, accel_sat_pct * 0.5)
        score -= 30.0 if critical else 0.0
        score = max(0, round(score))
        if critical or settle_03_s is None:
            quality, quality_key = "危险 / 未恢复", "danger"
        elif peak_abs < 2.0 and settle_03_s <= 3.0 and overshoot_ratio <= 0.65 and accel_sat_pct <= 20.0:
            quality, quality_key = "优秀", "excellent"
        elif peak_abs < 4.0 and settle_03_s <= 5.0 and overshoot_ratio <= 0.85:
            quality, quality_key = "良好", "good"
        else:
            quality, quality_key = "可用但需优化", "fair"
        summary = (
            f"主峰 {peak_signed:+.2f}°，首次反向过冲比 {overshoot_ratio * 100:.0f}%；"
            + (f"峰值后 {settle_03_s:.2f} 秒进入 ±0.3° 稳定带。" if settle_03_s is not None else "记录结束前未稳定进入 ±0.3°。")
        )
        if overshoot_ratio > 0.7:
            recommendations.append("反向过冲偏大，优先小步增加角度环阻尼；不要同时修改 Kp。")
        elif overshoot_ratio < 0.25 and settle_03_s is not None and settle_03_s > 4.0:
            recommendations.append("过冲很小但恢复偏慢，可小步提高角度环响应。")
        if accel_sat_pct > 20.0:
            recommendations.append("加速度限幅占比偏高，本组已进入执行器受限区，PID 对比会失真。")
        if current_saturated:
            recommendations.append("ODrive 电流达到当前上限；不要仅靠继续增益来提升恢复力。")
        if velocity_sat_count:
            recommendations.append("动量轮触及速度限制，应优先改善恢复轨迹和轮速回正。")
        elif speed_envelope_count:
            recommendations.append("动量轮进入25–35 tps速度包络，向外加速已平滑削弱，回正加速仍保持完整。")
        if rate_sat_count:
            recommendations.append("角度环目标角速度触及 ±5°/s 限幅，说明本次已进入大误差区。")
        if critical:
            recommendations.append("本组触发安全/ODrive 故障，停止继续加大扰动并检查故障记录。")
        if not recommendations:
            recommendations.append("本组峰值逐步衰减且无明显限幅，可在相近力度下复测确认重复性。")

    triggered_flags = sorted(
        {name for balance in balances for name in balance.get("flag_names", []) if name in {
            "BIAS_SAT", "VELOCITY_SAT", "ACCEL_SAT", "ZERO_LIMIT", "ODRIVE_TIMEOUT",
            "ODRIVE_FAULT", "RATE_TARGET_SAT", "FALL_DISARM", "ACCEL_BOOST_120",
            "SPEED_ENVELOPE", "TORQUE_CONTROL"
        }}
    )
    return {
        "quality": quality,
        "quality_key": quality_key,
        "score": score,
        "summary": summary,
        "recommendations": recommendations,
        "sample_count": count,
        "sample_rate_hz": (count - 1) / duration_s if duration_s > 0 else 0.0,
        "duration_s": duration_s,
        "disturbance_detected": disturbance_detected,
        "baseline_error_deg": baseline_error,
        "baseline_noise_deg": baseline_mad,
        "disturbance_onset_s": host_times[onset_index] - host_times[0] if onset_index is not None else None,
        "peak_time_s": host_times[peak_index] - host_times[0],
        "peak_signed_deg": peak_signed,
        "peak_abs_deg": peak_abs,
        "opposite_peak_deg": opposite_peak,
        "overshoot_ratio": overshoot_ratio,
        "settle_03_s": settle_03_s,
        "settle_02_s": settle_02_s,
        "onset_to_settle_03_s": onset_to_settle_03_s,
        "peak_sequence": peak_sequence,
        "roll_rate_peak_dps": rate_peak,
        "wheel_speed_peak_tps": wheel_peak,
        "accel_peak_tps2": accel_peak,
        "accel_raw_peak_tps2": accel_raw_peak,
        "current_peak_a": current_peak,
        "current_setpoint_peak_a": current_setpoint_peak,
        "current_limit_a": current_limit,
        "current_saturated": current_saturated,
        "vbus_min_v": min(vbus_values) if vbus_values else None,
        "vbus_max_v": max(vbus_values) if vbus_values else None,
        "accel_sat_pct": accel_sat_pct,
        "velocity_sat_pct": velocity_sat_pct,
        "rate_target_sat_pct": rate_sat_pct,
        "accel_boost_120_pct": accel_boost_pct,
        "speed_envelope_pct": speed_envelope_pct,
        "torque_control_pct": torque_control_pct,
        "fall_samples": fall_count,
        "odrive_fault_samples": odrive_fault_count,
        "odrive_timeout_samples": odrive_timeout_count,
        "triggered_flags": triggered_flags,
        "start_host_time_s": host_times[0],
        "end_host_time_s": host_times[-1],
        "start_record_id": int(records[0]["id"]),
        "end_record_id": int(records[-1]["id"]),
    }


class ControlStation:
    def __init__(self, port: str = "", history_seconds: int = 600, odrive_rate: float = 20.0):
        self.port = port
        self.api_token = secrets.token_urlsafe(32)
        self.max_records = max(2000, history_seconds * 25)
        self.odrive_rate = odrive_rate
        self.lock = threading.RLock()
        self.serial_write_lock = threading.Lock()
        self.drive_command_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.serial_fd: Any | None = None
        self.record_id = 0
        self.event_id = 0
        self.sequence = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self.records: deque[dict[str, Any]] = deque(maxlen=self.max_records)
        self.events: deque[dict[str, Any]] = deque(maxlen=500)
        self.latest_balance: dict[str, Any] | None = None
        self.latest_steering: dict[str, Any] | None = None
        self.latest_drive: dict[str, Any] | None = None
        self.latest_basic: dict[str, Any] | None = None
        self.last_steering_s = 0.0
        self.last_drive_s = 0.0
        self.latest_odrive: dict[str, Any] | None = None
        self.params: dict[str, float] = {}
        self.started_s = time.monotonic()
        self.last_uart_s = 0.0
        self.last_odrive_s = 0.0
        self.malformed_uart = 0
        self.odrive_poll_errors = 0
        self.uart_reconnects = 0
        self.odrive_reconnects = 0
        self.record_file: io.TextIOWrapper | None = None
        self.record_writer: csv.DictWriter | None = None
        self.record_path: Path | None = None
        self.active_session_start_id: int | None = None
        self.active_session_started_s: float | None = None
        self.latest_analysis: dict[str, Any] | None = None
        self.pending_replies: dict[int, tuple[threading.Event, list[str]]] = {}
        self.silent_replies: deque[int] = deque(maxlen=64)
        self.steering_target_us: float | None = None
        self.drive_active = False
        self.drive_target_speed_mps: float | None = None
        self.drive_mode = "stopped"
        self.last_drive_command_s = 0.0
        self.telemetry_mode = "basic"

    def add_event(self, level: str, message: str, kind: str = "system") -> None:
        with self.lock:
            self.event_id += 1
            self.events.append(
                {
                    "id": self.event_id,
                    "time": time.time(),
                    "level": level,
                    "kind": kind,
                    "message": message,
                }
            )

    def start(self) -> None:
        threading.Thread(target=self._uart_loop, name="uart7-reader", daemon=True).start()
        threading.Thread(target=self._odrive_loop, name="odrive-reader", daemon=True).start()
        threading.Thread(target=self._control_keepalive_loop, name="control-keepalive", daemon=True).start()
        self.add_event("info", f"控制站启动，UART={self.port}")

    def close(self) -> None:
        self.stop_event.set()
        with self.lock:
            if self.record_file is not None:
                self.record_file.close()
                self.record_file = None
                self.record_writer = None
            fd = self.serial_fd
            self.serial_fd = None
        if fd is not None:
            try:
                fd.close()
            except Exception:
                pass

    @staticmethod
    def available_ports() -> list[dict[str, str]]:
        if list_ports is None:
            return []
        return [
            {"device": item.device, "description": item.description or item.device}
            for item in list_ports.comports()
        ]

    def select_port(self, port: str) -> None:
        candidates = {item["device"] for item in self.available_ports()}
        if port and port not in candidates:
            raise ValueError("串口不存在；请刷新设备列表")
        with self.lock:
            self.port = port
            old = self.serial_fd
            self.serial_fd = None
            self.steering_target_us = None
            self.drive_active = False
            self.drive_target_speed_mps = None
            self.drive_mode = "stopped"
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
        self.add_event("info", f"UART7 端口已切换：{port or '未选择'}")

    def _uart_loop(self) -> None:
        decoder = Uart7StreamDecoder()
        while not self.stop_event.is_set():
            if self.serial_fd is None:
                if not self.port:
                    self.stop_event.wait(0.5)
                    continue
                try:
                    if serial is None:
                        raise RuntimeError("缺少 pyserial；请运行一键启动器安装依赖")
                    fd = serial.Serial(self.port, 115200, timeout=0.25, write_timeout=1.0)
                    with self.lock:
                        self.serial_fd = fd
                        self.uart_reconnects += 1
                    decoder.clear()
                    self.add_event("ok", "UART7 已连接")
                    self.send_tuning("GET", wait=False)
                    self.send_tuning("TELEM", self.telemetry_mode.upper(), wait=False)
                except Exception as error:
                    self.add_event("error", f"UART7 连接失败：{error}")
                    self.stop_event.wait(2.0)
                    continue

            fd = self.serial_fd
            if fd is None:
                continue
            try:
                chunk = fd.read(8192)
                if not chunk:
                    continue
                for event_type, payload in decoder.feed(chunk):
                    if event_type == "line":
                        self._handle_uart_line(str(payload))
                    elif event_type == "balance" and isinstance(payload, Sample):
                        self._handle_balance_sample(payload)
                    elif event_type == "steering" and isinstance(payload, dict):
                        self._handle_steering(payload)
                    elif event_type == "drive" and isinstance(payload, dict):
                        self._handle_drive(payload)
                    elif event_type == "basic" and isinstance(payload, dict):
                        self._handle_basic(payload)
                    elif event_type == "error":
                        self.malformed_uart += 1
            except Exception as error:
                self.add_event("error", f"UART7 断开：{error}")
                with self.lock:
                    self.steering_target_us = None
                    self.drive_active = False
                    self.drive_target_speed_mps = None
                    self.drive_mode = "stopped"
                    if self.serial_fd is fd:
                        try:
                            self.serial_fd.close()
                        except Exception:
                            pass
                        self.serial_fd = None
                self.stop_event.wait(1.0)

    def _handle_uart_line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("#P,"):
            parts = line.split(",", 3)
            try:
                sequence = int(parts[1])
            except (IndexError, ValueError):
                self.malformed_uart += 1
                return
            with self.lock:
                silent = sequence in self.silent_replies
                if silent:
                    self.silent_replies.remove(sequence)
            if not silent:
                self.add_event("ok" if ",OK," in line else "error", line, "command")
            if ",PARAMS," in line:
                for token in line.split(",PARAMS,", 1)[1].split(","):
                    if "=" not in token:
                        continue
                    name, value = token.split("=", 1)
                    try:
                        self.params[name] = float(value)
                    except ValueError:
                        pass
            pending = self.pending_replies.get(sequence)
            if pending is not None:
                pending[1].append(line)
                pending[0].set()
            return
        if line.startswith("#"):
            return
        try:
            sample = parse_sample(line)
            if sample is not None:
                self._handle_balance_sample(sample)
                return
            steering = parse_steering(line)
            if steering is not None:
                self._handle_steering(steering)
                return
        except (ValueError, OverflowError):
            self.malformed_uart += 1

    def _handle_balance_sample(self, sample: Sample) -> None:
        now = time.monotonic()
        balance = dataclasses.asdict(sample)
        balance["flags_hex"] = f"0x{sample.flags:X}"
        balance["flag_names"] = flag_names(sample.flags)
        balance["roll_error_deg"] = (
            sample.roll_deg - sample.zero_control_deg - sample.wheel_bias_deg
        )
        with self.lock:
            self.latest_balance = balance
            self.last_uart_s = now
            if not sample.compact:
                self.params.update(
                    {
                        "RATE_KP": sample.rate_kp,
                        "RATE_KI": sample.rate_ki,
                        "RATE_KD": sample.rate_kd,
                        "ANGLE_KP": sample.angle_kp,
                        "ANGLE_KI": sample.angle_ki,
                        "ANGLE_KD": sample.angle_kd,
                        "WHEEL_KP": sample.wheel_kp,
                        "WHEEL_KI": sample.wheel_ki,
                    }
                )
            self._append_record_locked(now)

    def _handle_steering(self, steering: dict[str, Any]) -> None:
        steering["flags_hex"] = f"0x{int(steering['flags']):X}"
        with self.lock:
            self.latest_steering = steering
            self.last_steering_s = time.monotonic()
            if not steering.get("compact"):
                self.params.update(
                    {
                        "STEER_KP": float(steering["kp"]),
                        "STEER_KI": float(steering["ki"]),
                        "STEER_KD": float(steering["kd"]),
                    }
                )
            self.last_uart_s = time.monotonic()

    def _handle_drive(self, drive: dict[str, Any]) -> None:
        drive["flags_hex"] = f"0x{int(drive['flags']):X}"
        with self.lock:
            self.latest_drive = drive
            self.last_drive_s = time.monotonic()
            self.last_uart_s = time.monotonic()

    def _handle_basic(self, basic: dict[str, Any]) -> None:
        steering = dict(basic.get("steering") or {})
        drive = dict(basic.get("drive") or {})
        if steering:
            steering["flags_hex"] = f"0x{int(steering.get('flags', 0)):X}"
        if drive:
            drive["flags_hex"] = f"0x{int(drive.get('flags', 0)):X}"
        with self.lock:
            now = time.monotonic()
            merged_steering = dict(self.latest_steering or {})
            merged_steering.update(steering)
            merged_drive = dict(self.latest_drive or {})
            merged_drive.update(drive)
            self.latest_steering = merged_steering
            self.latest_drive = merged_drive
            self.last_steering_s = now
            self.last_drive_s = self.last_steering_s
            self.latest_basic = {
                "balance_flags": int(basic.get("balance_flags", 0)),
                "time_ms": int(drive.get("time_ms", steering.get("time_ms", 0))),
            }
            self.last_uart_s = now
            if (self.drive_active and now - self.last_drive_command_s > 1.0 and
                    int(drive.get("mode", -1)) == 0):
                self.drive_active = False
                self.drive_target_speed_mps = None
                self.drive_mode = "stopped"

    def _append_record_locked(self, now: float) -> None:
        self.record_id += 1
        record = {
            "id": self.record_id,
            "host_time_s": now,
            "balance": dict(self.latest_balance or {}),
            "odrive": dict(self.latest_odrive or {}),
            "steering": dict(self.latest_steering or {}),
            "drive": dict(self.latest_drive or {}),
        }
        self.records.append(record)
        if self.record_writer is not None:
            self.record_writer.writerow(flatten_record(record))
            if self.record_id % 20 == 0 and self.record_file is not None:
                self.record_file.flush()

    def _odrive_loop(self) -> None:
        if odrive is None:
            self.add_event("warn", "当前 Python 环境未安装 odrive；UART 功能仍可使用")
            return
        period = 1.0 / self.odrive_rate
        device = None
        waiting_logged = False
        while not self.stop_event.is_set():
            if device is None:
                try:
                    device = odrive.find_any(timeout=5)
                    if device is None:
                        raise RuntimeError("未找到 ODrive")
                    self.odrive_reconnects += 1
                    self.add_event("ok", "ODrive USB 已连接")
                    waiting_logged = False
                except Exception as error:
                    if not waiting_logged:
                        self.add_event("warn", f"ODrive USB 等待连接：{error}")
                        waiting_logged = True
                    self.stop_event.wait(2.0)
                    continue
            started = time.monotonic()
            try:
                axis = device.axis0
                data = {
                    "vbus_v": float(device.vbus_voltage),
                    "ibus_a": float(device.ibus),
                    "wheel_tps": float(axis.encoder.vel_estimate),
                    "input_vel_tps": float(axis.controller.input_vel),
                    "input_torque_nm": float(getattr(axis.controller, "input_torque", 0.0)),
                    "vel_setpoint_tps": float(axis.controller.vel_setpoint),
                    "iq_setpoint_a": float(axis.motor.current_control.Iq_setpoint),
                    "iq_measured_a": float(axis.motor.current_control.Iq_measured),
                    "axis_state": int(axis.current_state),
                    "axis_error": f"0x{int(axis.error):X}",
                    "motor_error": f"0x{int(axis.motor.error):X}",
                    "encoder_error": f"0x{int(axis.encoder.error):X}",
                    "controller_error": f"0x{int(axis.controller.error):X}",
                    "current_lim_a": float(axis.motor.config.current_lim),
                    "current_range_a": float(axis.motor.config.requested_current_range),
                    "vel_limit_tps": float(axis.controller.config.vel_limit),
                    "vel_ramp_tps2": float(axis.controller.config.vel_ramp_rate),
                    "input_mode": int(axis.controller.config.input_mode),
                    "control_mode": int(axis.controller.config.control_mode),
                }
                finished = time.monotonic()
                data["poll_ms"] = (finished - started) * 1000.0
                with self.lock:
                    self.latest_odrive = data
                    self.last_odrive_s = finished
            except Exception as error:
                self.odrive_poll_errors += 1
                self.add_event("error", f"ODrive USB 断开：{error}")
                device = None
                self.stop_event.wait(1.0)
                continue
            self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))

    def send_tuning(
        self,
        action: str,
        *arguments: str,
        wait: bool = True,
        timeout: float = 2.0,
        log_command: bool = True,
    ) -> str:
        with self.lock:
            fd = self.serial_fd
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            sequence = self.sequence
        if fd is None:
            raise RuntimeError("UART7 未连接")
        event = threading.Event()
        replies: list[str] = []
        if wait:
            with self.lock:
                self.pending_replies[sequence] = (event, replies)
        command = build_command(sequence, action, *arguments)
        try:
            if not log_command:
                with self.lock:
                    self.silent_replies.append(sequence)
            with self.serial_write_lock:
                written = fd.write(command)
                if written != len(command):
                    raise RuntimeError("串口写入不完整，请检查无线连接")
            if log_command:
                self.add_event("info", command.decode("ascii").strip(), "command")
            if not wait:
                return "已发送"
            if not event.wait(timeout):
                raise TimeoutError("MCU 命令回执超时")
            reply = replies[-1]
            if ",ERR," in reply:
                raise RuntimeError(reply)
            return reply
        finally:
            with self.lock:
                self.pending_replies.pop(sequence, None)

    def set_parameter(self, name: str, value: float) -> str:
        descriptor = PARAMETERS.get(name)
        if descriptor is None:
            raise ValueError(f"未知参数：{name}")
        if not math.isfinite(value):
            raise ValueError("参数必须是有限数值")
        if value < descriptor["min"] or value > descriptor["max"]:
            raise ValueError(
                f"{name} 允许范围 {descriptor['min']} … {descriptor['max']}"
            )
        return self.send_tuning("SET", name, f"{value:.9g}")

    def set_telemetry_mode(self, mode: str) -> str:
        mode = mode.lower()
        if mode not in {"basic", "live"}:
            raise ValueError("遥测模式只能是 basic 或 live")
        reply = self.send_tuning("TELEM", mode.upper())
        with self.lock:
            self.telemetry_mode = mode
        self.add_event(
            "ok",
            "已开启详细 IMU 遥测" if mode == "live" else "已切换为低带宽基础状态",
            "telemetry",
        )
        return reply

    @staticmethod
    def _validate_steering_offset(offset_us: float) -> float:
        if not math.isfinite(offset_us):
            raise ValueError("舵角指令必须是有限数值")
        rounded = round(offset_us / 5.0) * 5.0
        if abs(offset_us - rounded) > 1e-6:
            raise ValueError("舵角只能按 5 µs 步进")
        if abs(rounded) > 150.0:
            raise ValueError("舵角范围为 -150 … +150 µs")
        if 0.0 < abs(rounded) < 50.0:
            raise ValueError("机械死区为 50 µs；非零舵角必须至少为 ±50 µs")
        return rounded

    def set_steering_offset(self, offset_us: float) -> str:
        offset_us = self._validate_steering_offset(offset_us)
        with self.lock:
            if self.drive_mode == "demo" or int((self.latest_drive or {}).get("mode", 0)) == 2:
                raise RuntimeError("循环演示正在控制转向；请先停止演示")
        sent_at = time.monotonic()
        if offset_us == 0.0:
            with self.lock:
                self.steering_target_us = None
            try:
                reply = self.send_tuning("STEER", "DISABLE", timeout=1.2)
            except TimeoutError:
                if not self._confirm_steering_target(0.0, sent_at):
                    raise
                reply = "遥测确认已回中（无线回执丢失）"
            self.add_event("ok", "转向平滑回中", "steering")
            return reply
        try:
            reply = self.send_tuning("STEER", "ANGLE", f"{offset_us:.0f}", timeout=1.2)
        except TimeoutError:
            if not self._confirm_steering_target(offset_us, sent_at):
                raise
            self.send_tuning("STEER", "KEEP", wait=False, log_command=False)
            reply = "遥测确认已执行（无线回执丢失）"
        with self.lock:
            self.steering_target_us = offset_us
        direction = "左" if offset_us > 0.0 else "右"
        self.add_event("ok", f"转向保持：{direction} {abs(offset_us):.0f} µs", "steering")
        return reply

    def _confirm_steering_target(self, target_us: float, since_s: float, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                steering = dict(self.latest_steering or {})
                fresh = self.last_steering_s >= since_s
            if not fresh:
                self.stop_event.wait(0.05)
                continue
            raw_offset = steering.get("raw_offset_us")
            mode = int(steering.get("mode", -1))
            if target_us == 0.0:
                if mode == 0 or (raw_offset is not None and abs(float(raw_offset)) <= 1.0):
                    return True
            elif mode == 3 and raw_offset is not None and abs(float(raw_offset) - target_us) <= 1.0:
                return True
            self.stop_event.wait(0.05)
        return False

    def set_drive(self, action: str, speed_mps: float | None = None) -> str:
        with self.drive_command_lock:
            return self._set_drive_serialized(action, speed_mps)

    def _set_drive_serialized(self, action: str, speed_mps: float | None) -> str:
        action = action.lower()
        sent_at = time.monotonic()
        if action == "stop":
            with self.lock:
                self.drive_active = False
                self.drive_target_speed_mps = None
                self.drive_mode = "stopped"
            try:
                reply = self.send_tuning("DRIVE", "STOP", timeout=1.2)
            except TimeoutError:
                if not self._confirm_drive("stopped", 0.0, sent_at):
                    raise
                reply = "遥测确认后轮正在停止（无线回执丢失）"
            self.add_event("ok", "后轮停止", "drive")
            return reply

        if action not in {"manual", "demo"}:
            raise ValueError("未知后轮控制动作")
        if speed_mps is None or not math.isfinite(speed_mps):
            raise ValueError("速度必须是有限数值")
        if action == "manual":
            if abs(speed_mps) < 0.05 or abs(speed_mps) > 1.0:
                raise ValueError("手动速度范围为 ±0.05 … ±1.00 m/s")
        elif speed_mps < 0.05 or speed_mps > 1.0:
            raise ValueError("演示速度范围为 0.05 … 1.00 m/s")

        subcommand = "MANUAL" if action == "manual" else "DEMO"
        try:
            reply = self.send_tuning("DRIVE", subcommand, f"{speed_mps:.3f}", timeout=1.2)
        except TimeoutError:
            if not self._confirm_drive(action, speed_mps, sent_at):
                raise
            self.send_tuning("DRIVE", "KEEP", wait=False, log_command=False)
            reply = "遥测确认后轮指令已执行（无线回执丢失）"
        with self.lock:
            self.drive_active = True
            self.drive_target_speed_mps = speed_mps
            self.drive_mode = action
            self.last_drive_command_s = time.monotonic()
        label = "循环演示" if action == "demo" else ("前进" if speed_mps > 0 else "后退")
        self.add_event("ok", f"后轮{label}：{abs(speed_mps):.2f} m/s", "drive")
        return reply

    def _confirm_drive(self, mode: str, speed_mps: float, since_s: float, timeout: float = 1.0) -> bool:
        expected_mode = {"stopped": 0, "manual": 1, "demo": 2}[mode]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                drive = dict(self.latest_drive or {})
                fresh = self.last_drive_s >= since_s
            if fresh and int(drive.get("mode", -1)) == expected_mode:
                if expected_mode == 0 or abs(float(drive.get("requested_speed_mps", 0.0)) - speed_mps) <= 0.01:
                    return True
            self.stop_event.wait(0.05)
        return False

    def _control_keepalive_loop(self) -> None:
        while not self.stop_event.wait(0.8):
            with self.lock:
                target = self.steering_target_us
                drive_active = self.drive_active
                connected = self.serial_fd is not None
            if not connected:
                continue
            try:
                if target is not None:
                    self.send_tuning("STEER", "KEEP", wait=False, log_command=False)
                if drive_active:
                    self.send_tuning("DRIVE", "KEEP", wait=False, log_command=False)
            except (OSError, RuntimeError):
                with self.lock:
                    self.steering_target_us = None
                    self.drive_active = False
                    self.drive_target_speed_mps = None
                    self.drive_mode = "stopped"

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            balance = dict(self.latest_balance or {})
            odrive_data = dict(self.latest_odrive or {})
            steering = dict(self.latest_steering or {})
            drive = dict(self.latest_drive or {})
            basic = dict(self.latest_basic or {})
            return {
                "server_uptime_s": now - self.started_s,
                "port": self.port,
                "uart_connected": self.serial_fd is not None,
                "uart_age_ms": (now - self.last_uart_s) * 1000.0 if self.last_uart_s else None,
                "odrive_connected": bool(self.last_odrive_s and now - self.last_odrive_s < 1.0),
                "odrive_age_ms": (now - self.last_odrive_s) * 1000.0 if self.last_odrive_s else None,
                "recording": self.record_writer is not None,
                "record_path": str(self.record_path) if self.record_path else None,
                "recording_start_id": self.active_session_start_id,
                "recording_elapsed_s": (
                    now - self.active_session_started_s
                    if self.active_session_started_s is not None
                    else None
                ),
                "latest_analysis": dict(self.latest_analysis or {}),
                "malformed_uart": self.malformed_uart,
                "odrive_poll_errors": self.odrive_poll_errors,
                "params": dict(self.params),
                "balance": balance,
                "odrive": odrive_data,
                "steering": steering,
                "drive": drive,
                "basic": basic,
                "steering_target_us": self.steering_target_us,
                "drive_target_speed_mps": self.drive_target_speed_mps,
                "drive_mode": self.drive_mode,
                "telemetry_mode": self.telemetry_mode,
                "last_record_id": self.record_id,
                "last_event_id": self.event_id,
            }

    def samples_after(self, after: int, limit: int = 500) -> list[dict[str, Any]]:
        with self.lock:
            matches = [record for record in self.records if int(record["id"]) > after]
            return matches[-max(1, min(limit, 2000)) :]

    def events_after(self, after: int) -> list[dict[str, Any]]:
        with self.lock:
            return [event for event in self.events if int(event["id"]) > after]

    @staticmethod
    def _downsample(records: list[dict[str, Any]], limit: int = 4000) -> list[dict[str, Any]]:
        if len(records) <= limit:
            return records
        stride = math.ceil(len(records) / limit)
        sampled = records[::stride]
        if sampled[-1] is not records[-1]:
            sampled.append(records[-1])
        return sampled

    def start_recording(self) -> dict[str, Any]:
        with self.lock:
            if self.record_writer is not None:
                return {
                    "path": str(self.record_path),
                    "start_record_id": self.active_session_start_id,
                }
            LOG_ROOT.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.record_path = LOG_ROOT / f"web_pid_tuning_{stamp}.csv"
            self.record_file = self.record_path.open("w", newline="", encoding="utf-8")
            self.record_writer = csv.DictWriter(self.record_file, fieldnames=EXPORT_FIELDS)
            self.record_writer.writeheader()
            self.active_session_start_id = self.record_id + 1
            self.active_session_started_s = time.monotonic()
            self.latest_analysis = None
            path = str(self.record_path)
        self.add_event("ok", f"开始记录：{path}", "record")
        return {"path": path, "start_record_id": self.active_session_start_id}

    def stop_recording(self) -> dict[str, Any]:
        with self.lock:
            if self.record_writer is None or self.active_session_start_id is None:
                raise ValueError("当前没有正在进行的记录")
            path = str(self.record_path) if self.record_path else None
            start_id = self.active_session_start_id
            end_id = self.record_id
            if self.record_file is not None:
                self.record_file.close()
            self.record_file = None
            self.record_writer = None
            records = [
                record
                for record in self.records
                if start_id is not None and start_id <= int(record["id"]) <= end_id
            ]
            self.active_session_start_id = None
            self.active_session_started_s = None

        analysis = analyze_records(records)
        if path:
            analysis_path = Path(path).with_suffix(".analysis.json")
            analysis["csv_path"] = path
            analysis["analysis_path"] = str(analysis_path)
            analysis_path.write_text(
                json.dumps(analysis, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        with self.lock:
            self.latest_analysis = dict(analysis)
        self.add_event("info", f"停止记录：{path or '无文件'}", "record")
        self.add_event(
            "ok" if analysis.get("quality_key") in {"excellent", "good"} else "warn",
            f"区间评估：{analysis.get('quality')} · {analysis.get('summary')}",
            "analysis",
        )
        return {
            "path": path,
            "analysis": analysis,
            "samples": self._downsample(records),
        }

    def export_csv(self, seconds: float) -> bytes:
        cutoff = time.monotonic() - max(1.0, min(seconds, 600.0))
        with self.lock:
            records = [record for record in self.records if record["host_time_s"] >= cutoff]
        output = io.StringIO()
        writer = csv.DictWriter(output, fieldnames=EXPORT_FIELDS)
        writer.writeheader()
        writer.writerows(flatten_record(record) for record in records)
        return output.getvalue().encode("utf-8")


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "BikeControlStation/1.0"

    @property
    def station(self) -> ControlStation:
        return self.server.station  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        if not self.headers.get("Content-Type", "").lower().startswith("application/json"):
            raise ValueError("请求必须使用 application/json")
        if self.headers.get("X-Bike-Token") != self.station.api_token:
            raise ValueError("控制会话已失效，请刷新页面")
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 65536:
            raise ValueError("请求体为空或过大")
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/bootstrap":
            self._json(
                {
                    "api_token": self.station.api_token,
                    "status": self.station.status(),
                    "parameter_groups": PARAMETER_GROUPS,
                    "samples": self.station.samples_after(0, 1200),
                    "events": self.station.events_after(0),
                }
            )
            return
        if parsed.path == "/api/ports":
            self._json({"ports": self.station.available_ports(), "selected": self.station.port})
            return
        if parsed.path == "/api/status":
            self._json(self.station.status())
            return
        if parsed.path == "/api/samples":
            query = parse_qs(parsed.query)
            after = int(query.get("after", ["0"])[0])
            limit = int(query.get("limit", ["500"])[0])
            self._json({"samples": self.station.samples_after(after, limit)})
            return
        if parsed.path == "/api/events":
            query = parse_qs(parsed.query)
            after = int(query.get("after", ["0"])[0])
            self._json({"events": self.station.events_after(after)})
            return
        if parsed.path == "/api/export":
            query = parse_qs(parsed.query)
            seconds = float(query.get("seconds", ["120"])[0])
            body = self.station.export_csv(seconds)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="bike_telemetry_{stamp}.csv"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self._serve_static(parsed.path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            payload = self._read_json()
            if parsed.path == "/api/tune":
                action = str(payload.get("action", ""))
                if action == "set":
                    reply = self.station.set_parameter(
                        str(payload.get("name", "")).upper(), float(payload.get("value"))
                    )
                elif action == "get":
                    reply = self.station.send_tuning("GET")
                elif action == "revert":
                    reply = self.station.send_tuning("REVERT")
                else:
                    raise ValueError("未知调参动作")
                self._json({"ok": True, "reply": reply})
                return
            if parsed.path == "/api/steering":
                offset_us = float(payload.get("offset_us"))
                reply = self.station.set_steering_offset(offset_us)
                self._json(
                    {
                        "ok": True,
                        "reply": reply,
                        "target_us": self.station.steering_target_us,
                    }
                )
                return
            if parsed.path == "/api/drive":
                action = str(payload.get("action", ""))
                speed_value = payload.get("speed_mps")
                speed_mps = None if speed_value is None else float(speed_value)
                reply = self.station.set_drive(action, speed_mps)
                self._json(
                    {
                        "ok": True,
                        "reply": reply,
                        "mode": self.station.drive_mode,
                        "target_speed_mps": self.station.drive_target_speed_mps,
                    }
                )
                return
            if parsed.path == "/api/telemetry":
                mode = str(payload.get("mode", ""))
                reply = self.station.set_telemetry_mode(mode)
                self._json({"ok": True, "reply": reply, "mode": mode.lower()})
                return
            if parsed.path == "/api/port":
                port = str(payload.get("port", ""))
                self.station.select_port(port)
                self._json({"ok": True, "selected": self.station.port})
                return
            if parsed.path == "/api/record":
                action = str(payload.get("action", ""))
                if action == "start":
                    result = self.station.start_recording()
                elif action == "stop":
                    result = self.station.stop_recording()
                else:
                    raise ValueError("未知记录动作")
                self._json({"ok": True, **result})
                return
            if parsed.path == "/api/mark":
                label = str(payload.get("label", "扰动标记"))[:80]
                self.station.add_event("info", label, "marker")
                self._json({"ok": True})
                return
            self._json({"ok": False, "error": "接口不存在"}, HTTPStatus.NOT_FOUND)
        except (ValueError, RuntimeError, TimeoutError, json.JSONDecodeError) as error:
            self.station.add_event("error", str(error), "api")
            self._json({"ok": False, "error": str(error)}, HTTPStatus.BAD_REQUEST)

    def _serve_static(self, url_path: str) -> None:
        relative = "index.html" if url_path in ("", "/") else url_path.lstrip("/")
        target = (STATIC_ROOT / relative).resolve()
        if STATIC_ROOT.resolve() not in target.parents and target != STATIC_ROOT.resolve():
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_types = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".svg": "image/svg+xml",
        }
        body = target.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_types.get(target.suffix, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)


class ControlStationServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], station: ControlStation):
        self.station = station
        super().__init__(address, RequestHandler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="UART7 serial device; select in the web app if omitted")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--history-seconds", type=int, default=600)
    parser.add_argument("--odrive-rate", type=float, default=20.0)
    parser.add_argument("--open-browser", action="store_true", help="open the dashboard after startup")
    args = parser.parse_args()
    if args.odrive_rate <= 0 or args.odrive_rate > 100:
        parser.error("--odrive-rate must be in (0, 100]")
    station = ControlStation(args.port, args.history_seconds, args.odrive_rate)
    station.start()
    server = ControlStationServer((args.host, args.http_port), station)

    def stop_handler(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    print(f"Bike Control Station: http://{args.host}:{args.http_port}")
    print(f"UART7: {args.port or 'select in dashboard'}; ODrive polling: {args.odrive_rate:g} Hz")
    if args.open_browser or getattr(sys, "frozen", False):
        threading.Timer(0.5, webbrowser.open, args=(f"http://{args.host}:{args.http_port}",)).start()
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        station.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
