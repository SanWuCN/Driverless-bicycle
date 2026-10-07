#!/usr/bin/env python3
"""Run a randomized, safety-bounded UART PID sweep without per-trial tuning decisions."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import select
import sys
import termios
import time
from pathlib import Path

try:
    import balance_telemetry as bt
except ModuleNotFoundError:  # Also support `python -m tools.pid_batch_sweep`.
    from tools import balance_telemetry as bt


EXTRA_FIELDS = ("trial", "requested_value", "phase", "host_elapsed_s")


def values_inclusive(start: float, stop: float, step: float) -> list[float]:
    if step <= 0.0:
        raise ValueError("step must be positive")
    direction = 1.0 if stop >= start else -1.0
    count = int(round(abs(stop - start) / step))
    values = [start + direction * step * index for index in range(count + 1)]
    if not math.isclose(values[-1], stop, rel_tol=0.0, abs_tol=step * 1e-6):
        raise ValueError("range must be an integer number of steps")
    return [round(value, 9) for value in values]


def build_order(
    values: list[float], repeat: int, anchor: float, anchor_every: int, seed: int
) -> tuple[list[float], list[int]]:
    rng = random.Random(seed)
    candidates = values * repeat
    rng.shuffle(candidates)
    order: list[float] = []
    anchor_trials: list[int] = []
    for index, value in enumerate(candidates, 1):
        order.append(value)
        if anchor_every > 0 and index % anchor_every == 0:
            order.append(anchor)
            anchor_trials.append(len(order))
    if not anchor_trials or anchor_trials[-1] != len(order):
        order.append(anchor)
        anchor_trials.append(len(order))
    return order, anchor_trials


class SerialSession:
    def __init__(self, port: str):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        bt.configure_115200(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.decoder = bt.Uart7StreamDecoder()

    def close(self) -> None:
        os.close(self.fd)

    def lines(self, timeout_s: float) -> list[str]:
        return [
            str(payload)
            for event_type, payload in self.events(timeout_s)
            if event_type == "line"
        ]

    def events(self, timeout_s: float) -> list[tuple[str, object]]:
        readable, _, _ = select.select([self.fd], [], [], timeout_s)
        if not readable:
            return []
        return self.decoder.feed(os.read(self.fd, 4096))

    def set_parameter(self, name: str, value: float, timeout_s: float = 2.0) -> str:
        command = bt.tuning_command("SET", name, f"{value:.9g}")
        for _ in range(3):
            os.write(self.fd, command)
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                for line in self.lines(0.1):
                    if line.startswith("#P,"):
                        if ",OK,SET," in line:
                            return line
                        raise RuntimeError(f"firmware rejected command: {line}")
        raise TimeoutError(f"no SET acknowledgement for {name}={value}")


def safety_reason(sample: bt.Sample, max_roll: float, max_rate: float, max_wheel: float) -> str | None:
    roll_error = sample.roll_deg - sample.zero_control_deg
    if abs(roll_error) > max_roll:
        return f"roll error {roll_error:+.3f} deg exceeds {max_roll}"
    if abs(sample.roll_rate_dps) > max_rate:
        return f"roll rate {sample.roll_rate_dps:+.3f} dps exceeds {max_rate}"
    if abs(sample.wheel_tps) > max_wheel:
        return f"wheel speed {sample.wheel_tps:+.3f} tps exceeds {max_wheel}"
    if not sample.flags & (1 << 7):
        return "balance control disarmed"
    if sample.flags & (1 << 15):
        return "ODrive feedback timeout"
    if sample.flags & (1 << 16):
        return "ODrive fault"
    if sample.flags & (1 << 18):
        return "UART7 RX overflow"
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--parameter", required=True, choices=sorted(bt.TUNABLE_PARAMETERS))
    parser.add_argument("--start", required=True, type=float)
    parser.add_argument("--stop", required=True, type=float)
    parser.add_argument("--step", required=True, type=float)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--settle", type=float, default=5.0)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--anchor", required=True, type=float)
    parser.add_argument("--anchor-every", type=int, default=5)
    parser.add_argument("--restore", required=True, type=float)
    parser.add_argument("--seed", type=int, default=20261006)
    parser.add_argument("--max-roll", type=float, default=1.5)
    parser.add_argument("--max-rate", type=float, default=8.0)
    parser.add_argument("--max-wheel", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.duration <= 0 or args.settle < 0 or args.repeat <= 0:
        parser.error("duration/repeat must be positive and settle must be non-negative")
    values = values_inclusive(args.start, args.stop, args.step)
    order, anchor_trials = build_order(
        values, args.repeat, args.anchor, args.anchor_every, args.seed
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output.with_suffix(".json")
    manifest = {
        "parameter": args.parameter,
        "values": values,
        "order": order,
        "anchor_trials": anchor_trials,
        "duration_s": args.duration,
        "settle_s": args.settle,
        "seed": args.seed,
        "anchor": args.anchor,
        "restore": args.restore,
        "status": "RUNNING",
        "completed_trials": 0,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    session = SerialSession(args.port)
    started = time.monotonic()
    abort_reason: str | None = None
    try:
        with args.output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.writer(output_file)
            writer.writerow((*EXTRA_FIELDS, *bt.CSV_FIELDS))
            for trial, requested in enumerate(order, 1):
                acknowledgement = session.set_parameter(args.parameter, requested)
                print(f"trial {trial:03d}/{len(order):03d} {args.parameter}={requested:.9g} {acknowledgement}")
                trial_started = time.monotonic()
                last_sample_at = trial_started
                recorded = 0
                while time.monotonic() - trial_started < args.settle + args.duration:
                    events = session.events(0.2)
                    now = time.monotonic()
                    for event_type, payload in events:
                        sample = payload if event_type == "balance" else None
                        if event_type == "line":
                            try:
                                sample = bt.parse_sample(str(payload))
                            except (ValueError, OverflowError):
                                continue
                        if sample is None:
                            continue
                        last_sample_at = now
                        reason = safety_reason(sample, args.max_roll, args.max_rate, args.max_wheel)
                        if reason is not None:
                            abort_reason = f"trial {trial}: {reason}"
                            break
                        elapsed = now - trial_started
                        phase = "settle" if elapsed < args.settle else "record"
                        if phase == "record":
                            writer.writerow((trial, requested, phase, now - started, *sample.csv_row()))
                            recorded += 1
                    if abort_reason is not None:
                        break
                    if now - last_sample_at > 1.0:
                        abort_reason = f"trial {trial}: telemetry absent for more than 1 s"
                        break
                output_file.flush()
                if abort_reason is not None:
                    break
                if recorded < max(1, int(args.duration * 15)):
                    abort_reason = f"trial {trial}: only {recorded} recorded samples"
                    break
                manifest["completed_trials"] = trial
                manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    except KeyboardInterrupt:
        abort_reason = "operator interrupted sweep"
    except Exception as error:
        abort_reason = f"{type(error).__name__}: {error}"
    finally:
        try:
            restore_reply = session.set_parameter(args.parameter, args.restore)
            manifest["restore_reply"] = restore_reply
        except Exception as error:  # Keep the original failure visible as well.
            manifest["restore_error"] = str(error)
        session.close()

    manifest["status"] = "ABORTED" if abort_reason else "COMPLETE"
    manifest["abort_reason"] = abort_reason
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 2 if abort_reason else 0


if __name__ == "__main__":
    raise SystemExit(main())
