#!/usr/bin/env python3
"""Summarize a completed PID sweep once, without making per-trial decisions."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


MINIMIZE = (
    "roll_error_rms_deg",
    "rate_error_rms_dps",
    "wheel_rms_tps",
    "accel_rms_tps2",
)


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def percentile_abs(values: list[float], probability: float) -> float:
    ordered = sorted(abs(value) for value in values)
    index = min(len(ordered) - 1, math.ceil(probability * len(ordered)) - 1)
    return ordered[index]


def trial_metrics(rows: list[dict[str, str]]) -> dict[str, float | int]:
    roll_error = [float(row["roll_deg"]) - float(row["zero_control_deg"]) for row in rows]
    rate = [float(row["roll_rate_dps"]) for row in rows]
    rate_error = [float(row["rate_error_dps"]) for row in rows]
    wheel = [float(row["wheel_tps"]) for row in rows]
    accel = [float(row["accel_cmd_tps2"]) for row in rows]
    flags = [int(row["flags"], 0) for row in rows]
    return {
        "samples": len(rows),
        "roll_error_mean_deg": statistics.fmean(roll_error),
        "roll_error_rms_deg": rms(roll_error),
        "roll_error_p95_abs_deg": percentile_abs(roll_error, 0.95),
        "roll_error_max_abs_deg": max(map(abs, roll_error)),
        "roll_rate_rms_dps": rms(rate),
        "rate_error_mean_dps": statistics.fmean(rate_error),
        "rate_error_rms_dps": rms(rate_error),
        "wheel_mean_tps": statistics.fmean(wheel),
        "wheel_rms_tps": rms(wheel),
        "wheel_max_abs_tps": max(map(abs, wheel)),
        "accel_rms_tps2": rms(accel),
        "accel_p95_abs_tps2": percentile_abs(accel, 0.95),
        "accel_max_abs_tps2": max(map(abs, accel)),
        "accel_sat_pct": 100.0 * sum(bool(flag & (1 << 4)) for flag in flags) / len(flags),
        "velocity_sat_pct": 100.0 * sum(bool(flag & (1 << 3)) for flag in flags) / len(flags),
        "fault_samples": sum(bool(flag & ((1 << 15) | (1 << 16))) for flag in flags),
    }


def median_metrics(items: list[dict[str, float | int]]) -> dict[str, float]:
    result: dict[str, float] = {}
    for name in items[0]:
        if name not in {"trial", "kind", "requested_value", "samples"}:
            result[name] = statistics.median(float(item[name]) for item in items)
    for name in MINIMIZE:
        center = result[name]
        result[name + "_mad"] = statistics.median(
            abs(float(item[name]) - center) for item in items
        )
    return result


def pareto_values(items: list[dict[str, float]]) -> set[float]:
    frontier: set[float] = set()
    for candidate in items:
        dominated = False
        for other in items:
            if other is candidate:
                continue
            no_worse = all(other[name] <= candidate[name] for name in MINIMIZE)
            strictly_better = any(other[name] < candidate[name] for name in MINIMIZE)
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            frontier.add(candidate["requested_value"])
    return frontier


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    manifest_path = args.csv.with_suffix(".json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "COMPLETE":
        parser.error(f"sweep is not complete: {manifest.get('status')}")
    anchor_trials = {int(value) for value in manifest.get("anchor_trials", [])}

    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    with args.csv.open(newline="", encoding="utf-8") as input_file:
        for row in csv.DictReader(input_file):
            grouped[int(row["trial"])].append(row)

    per_trial: list[dict[str, float | int | str]] = []
    for trial in sorted(grouped):
        rows = grouped[trial]
        requested = float(rows[0]["requested_value"])
        per_trial.append({
            "trial": trial,
            "kind": "anchor" if trial in anchor_trials else "candidate",
            "requested_value": requested,
            **trial_metrics(rows),
        })

    by_value: dict[float, list[dict[str, float | int]]] = defaultdict(list)
    for item in per_trial:
        if item["kind"] == "candidate":
            by_value[float(item["requested_value"])].append(item)

    summary: list[dict[str, float]] = []
    for value in sorted(by_value):
        aggregate = median_metrics(by_value[value])
        aggregate["requested_value"] = value
        aggregate["trials"] = float(len(by_value[value]))
        summary.append(aggregate)

    safe = [
        item for item in summary
        if item["fault_samples"] == 0.0
        and item["accel_sat_pct"] == 0.0
        and item["velocity_sat_pct"] == 0.0
    ]
    frontier = pareto_values(safe)
    for item in summary:
        item["pareto"] = 1.0 if item["requested_value"] in frontier else 0.0

    output_path = args.output or args.csv.with_name(args.csv.stem + "-summary.csv")
    fieldnames = ["requested_value", "trials", "pareto"]
    fieldnames.extend(
        name for name in summary[0]
        if name not in {"requested_value", "trials", "pareto"}
    )
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(summary)

    anchors = [item for item in per_trial if item["kind"] == "anchor"]
    report = {
        "parameter": manifest["parameter"],
        "candidate_values": len(summary),
        "candidate_trials": sum(len(items) for items in by_value.values()),
        "anchor_trials": len(anchors),
        "pareto_values": sorted(frontier),
        "anchor_first_roll_rms": anchors[0]["roll_error_rms_deg"] if anchors else None,
        "anchor_last_roll_rms": anchors[-1]["roll_error_rms_deg"] if anchors else None,
        "summary_csv": str(output_path),
    }
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
