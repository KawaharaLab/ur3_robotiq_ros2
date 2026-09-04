#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def robust_threshold(values: np.ndarray, multiplier: float = 12.0) -> float:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.inf
    median = np.median(finite)
    mad = np.median(np.abs(finite - median))
    return float(median + multiplier * max(mad, 1e-9))


def analyze(csv_path: Path, label: str, output_dir: Path) -> dict:
    df = pd.read_csv(csv_path)
    if len(df) < 3:
        raise RuntimeError(f"Not enough rows in {csv_path}")

    t = df["header_time_ns"].to_numpy(dtype=np.int64) / 1e9
    fy = df["force_y"].to_numpy(dtype=float)

    dt = np.diff(t)
    diff = np.abs(np.diff(fy))

    positive_dt = dt[dt > 0]
    median_dt = float(np.median(positive_dt)) if positive_dt.size else np.nan
    effective_hz = 1.0 / median_dt if median_dt > 0 else np.nan

    threshold = robust_threshold(diff)
    candidate_indices = np.flatnonzero(diff > threshold)

    candidate_df = pd.DataFrame({
        "index_before": candidate_indices,
        "time_s": t[candidate_indices + 1] - t[0],
        "force_y_before": fy[candidate_indices],
        "force_y_after": fy[candidate_indices + 1],
        "abs_step": diff[candidate_indices],
        "threshold": threshold,
    })
    candidate_df.to_csv(
        output_dir / f"{label}_noise_candidates.csv",
        index=False,
    )

    plt.figure(figsize=(14, 5))
    plt.plot(t - t[0], fy, linewidth=0.7)
    if candidate_indices.size:
        plt.scatter(
            t[candidate_indices + 1] - t[0],
            fy[candidate_indices + 1],
            s=12,
        )
    plt.xlabel("Elapsed time [s]")
    plt.ylabel("Force Y [N]")
    plt.title(f"{label}: Force Y")
    plt.tight_layout()
    plt.savefig(output_dir / f"{label}_force_y.png", dpi=180)
    plt.close()

    return {
        "sensor": label,
        "rows": len(df),
        "duration_s": float(t[-1] - t[0]),
        "effective_hz_from_header_stamp": effective_hz,
        "median_dt_ms": median_dt * 1000.0,
        "p95_dt_ms": float(np.percentile(positive_dt, 95) * 1000.0)
        if positive_dt.size else np.nan,
        "max_dt_ms": float(np.max(positive_dt) * 1000.0)
        if positive_dt.size else np.nan,
        "nonpositive_dt_count": int(np.sum(dt <= 0)),
        "max_abs_force_y_step": float(np.max(diff)),
        "noise_threshold": threshold,
        "candidate_count": int(candidate_indices.size),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze left/right MMS101 CSV timing and Force-Y spikes."
    )
    parser.add_argument("csv_dir", help="Directory containing left_force.csv and right_force.csv")
    args = parser.parse_args()

    csv_dir = Path(args.csv_dir).expanduser().resolve()
    output_dir = csv_dir / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    results = [
        analyze(csv_dir / "left_force.csv", "left", output_dir),
        analyze(csv_dir / "right_force.csv", "right", output_dir),
    ]

    summary = pd.DataFrame(results)
    summary.to_csv(output_dir / "summary.csv", index=False)
    print(summary.to_string(index=False))
    print(f"\nOutputs: {output_dir}")
    
    output_dir = csv_dir / "analysis" / "zoom"
    output_dir.mkdir(parents=True, exist_ok=True)

    right = pd.read_csv(csv_dir / "right_force.csv")
    left = pd.read_csv(csv_dir / "left_force.csv")
    
    right_t = (
        right["header_time_ns"] - right["header_time_ns"].iloc[0]
    ) / 1e9
    left_t = (
        left["header_time_ns"] - left["header_time_ns"].iloc[0]
    ) / 1e9
    
    # right側で最大変化が大きかった時刻
    target_times = [
        160.501298,
        196.824654,
        209.856755,
        322.823398,
        358.811772,
        383.545614,
        426.436182,
        430.466420,
    ]

    window_s = 0.1

    for target in target_times:
        right_mask = (
            (right_t >= target - window_s) &
            (right_t <= target + window_s)
        )
        left_mask = (
            (left_t >= target - window_s) &
            (left_t <= target + window_s)
        )

        plt.figure(figsize=(12, 5))

        plt.plot(
            right_t[right_mask] - target,
            right.loc[right_mask, "force_y"],
            label="right Force Y",
            linewidth=1.0,
        )

        plt.plot(
            left_t[left_mask] - target,
            left.loc[left_mask, "force_y"],
            label="left Force Y",
            linewidth=1.0,
        )

        plt.axvline(0.0, linestyle="--", linewidth=0.8)

        plt.xlabel("Time from candidate [s]")
        plt.ylabel("Force Y [N]")
        plt.title(f"Candidate around {target:.3f} s")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        filename = output_dir / f"zoom_{target:.3f}s.png"
        plt.savefig(filename, dpi=180)
        plt.close()

    print(f"Saved zoom plots to: {output_dir}")


if __name__ == "__main__":
    main()
