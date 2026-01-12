from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

# Force non-interactive backend to avoid GUI issues in headless runs.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


# Columns to plot from each CSV (edit here as needed)
SCALED_COLS = [
    # "scaled_joint_trajectory_controller_controller_state.feedback.positions[3]",
    "scaled_joint_trajectory_controller_controller_state.reference.positions[3]",
]

JOINT_COLS = [
    "joint_states.position[2]",
]

# Optional plotting window (stamp_ns range). Set to None to plot all data.
# Example: STAMP_RANGE = (1.76656105042e18, 1.76656105062e18)
STAMP_RANGE: tuple[float, float] | None = (1766561178398760503, 1766561186622688662)


def _load_subset(path: Path, cols: list[str]) -> pd.DataFrame:
    usecols = ["stamp_ns", *cols]
    dtype_map = {c: float for c in usecols}
    df = pd.read_csv(path, usecols=usecols, dtype=dtype_map, low_memory=False)
    return df.dropna(subset=usecols)


def plot_joint_series(csv_dir: Path, output_path: Path | None) -> None:
    scaled_csv = csv_dir / "scaled_joint_trajectory_controller_controller_state.csv"
    joint_csv = csv_dir / "joint_states.csv"

    if not scaled_csv.exists():
        raise FileNotFoundError(f"CSV not found: {scaled_csv}")
    if not joint_csv.exists():
        raise FileNotFoundError(f"CSV not found: {joint_csv}")

    df_scaled = _load_subset(scaled_csv, SCALED_COLS)
    df_joint = _load_subset(joint_csv, JOINT_COLS)

    # Apply optional stamp_ns window if specified
    if STAMP_RANGE is not None:
        lo, hi = STAMP_RANGE
        df_scaled = df_scaled[(df_scaled["stamp_ns"] >= lo) & (df_scaled["stamp_ns"] <= hi)]
        df_joint = df_joint[(df_joint["stamp_ns"] >= lo) & (df_joint["stamp_ns"] <= hi)]

    fig, ax = plt.subplots(figsize=(10, 5))

    for col in SCALED_COLS:
        ax.plot(df_scaled["stamp_ns"], df_scaled[col], label=f"scaled: {col}")

    for col in JOINT_COLS:
        ax.plot(df_joint["stamp_ns"], df_joint[col], label=f"joint: {col}", linestyle="--")

    ax.set_xlabel("stamp_ns")
    ax.set_ylabel("value")
    ax.set_title("Scaled controller vs joint_states")
    ax.grid(True, linewidth=0.4, linestyle="--")
    ax.legend()
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Plot selected columns from scaled_joint_trajectory_controller_controller_state.csv "
            "and joint_states.csv using stamp_ns as the shared x-axis."
        )
    )
    parser.add_argument(
        "--csv_dir",
        type=Path,
        required=True,
        help="Directory containing the CSV files (scaled_joint_trajectory_controller_controller_state.csv and joint_states.csv).",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Optional path to save the plot; defaults to csv_dir/joints.png",
    )
    args = parser.parse_args()

    output_path = args.output or (args.csv_dir / "joints.png")
    plot_joint_series(args.csv_dir, output_path)


if __name__ == "__main__":
    main()
