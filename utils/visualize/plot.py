from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

# Force non-interactive backend to avoid GUI issues in headless runs.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def plot_ft_timeseries(csv_path: Path, output_path: Path) -> None:
    """Plot force/torque time series from a per-inference CSV and save PNG."""
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    cols = ["step", "fx", "fy", "fz", "tx", "ty", "tz"]
    df = pd.read_csv(csv_path, usecols=cols, dtype=float, low_memory=False)

    fig, (ax_f, ax_t) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    df.plot(x="step", y=["fx", "fy", "fz"], ax=ax_f)
    ax_f.set_ylabel("force")
    ax_f.grid(True, linewidth=0.4, linestyle="--")

    df.plot(x="step", y=["tx", "ty", "tz"], ax=ax_t)
    ax_t.set_xlabel("step")
    ax_t.set_ylabel("torque")
    ax_t.grid(True, linewidth=0.4, linestyle="--")

    fig.suptitle(csv_path.name)
    plt.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def plot_directory(input_dir: Path, output_dir: Path | None = None) -> list[Path]:
    """Plot all *_left_ft.csv and *_right_ft.csv files in a directory."""
    if not input_dir.exists():
        raise FileNotFoundError(f"Input dir not found: {input_dir}")

    target_dir = output_dir or input_dir
    target_dir.mkdir(parents=True, exist_ok=True)

    generated: list[Path] = []
    for csv_path in sorted(input_dir.glob("*_ft.csv")):
        out_path = target_dir / f"{csv_path.stem}.png"
        plot_ft_timeseries(csv_path, out_path)
        generated.append(out_path)
    return generated


def main() -> None:
    parser = argparse.ArgumentParser(
        description=("Plot all *_ft.csv files in a directory produced by inference logging.")
    )
    parser.add_argument(
        "--input_dir",
        type=Path,
        default=Path(__file__).resolve().parent / "inference_inputs",
        help="Directory containing *_ft.csv files (default: data/inference_inputs).",
    )
    parser.add_argument(
        "--output_dir",
        type=Path,
        default=None,
        help="Optional directory to write plots (defaults to input_dir).",
    )
    args = parser.parse_args()

    generated = plot_directory(args.input_dir, args.output_dir)
    print(f"Wrote {len(generated)} plots:")
    for path in generated:
        print(f"  {path}")


if __name__ == "__main__":
    main()
