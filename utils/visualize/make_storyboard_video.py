from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import cv2
import numpy as np
import pandas as pd
try:
    import imageio.v3 as iio
except Exception:  # pragma: no cover - optional dependency
    iio = None


def load_and_resize(path: Path, size: Tuple[int, int], label: str) -> np.ndarray:
    """Load an image; if missing, create a placeholder with a label."""
    img = None
    if path and path.is_file():
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        img = np.full((size[1], size[0], 3), 32, dtype=np.uint8)
        cv2.putText(img, f"missing: {label}", (10, size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        return img
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA)


def compose_frame(
    images: Dict[str, Path],
    plots: Dict[str, Path],
    size: Tuple[int, int],
    wrist_key: str,
    fixed_key: str,
) -> np.ndarray:
    """Make a 2x2 grid (wrist, fixed, left plot, right plot)."""
    cell_w, cell_h = size
    wrist = load_and_resize(images.get(wrist_key), (cell_w, cell_h), f"wrist image ({wrist_key})")
    fixed = load_and_resize(images.get(fixed_key), (cell_w, cell_h), f"fixed image ({fixed_key})")
    left_plot = load_and_resize(plots.get("left"), (cell_w, cell_h), "left ft plot")
    right_plot = load_and_resize(plots.get("right"), (cell_w, cell_h), "right ft plot")

    top = np.hstack([wrist, fixed])
    bottom = np.hstack([left_plot, right_plot])
    return np.vstack([top, bottom])


def collect_records(
    inputs_csv: Path,
    input_dir: Path,
    plots_dir: Path,
    wrist_key: str,
    fixed_key: str,
) -> Iterable[Tuple[str, Dict[str, Path], Dict[str, Path]]]:
    """Yield (inference_id, images, plots) ordered by time."""
    df = pd.read_csv(inputs_csv)
    if "timestamp_sec" in df.columns:
        df = df.sort_values("timestamp_sec")
    image_cols = [c for c in df.columns if c.startswith("image_")]

    for _, row in df.iterrows():
        inference_id = str(row.get("inference_id")) if "inference_id" in row else None
        if not inference_id:
            continue

        images: Dict[str, Path] = {}
        for col in image_cols:
            val = row.get(col)
            if pd.isna(val):
                continue
            key = col.removeprefix("image_")
            images[key] = input_dir / str(val)

        # If wrist/fixed keys are missing, attempt common fallbacks.
        if wrist_key not in images:
            for fallback in ("cam_left_wrist", "wrist"):
                if fallback in images:
                    images[wrist_key] = images[fallback]
                    break
        if fixed_key not in images:
            for fallback in ("cam_high", "fixed"):
                if fallback in images:
                    images[fixed_key] = images[fallback]
                    break

        plots: Dict[str, Path] = {
            "left": plots_dir / f"{inference_id}_left_ft.png",
            "right": plots_dir / f"{inference_id}_right_ft.png",
        }

        yield inference_id, images, plots


def build_storyboards(
    input_dir: Path,
    plots_dir: Path,
    output_dir: Path,
    cell_size: Tuple[int, int],
    wrist_key: str,
    fixed_key: str,
) -> List[Path]:
    inputs_csv = input_dir / "inputs.csv"
    if not inputs_csv.is_file():
        raise FileNotFoundError(f"inputs.csv not found in {input_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    frames: List[Path] = []
    for idx, (inference_id, images, plots) in enumerate(
        collect_records(inputs_csv, input_dir, plots_dir, wrist_key, fixed_key)
    ):
        frame = compose_frame(images, plots, cell_size, wrist_key, fixed_key)
        out_path = output_dir / f"{idx:05d}_{inference_id}.png"
        cv2.imwrite(str(out_path), frame)
        frames.append(out_path)
    return frames


def write_video(frames: List[Path], output_path: Path, fps: int) -> None:
    """Write frames to an MP4 using imageio+ffmpeg; fallback to OpenCV."""
    if not frames:
        raise ValueError("No frames to write")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    images: List[np.ndarray] = []
    target_size: Tuple[int, int] | None = None
    for path in frames:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            continue
        if target_size is None:
            target_size = (img.shape[1], img.shape[0])
        if (img.shape[1], img.shape[0]) != target_size:
            img = cv2.resize(img, target_size, interpolation=cv2.INTER_AREA)
        images.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    if not images:
        raise ValueError("No readable frames to write")

    # Prefer imageio/ffmpeg for reliable MP4; fallback to OpenCV if unavailable.
    if iio is not None:
        iio.imwrite(output_path, images, fps=fps, codec="libx264", quality=8)
        return

    height, width = images[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open video writer for {output_path}")
    for rgb in images:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="Build collages and a 1fps video from inference logs.")
    parser.add_argument("--input_dir", type=Path, default=Path(__file__).resolve().parent / "inference_inputs")
    parser.add_argument("--plots_dir", type=Path, default=None, help="Directory containing *_ft.png plots (default: input_dir)")
    parser.add_argument("--output_dir", type=Path, default=Path(__file__).resolve().parent / "storyboards")
    parser.add_argument("--video", type=Path, default=None, help="Output video path (default: output_dir/storyboard.mp4)")
    parser.add_argument("--fps", type=int, default=1, help="Frames per second for the video")
    parser.add_argument("--cell_width", type=int, default=640, help="Cell width for each panel")
    parser.add_argument("--cell_height", type=int, default=480, help="Cell height for each panel")
    parser.add_argument("--wrist_key", type=str, default="cam_left_wrist", help="Key name for wrist image column")
    parser.add_argument("--fixed_key", type=str, default="cam_high", help="Key name for fixed image column")
    args = parser.parse_args()

    plots_dir = args.plots_dir or args.input_dir
    video_path = args.video or (args.output_dir / "storyboard.mp4")

    frames = build_storyboards(
        args.input_dir,
        plots_dir,
        args.output_dir,
        (args.cell_width, args.cell_height),
        args.wrist_key,
        args.fixed_key,
    )
    print(f"Wrote {len(frames)} storyboard frames to {args.output_dir}")

    write_video(frames, video_path, args.fps)
    print(f"Wrote video -> {video_path}")


if __name__ == "__main__":
    main()
