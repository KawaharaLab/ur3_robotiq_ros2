"""Configuration helpers for the GelSight ROS 2 package."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


@dataclass
class ConfigModel:
    """Strongly-typed representation of the GelSight config file."""

    camera_width: int = 640
    camera_height: int = 480
    border_fraction: float = 0.1
    default_camera_index: Optional[int] = 0


class GSConfig:
    """Loads and normalises GelSight configuration files."""

    def __init__(self, config_path: str) -> None:
        self.path = self._resolve_path(config_path)
        self.config = self._load()

    def _resolve_path(self, config_path: str) -> Path:
        """Resolve the config path, probing common locations."""

        user_path = Path(config_path)
        candidates: list[Path] = []

        # Absolute path supplied by the user
        if user_path.is_absolute():
            candidates.append(user_path)
        else:
            # Relative to the current working directory
            candidates.append((Path.cwd() / user_path).resolve())
            # Relative to the package's python module directory
            package_root = Path(__file__).resolve().parent
            candidates.append(package_root / user_path)
            # Relative to the repository root (one level up from the python package)
            candidates.append(package_root.parent / user_path)
            # Relative to the top-level config directory in the source tree
            candidates.append(package_root.parent / "config" / user_path.name)

        # Finally, try the installed share directory
        try:
            share_dir = Path(get_package_share_directory("ros2_gelsight_package"))
            candidates.append(share_dir / user_path)
            candidates.append(share_dir / "config" / user_path.name)
        except PackageNotFoundError:
            pass

        for candidate in candidates:
            if candidate.is_file():
                return candidate

        searched = "\n - ".join(str(p) for p in candidates)
        raise FileNotFoundError(
            f"Could not locate GelSight config file '{config_path}'.\nSearched paths:\n - {searched}"
        )

    def _load(self) -> ConfigModel:
        defaults = ConfigModel()
        with self.path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)

        return ConfigModel(
            camera_width=int(data.get("camera_width", defaults.camera_width)),
            camera_height=int(data.get("camera_height", defaults.camera_height)),
            border_fraction=float(data.get("border_fraction", defaults.border_fraction)),
            default_camera_index=data.get(
                "default_camera_index", defaults.default_camera_index
            ),
        )

    def as_dict(self) -> dict[str, int | float | Optional[int]]:
        return {
            "camera_width": self.config.camera_width,
            "camera_height": self.config.camera_height,
            "border_fraction": self.config.border_fraction,
            "default_camera_index": self.config.default_camera_index,
        }
