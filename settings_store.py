from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

from config import ControlConfig
from ik_solver import NeroParams

SETTINGS_PATH = Path(__file__).resolve().parent / "settings.json"
STARTUP_FIELDS = ("left_pose_rad", "right_pose_rad")
FRAME_FIELDS = (
    "translation_horizon_to_left_shoulder_m",
    "translation_horizon_to_right_shoulder_m",
    "translation_left_shoulder_to_left_arm_base_m",
    "rotation_left_shoulder_to_left_arm_base_rpy_deg",
    "translation_right_shoulder_to_right_arm_base_m",
    "rotation_right_shoulder_to_right_arm_base_rpy_deg",
)


def load_settings(path: Path = SETTINGS_PATH) -> tuple[dict, str | None]:
    if not path.is_file():
        return {}, None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {}, f"invalid settings.json: {exc}"
    if not isinstance(data, dict):
        return {}, "invalid settings.json: root must be an object"
    return data, None


def apply_settings(config: ControlConfig, data: dict) -> ControlConfig:
    startup_data = data.get("startup", {})
    frame_data = data.get("frames", {})
    if not isinstance(startup_data, dict) or not isinstance(frame_data, dict):
        raise ValueError("settings startup and frames must be objects")

    startup = config.startup
    frames = config.frames
    limits = NeroParams.default().joint_limits

    for field in STARTUP_FIELDS:
        if field not in startup_data:
            continue
        values = _floats(startup_data[field], 7, field)
        for index, value in enumerate(values):
            if value < float(limits[index, 0]) or value > float(limits[index, 1]):
                raise ValueError(f"{field} joint {index + 1} is outside the Nero joint limits")
        startup = replace(startup, **{field: tuple(values)})

    for field in FRAME_FIELDS:
        if field not in frame_data:
            continue
        frames = replace(frames, **{field: tuple(_floats(frame_data[field], 3, field))})

    return replace(config, startup=startup, frames=frames)


def save_settings(data: dict, path: Path = SETTINGS_PATH) -> None:
    apply_settings(ControlConfig(), data)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def settings_data(config: ControlConfig) -> dict:
    return {
        "startup": {field: list(getattr(config.startup, field)) for field in STARTUP_FIELDS},
        "frames": {field: list(getattr(config.frames, field)) for field in FRAME_FIELDS},
    }


def reset_settings(path: Path = SETTINGS_PATH) -> None:
    path.unlink(missing_ok=True)


def _floats(value, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{name} must contain only numbers")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{name} must contain finite numbers")
        result.append(number)
    return result
