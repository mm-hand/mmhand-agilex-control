from __future__ import annotations

import math

import pytest

from config import ControlConfig
from settings_store import apply_settings, load_settings, reset_settings, save_settings, settings_data


def test_apply_settings_overrides_startup_and_frames():
    config = ControlConfig()
    data = settings_data(config)
    data["startup"]["left_pose_rad"][0] = 0.5
    data["frames"]["translation_horizon_to_left_shoulder_m"][0] = -0.2
    updated = apply_settings(config, data)
    assert updated.startup.left_pose_rad[0] == 0.5
    assert updated.frames.translation_horizon_to_left_shoulder_m[0] == -0.2
    assert updated.startup.right_pose_rad == config.startup.right_pose_rad


def test_save_and_load_roundtrip(tmp_path):
    path = tmp_path / "settings.json"
    data = settings_data(ControlConfig())
    data["startup"]["right_pose_rad"][3] = 1.0
    save_settings(data, path)
    loaded, error = load_settings(path)
    assert error is None
    assert loaded["startup"]["right_pose_rad"][3] == 1.0


def test_invalid_json_returns_error(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text("{not json", encoding="utf-8")
    data, error = load_settings(path)
    assert data == {}
    assert error is not None


def test_invalid_settings_are_rejected(tmp_path):
    path = tmp_path / "settings.json"
    cases = [
        lambda data: data["startup"]["left_pose_rad"].pop(),
        lambda data: data["startup"]["left_pose_rad"].__setitem__(0, math.nan),
        lambda data: data["startup"]["left_pose_rad"].__setitem__(0, 100.0),
        lambda data: data["frames"]["translation_horizon_to_left_shoulder_m"].pop(),
    ]
    for mutate in cases:
        data = settings_data(ControlConfig())
        mutate(data)
        with pytest.raises(ValueError):
            apply_settings(ControlConfig(), data)
        with pytest.raises(ValueError):
            save_settings(data, path)
        assert not path.exists()


def test_reset_settings_removes_override(tmp_path):
    path = tmp_path / "settings.json"
    save_settings(settings_data(ControlConfig()), path)
    assert path.exists()
    reset_settings(path)
    assert not path.exists()
