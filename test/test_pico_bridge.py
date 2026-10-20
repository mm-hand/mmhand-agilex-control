from __future__ import annotations

import logging
from types import SimpleNamespace

from config import PicoConfig
from pico_bridge.bridge import (
    PicoBridge,
    adb_remove_reverse,
    adb_setup,
    find_adb,
    hand_dialog_focused,
    restore_focus,
)


def test_find_adb_uses_configured_path(tmp_path):
    adb = tmp_path / "adb.exe"
    adb.write_text("", encoding="utf-8")
    assert find_adb(PicoConfig(adb_path=str(adb))) == str(adb)


def test_adb_setup_runs_reverse_and_launches_package(tmp_path, monkeypatch):
    adb = tmp_path / "adb.exe"
    adb.write_text("", encoding="utf-8")
    calls = []
    monkeypatch.setattr(
        "pico_bridge.bridge.subprocess.run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(returncode=0),
    )
    config = PicoConfig(adb_path=str(adb), device_port=29761, vr_port=9100)
    adb_setup(config)
    assert calls[0] == [str(adb), "get-state"]
    assert calls[1] == [str(adb), "reverse", "tcp:29761", "tcp:9100"]
    assert calls[2][:5] == [str(adb), "shell", "monkey", "-p", config.package]


def test_adb_remove_reverse_is_best_effort(tmp_path, monkeypatch):
    adb = tmp_path / "adb.exe"
    adb.write_text("", encoding="utf-8")
    calls = []
    monkeypatch.setattr(
        "pico_bridge.bridge.subprocess.run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(returncode=0),
    )
    adb_remove_reverse(PicoConfig(adb_path=str(adb)))
    assert calls == [[str(adb), "reverse", "--remove", "tcp:29761"]]


def test_status_reports_disabled_adb():
    bridge = PicoBridge(PicoConfig(), logging.getLogger("test"), no_adb=True)
    status = bridge._status()
    assert status["adb"] == "disabled"
    assert status["rate_hz"] == 0.0


def test_hand_dialog_focused_detects_system_dialog():
    text = (
        "  mCurrentFocus=Window{6502072 u0 com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity}\n"
        "  mFocusedApp=ActivityRecord{55a3138 u0 com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity t14}\n"
    )
    assert hand_dialog_focused(text)


def test_hand_dialog_focused_ignores_app_and_home():
    text = (
        "  mCurrentFocus=Window{6708500 u0 com.mmhand.pico_head_relative_pose/..."
        "com.mmhand.pico_head_relative_pose.MainActivity}\n"
        "  mFocusedApp=ActivityRecord{b5e96cf u0 com.pvr.home/.RecActivity t4}\n"
    )
    assert not hand_dialog_focused(text)


def test_restore_focus_sends_back_then_starts_app(tmp_path, monkeypatch):
    adb = tmp_path / "adb.exe"
    adb.write_text("", encoding="utf-8")
    calls = []
    monkeypatch.setattr(
        "pico_bridge.bridge.subprocess.run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(returncode=0),
    )
    config = PicoConfig(adb_path=str(adb))
    restore_focus(config)
    assert calls[0] == [str(adb), "shell", "input", "keyevent", "KEYCODE_BACK"]
    assert calls[1] == [
        str(adb),
        "shell",
        "am",
        "start",
        "-n",
        f"{config.package}/.MainActivity",
    ]
