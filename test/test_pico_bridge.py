from __future__ import annotations

import logging
from types import SimpleNamespace

from config import PicoConfig
from pico_bridge.bridge import (
    PicoBridge,
    adb_remove_reverse,
    adb_setup,
    find_adb,
    focus_stealer,
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


def test_focus_stealer_detects_system_dialog():
    text = (
        "  mCurrentFocus=Window{6502072 u0 com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity}\n"
        "  mFocusedApp=ActivityRecord{55a3138 u0 com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity t14}\n"
    )
    assert focus_stealer(text) == "HandDialogActivity"


def test_focus_stealer_ignores_app_and_home():
    text = (
        "  mCurrentFocus=Window{6708500 u0 com.mmhand.pico_head_relative_pose/..."
        "com.mmhand.pico_head_relative_pose.MainActivity}\n"
        "  mFocusedApp=ActivityRecord{b5e96cf u0 com.pvr.home/.RecActivity t4}\n"
    )
    assert focus_stealer(text) is None


def test_focus_stealer_detects_the_vr_custom_dialog():
    # Observed on a PICO: this window, not HandDialogActivity, is what covers the
    # VR view, and the old guard only matched HandDialogActivity.
    text = "  mCurrentFocus=Window{a46feec u0 PicoXR_CustomDialog}\n"
    assert focus_stealer(text) == "PicoXR_CustomDialog"


def test_focus_stealer_ignores_a_stale_focused_app_record():
    # After the dialog window is gone, mFocusedApp keeps its ActivityRecord; the
    # guard must not keep force-stopping the overlay because of it.
    text = (
        "  mCurrentFocus=null  mFocusedApp=ActivityRecord{bf1fd22 u0 "
        "com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity t11}\n"
        "  mCurrentFocus=Window{22b3a33 u0 com.mmhand.pico_head_relative_pose/..."
        "com.mmhand.pico_head_relative_pose.MainActivity}\n"
    )
    assert focus_stealer(text) is None


def test_focus_stealer_ignores_the_system_menu():
    text = (
        "  mCurrentFocus=Window{7044a4e u0 com.pvr.appmanager/.AllAppActivity}\n"
        "  mFocusedApp=ActivityRecord{b5e96cf u0 com.pvr.home/.RecActivity t4}\n"
    )
    assert focus_stealer(text) is None


def test_focus_loop_dismisses_the_overlay_and_confirms(monkeypatch):
    import asyncio

    stolen = "  mCurrentFocus=Window{1b245c4 u0 com.picovr.systemext/com.pvr.vrdisplay.HandDialogActivity}\n"
    healthy = "  mCurrentFocus=Window{22b3a33 u0 com.mmhand.pico_head_relative_pose/.MainActivity}\n"
    outputs = iter([stolen, healthy])
    dismissed = []
    monkeypatch.setattr("pico_bridge.bridge.read_focus", lambda _config: next(outputs, healthy))
    monkeypatch.setattr("pico_bridge.bridge.restore_focus", lambda _config: dismissed.append(True))
    config = PicoConfig(focus_guard_interval_s=0.01, focus_guard_min_interval_s=0.0)
    bridge = PicoBridge(config, logging.getLogger("test"))

    async def run() -> None:
        task = asyncio.create_task(bridge._focus_loop())
        await asyncio.sleep(0.08)
        bridge.stop()
        await asyncio.wait_for(task, timeout=2.0)

    asyncio.run(run())
    assert dismissed == [True]


def test_restore_focus_force_stops_the_overlay(tmp_path, monkeypatch):
    adb = tmp_path / "adb.exe"
    adb.write_text("", encoding="utf-8")
    calls = []
    monkeypatch.setattr(
        "pico_bridge.bridge.subprocess.run",
        lambda command, **kwargs: calls.append(command) or SimpleNamespace(returncode=0),
    )
    config = PicoConfig(adb_path=str(adb))
    restore_focus(config)
    # BACK never reached the dialog (it owns its own virtual display) and starting
    # MainActivity was a no-op, so only the force-stop is issued.
    assert calls == [[str(adb), "shell", "am", "force-stop", config.overlay_package]]
