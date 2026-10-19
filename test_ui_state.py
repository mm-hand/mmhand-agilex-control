from __future__ import annotations

from ui.app import TABS, _pico_state, status_background


def test_arm_simulation_is_first_and_default_tab():
    assert TABS == ["Arm Simulation", "Controllers"]


def test_status_colors_cover_running_waiting_and_fault_states():
    assert status_background("TELEOP") == "#bbf7d0"
    assert status_background("CONNECTED 96Hz") == "#bbf7d0"
    assert status_background("ALIGNING") == "#bfdbfe"
    assert status_background("WAITING") == "#fde68a"
    assert status_background("STOPPED") == "#fecaca"
    assert status_background("SIM") == "#e2e8f0"


def test_pico_state_requires_live_frames():
    assert _pico_state({"connected": False, "adb": "waiting"}) == "WAITING"
    assert _pico_state({"connected": True, "adb": "connected", "rate_hz": 96.2}) == "CONNECTED 96Hz"
    assert _pico_state({"connected": False, "adb": "error"}) == "ERROR"
