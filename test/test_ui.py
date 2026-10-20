from __future__ import annotations

import ast
import asyncio
from pathlib import Path

from config import ControlConfig
from controller_state import ArmStatus, ControlSnapshot, SnapshotStore
from logging_utils import LogBuffer
from nicegui import ui
from ui.app import TABS, _pico_state, build_page, status_background
from ui.scene import CAMERA_FOV, CAMERA_POSITION

UI = Path(__file__).resolve().parents[1] / "ui"
FORBIDDEN = (
    "Return to Initial Pose",
    "Pause Teleop",
    "Sync Real Arm",
    "Base pose",
    "Iso",
    "Top",
    "Front",
    "Reset View",
)


class FakeController:
    def submit(self, command) -> None:
        pass


def test_ui_strings_are_english_and_hide_removed_controls():
    strings = []
    for path in UI.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        strings.extend(
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        )
    joined = "\n".join(strings)
    assert not any("\u4e00" <= char <= "\u9fff" for char in joined)
    for text in FORBIDDEN:
        assert text not in joined


def test_arm_simulation_is_first_and_default_tab():
    assert TABS == ["Arm Simulation", "Controllers", "Settings"]


def test_scene_camera_is_zoomed_in():
    assert CAMERA_FOV < 75
    assert CAMERA_POSITION != (1.7, -2.3, 1.8)


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


async def test_page_builds_with_english_shell_and_system_log():
    from nicegui.testing.user_simulation import user_simulation

    store = SnapshotStore()
    store.publish(ControlSnapshot(phase="TELEOP"))
    config = ControlConfig()
    buffer = LogBuffer()
    buffer.append("control", "system log line")

    async with user_simulation(
        root=lambda: build_page(FakeController(), store, config, buffer)
    ) as user:
        await user.open("/")
        await user.should_see("AgileX Nero Teleop")
        await user.should_see("Connect Arms")
        await user.should_see("Arm Simulation")
        await user.should_see("Controllers")
        await user.should_see("System Log")
        await user.should_see("system log line")
        await user.should_see("System: TELEOP")
        await user.should_see("Left Control")
        await user.should_see("Right Control")
        await user.should_see("ROS 2 Output")
        await user.should_see("Settings")
        await user.should_see("Startup Pose")
        await user.should_see("Base Frames")
        await user.should_see("Restore Defaults")
        await user.should_see("Head / Horizon")
        await user.should_see("STOP")


async def test_start_button_replaces_stop_after_stop():
    from nicegui.testing.user_simulation import user_simulation

    store = SnapshotStore()
    store.publish(ControlSnapshot(phase="STOPPED", can_reset=True))
    config = ControlConfig()
    buffer = LogBuffer()

    async with user_simulation(
        root=lambda: build_page(FakeController(), store, config, buffer)
    ) as user:
        await user.open("/")
        await asyncio.sleep(0.2)
        await user.should_see("Start")
        await user.should_not_see(kind=ui.button, content="STOP")


async def test_start_dialog_requires_verification_for_connected_arm():
    from nicegui.testing.user_simulation import user_simulation

    store = SnapshotStore()
    store.publish(
        ControlSnapshot(
            phase="STOPPED",
            can_reset=True,
            arm_status={"left": ArmStatus(state="STOPPED"), "right": ArmStatus()},
        )
    )
    config = ControlConfig()
    buffer = LogBuffer()

    async with user_simulation(
        root=lambda: build_page(FakeController(), store, config, buffer)
    ) as user:
        await user.open("/")
        await asyncio.sleep(0.2)
        user.find(marker="start-top").click()
        await asyncio.sleep(0.2)
        await user.should_see("I have verified the simulated and real arm bases match.")
        await user.should_see("Start returns the simulation and every connected real arm to the startup pose.")


async def test_real_feedback_creates_ghost_arm():
    from nicegui.testing.user_simulation import user_simulation

    store = SnapshotStore()
    config = ControlConfig()
    buffer = LogBuffer()

    async with user_simulation(
        root=lambda: build_page(FakeController(), store, config, buffer)
    ) as user:
        await user.open("/")
        store.publish(
            ControlSnapshot(
                phase="TELEOP",
                real_q={"left": (0.1,) * 7, "right": None},
            )
        )
        await asyncio.sleep(0.3)
