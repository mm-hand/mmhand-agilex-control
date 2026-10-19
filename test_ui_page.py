from __future__ import annotations

import asyncio

import pytest

from config import ControlConfig
from controller_state import ControlSnapshot, SnapshotStore
from logging_utils import LogBuffer
from ui.app import build_page


class FakeController:
    def submit(self, command) -> None:
        pass


@pytest.mark.asyncio
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
        await user.should_see("SYSTEM: TELEOP")
        await user.should_see("ROS2 Output")
        await user.should_see("STOP")


@pytest.mark.asyncio
async def test_handoff_dialog_opens_for_connected_arms():
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
                phase="HANDOFF",
                handoff_id=1,
                handoff_sides=("left",),
            )
        )
        await asyncio.sleep(0.3)
        await user.should_see("Real Arm Handoff")
        await user.should_see("Connected arms: LEFT")
        await user.should_see("I have verified the simulated and real arm bases match.")
