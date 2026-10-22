from __future__ import annotations

import math
from typing import Callable

from nicegui import ui

from config import ControlConfig
from settings_store import reset_settings, save_settings, settings_data


def build_settings(config: ControlConfig, on_apply: Callable[[ControlConfig], None]) -> None:
    values = settings_data(config)
    startup_inputs = {}
    frame_inputs = {}

    ui.label("Settings are applied immediately.").classes("text-sm text-slate-600")

    with ui.card().classes("w-full"):
        ui.label("Startup Pose").classes("font-semibold text-base")
        for side, title in (("left", "Left Arm"), ("right", "Right Arm")):
            ui.label(f"{title} (deg)").classes("text-xs font-semibold text-slate-600")
            with ui.row().classes("w-full items-center gap-2"):
                inputs = []
                for index, value in enumerate(values["startup"][f"{side}_pose_rad"]):
                    ui.label(f"J{index + 1}").classes("text-xs")
                    inputs.append(
                        ui.number(value=math.degrees(value), step=0.1, format="%.2f")
                        .props("dense outlined")
                        .classes("w-24")
                    )
            startup_inputs[side] = inputs

    with ui.card().classes("w-full"):
        ui.label("Base Frames").classes("font-semibold text-base")
        for side, title in (("left", "Left Arm"), ("right", "Right Arm")):
            ui.label(title).classes("text-xs font-semibold text-slate-600")
            frame_inputs[side] = {}
            for key, label, step, fmt in (
                (f"translation_horizon_to_{side}_shoulder_m", "Horizon to Shoulder (m)", 0.001, "%.4f"),
                (f"translation_{side}_shoulder_to_{side}_arm_base_m", "Shoulder to Base (m)", 0.001, "%.4f"),
                (f"rotation_{side}_shoulder_to_{side}_arm_base_rpy_deg", "Shoulder to Base RPY (deg)", 0.1, "%.2f"),
            ):
                with ui.row().classes("w-full items-center gap-2"):
                    ui.label(label).classes("w-52 text-xs")
                    inputs = [
                        ui.number(value=value, step=step, format=fmt).props("dense outlined").classes("w-24")
                        for value in values["frames"][key]
                    ]
                frame_inputs[side][key] = inputs

    status = ui.label("").classes("text-sm text-slate-700")

    def save() -> None:
        nonlocal config
        try:
            data = {
                "startup": {
                    f"{side}_pose_rad": [math.radians(number.value) for number in startup_inputs[side]]
                    for side in ("left", "right")
                },
                "frames": {
                    key: [number.value for number in inputs]
                    for side in ("left", "right")
                    for key, inputs in frame_inputs[side].items()
                },
            }
            config = save_settings(data, config=config)
        except Exception as exc:
            status.set_text(f"Save failed: {exc}")
            ui.notify(f"Save failed: {exc}", type="negative")
            return
        on_apply(config)
        status.set_text("Applied.")
        ui.notify("Settings applied.", type="positive")

    def restore() -> None:
        nonlocal config
        reset_settings()
        defaults = settings_data(ControlConfig())
        for side in ("left", "right"):
            for index, number in enumerate(startup_inputs[side]):
                number.set_value(math.degrees(defaults["startup"][f"{side}_pose_rad"][index]))
            for key, inputs in frame_inputs[side].items():
                for index, number in enumerate(inputs):
                    number.set_value(defaults["frames"][key][index])
        config = ControlConfig()
        on_apply(config)
        status.set_text("Defaults restored and applied.")
        ui.notify("Defaults restored and applied.", type="positive")

    with ui.row().classes("w-full items-center gap-2"):
        ui.button("Apply", on_click=save).props("unelevated")
        ui.button("Restore Defaults", on_click=restore).props("outline")
