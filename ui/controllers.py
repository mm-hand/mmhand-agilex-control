from __future__ import annotations

import math

from nicegui import ui

KEYS = {
    "left": (("X", 0), ("Y", 1), ("Menu", 2), ("Trigger", 3), ("Grip", 4), ("Stick", 5)),
    "right": (("A", 0), ("B", 1), ("Trigger", 2), ("Grip", 3), ("Stick", 4)),
}


def _ypr(quat) -> tuple[float, float, float]:
    x, y, z, w = quat
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def build_controllers():
    cards = {}
    with ui.row().classes("w-full items-start gap-4"):
        for side, title in (("left", "Left Controller"), ("right", "Right Controller")):
            with ui.card().classes("flex-1 min-w-0"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label(title).classes("font-semibold text-sm")
                    status = ui.label("Waiting").classes("status-chip text-xs")
                position = ui.label("Position (m)   --").classes("mono text-xs")
                quaternion = ui.label("Quaternion     --").classes("mono text-xs")
                angles = ui.label("Yaw/Pitch/Roll --").classes("mono text-xs")
                with ui.row().classes("gap-1"):
                    buttons = []
                    for name, _bit in KEYS[side]:
                        buttons.append(ui.label(name).classes("key"))
                cards[side] = (status, position, quaternion, angles, buttons)

    def refresh(snapshot) -> None:
        for side in ("left", "right"):
            status, position, quaternion, angles, buttons = cards[side]
            controller = snapshot.controllers.get(side)
            if controller is None:
                status.set_text("Waiting")
                status.style("background-color:#e2e8f0")
                position.set_text("Position (m)   --")
                quaternion.set_text("Quaternion     --")
                angles.set_text("Yaw/Pitch/Roll --")
                for button in buttons:
                    button.classes(remove="key-on")
                continue
            status.set_text("Tracked")
            status.style("background-color:#bbf7d0")
            x, y, z = controller.xyz
            qx, qy, qz, qw = controller.quat
            roll, pitch, yaw = _ypr(controller.quat)
            position.set_text(f"Position (m)   {x:+.3f}  {y:+.3f}  {z:+.3f}")
            quaternion.set_text(f"Quaternion     {qx:+.3f}  {qy:+.3f}  {qz:+.3f}  {qw:+.3f}")
            angles.set_text(f"Yaw/Pitch/Roll {yaw:+.1f} / {pitch:+.1f} / {roll:+.1f}")
            for index, button in enumerate(buttons):
                on = bool(controller.buttons & (1 << KEYS[side][index][1]))
                if on:
                    button.classes(add="key-on")
                else:
                    button.classes(remove="key-on")

    return refresh
