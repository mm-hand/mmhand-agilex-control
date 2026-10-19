from __future__ import annotations

from nicegui import ui

from config import ControlConfig
from ui.scene import TeleopScene


def build_simulation(config: ControlConfig):
    with ui.element("div").classes("sim-viewport w-full h-full"):
        scene = ui.scene(grid=(2, 20), background_color="#f8fafc", fps=10).classes("w-full h-full")
        scene.move_camera(
            x=1.7,
            y=-2.3,
            z=1.8,
            look_at_x=0.0,
            look_at_y=0.0,
            look_at_z=0.65,
            duration=0.0,
        )
        with ui.element("div").classes("scene-legend"):
            ui.label("LEFT").classes("legend-left")
            ui.label("RIGHT").classes("legend-right")
            ui.label("REAL FEEDBACK").classes("legend-real")
    visual = TeleopScene(scene, config)

    def refresh(snapshot) -> None:
        visual.update(snapshot)

    return refresh, visual
