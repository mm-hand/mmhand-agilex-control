from __future__ import annotations

import sys

from nicegui import app, ui

from config import default_can_channels
from controller_state import ControlCommand
from logging_utils import LogBuffer
from ui.controllers import build_controllers
from ui.scene import MESH_DIR, build_scene
from ui.settings import build_settings

TABS = ["Arm Simulation", "Controllers", "Settings"]
DEFAULT_CHANNELS = dict(zip(("left", "right"), default_can_channels()))
REAL_ARM_SUPPORTED = sys.platform.startswith(("win32", "linux"))

CSS = """
body {background:#F5F7FA;color:#1e293b}
.nicegui-content {padding:0;gap:0;height:100dvh;overflow:hidden}
.console-shell {display:flex;flex-direction:column;height:100dvh;width:100%;overflow:hidden}
.connection-bar {min-height:44px;padding:4px 12px;background:white;gap:8px;flex-shrink:0;align-items:center}
.connection-title {font-size:16px;font-weight:600;white-space:nowrap}
.status-bar {min-height:32px;padding:4px 12px;background:#eef2f7;gap:8px;flex-shrink:0;overflow-x:auto;flex-wrap:nowrap}
.status-chip {padding:4px 8px;border-radius:5px;color:#1e293b;font-weight:500;white-space:nowrap}
.console-tabs {height:36px;min-height:36px;background:white;flex-shrink:0}
.console-tabs .q-tab {min-height:36px;padding:0 12px;font-size:12px}
.console-workspace {flex:1;min-height:0;width:100%}
.console-workspace > .q-splitter__before {display:flex;flex-direction:column;min-height:0}
.console-workspace > .q-splitter__separator {height:8px;background:#e2e8f0;cursor:row-resize}
.log-resize-grip {width:40px;height:3px;border-radius:2px;background:#94a3b8}
.page-panels {flex:1;min-height:0;width:100%;overflow:hidden}
.page-panels .q-tab-panel {padding:0;height:100%;display:flex;flex-direction:column;gap:0}
.controllers-panel {padding:12px;overflow:auto;gap:12px}
.settings-panel {padding:12px;overflow:auto;gap:12px}
.sim-viewport {position:relative;flex:1;min-height:0;width:100%}
.sim-viewport .nicegui-scene {width:100%;height:100%}
.scene-legend {position:absolute;left:12px;bottom:10px;display:flex;gap:10px;font-size:11px;background:#ffffffcc;padding:4px 8px;border-radius:5px;pointer-events:none}
.legend-left {color:#2563eb;font-weight:600}
.legend-right {color:#f97316;font-weight:600}
.legend-real {color:#16a34a;font-weight:600}
.legend-frame {color:#475569;font-weight:600}
.system-log {height:100%;min-height:0;background:white;padding:6px 12px;gap:4px;overflow:hidden}
.system-log .nicegui-log {flex:1;min-height:0;width:100%;font:11px ui-monospace,monospace;color:#475569}
.q-card {border-radius:10px;box-shadow:0 1px 4px #182b4410;padding:12px;gap:8px}
.q-btn {text-transform:none;border-radius:6px;font-size:12px}
.stop-button {background:#dc2626!important;color:white!important;font-weight:700;min-width:86px}
.key {display:inline-block;padding:3px 7px;border:1px solid #cbd5e1;border-radius:4px;color:#64748b;font-size:11px}
.key.key-on {background:#2563eb;border-color:#2563eb;color:white}
.mono {font-family:ui-monospace,monospace;white-space:pre}
"""


def status_background(value: str) -> str:
    state = str(value).upper()
    if state.startswith("CONNECTED") or state in ("TELEOP", "TRACKED", "READY", "ON"):
        return "#bbf7d0"
    if state in ("STARTUP", "ALIGNING", "CONNECTING"):
        return "#bfdbfe"
    if state in ("WAITING",):
        return "#fde68a"
    if state in ("STOPPED", "FAULT", "ERROR", "UNAVAILABLE"):
        return "#fecaca"
    return "#e2e8f0"


def _pico_state(pico: dict) -> str:
    if pico.get("adb") == "error":
        return "ERROR"
    if not pico.get("connected"):
        return "WAITING"
    rate = pico.get("rate_hz", 0.0)
    return f"CONNECTED {rate:.0f}Hz" if rate else "CONNECTED"


def build_page(controller, store, config, log_buffer: LogBuffer) -> None:
    ui.colors(primary="#2563eb", positive="#16a34a", warning="#ea880c", negative="#dc2626")
    ui.add_css(CSS)
    log_sequence = [0]
    last_error = [None]
    ros_syncing = [False]

    with ui.dialog().props("persistent") as connect_dialog, ui.card().classes("w-[560px] max-w-full"):
        ui.label("Connect Arms").classes("font-semibold text-lg")
        rows = {}
        for side in ("left", "right"):
            with ui.row().classes("w-full items-center gap-2"):
                ui.label(f"{side.title()} Arm").classes("w-24 font-semibold text-xs")
                state = ui.label("SIM").classes("status-chip text-xs w-24")
                channel = ui.input("Channel", value=DEFAULT_CHANNELS[side]).props("dense outlined").classes("w-28")
                button = ui.button("Connect").props("dense outline")
            rows[side] = (state, channel, button)
        ui.button("Close", on_click=connect_dialog.close).props("flat")
        if not REAL_ARM_SUPPORTED:
            ui.label("Real arm connection requires Windows (agx_cando) or Linux (SocketCAN).").classes("text-xs text-red-600")

    def connect_click(side: str) -> None:
        snapshot = store.read()
        state = snapshot.arm_status[side].state
        if state in ("READY", "TELEOP", "STOPPED"):
            controller.submit(ControlCommand("DISCONNECT", side=side))
        else:
            controller.submit(ControlCommand("CONNECT", side=side, channel=rows[side][1].value or DEFAULT_CHANNELS[side]))

    for side in rows:
        rows[side][2].on("click", lambda _=None, side=side: connect_click(side))

    start_requires_verification = [False]

    with ui.dialog().props("persistent") as start_dialog, ui.card().classes("w-[560px] max-w-full"):
        ui.label("Start").classes("font-semibold text-lg")
        ui.label(
            "STOP is latched. Start returns the simulation and every connected real arm to the startup pose."
        ).classes("text-sm whitespace-pre-wrap")
        start_verification = ui.checkbox("I have verified the simulated and real arm bases match.")
        start_verification.set_visibility(False)
        with ui.row().classes("w-full justify-end"):
            start_cancel = ui.button("Cancel").props("flat")
            start_confirm = ui.button("Start").props("unelevated")

        def update_start_confirm() -> None:
            start_confirm.set_enabled(not start_requires_verification[0] or bool(start_verification.value))

        def open_start_dialog() -> None:
            snapshot = store.read()
            connected = any(
                snapshot.arm_status[side].state in ("READY", "STOPPED", "TELEOP", "ALIGNING")
                for side in ("left", "right")
            )
            start_requires_verification[0] = connected
            start_verification.set_visibility(connected)
            start_verification.set_value(False)
            update_start_confirm()
            start_dialog.open()

        def confirm_start() -> None:
            controller.submit(ControlCommand("RESET"))
            start_dialog.close()

        start_verification.on_value_change(lambda _=None: update_start_confirm())
        start_cancel.on("click", lambda _=None: start_dialog.close())
        start_confirm.on("click", lambda _=None: confirm_start())

    with ui.element("div").classes("console-shell"):
        with ui.row().classes("connection-bar w-full"):
            ui.label("AgileX Nero Teleop").classes("connection-title")
            connect_button = ui.button("Connect Arms").props("dense outline")
            connect_button.on("click", lambda _=None: connect_dialog.open())
            ui.space()
            ros_switch = ui.switch("ROS 2 Output")
            start_button = ui.button("Start").props("outline").mark("start-top")
            start_button.set_visibility(False)
            with start_button:
                ui.tooltip("Start returns the simulation and connected arms to the startup pose after STOP.")
            stop_button = ui.button("STOP").classes("stop-button").props("unelevated")

        with ui.row().classes("status-bar w-full"):
            chips = {
                "System": ui.label().classes("status-chip text-xs"),
                "PICO Input": ui.label().classes("status-chip text-xs"),
                "Left Control": ui.label().classes("status-chip text-xs"),
                "Right Control": ui.label().classes("status-chip text-xs"),
                "Left Arm": ui.label().classes("status-chip text-xs"),
                "Right Arm": ui.label().classes("status-chip text-xs"),
                "ROS 2": ui.label().classes("status-chip text-xs"),
            }

        with ui.tabs(value=TABS[0]).classes("console-tabs w-full").props("dense no-caps active-color=primary") as tabs:
            for name in TABS:
                ui.tab(name)

        with ui.splitter(horizontal=True, reverse=True, value=18, limits=(10, 70)).classes("console-workspace") as workspace:
            with workspace.before:
                with ui.tab_panels(tabs, value=TABS[0], animated=False).classes("page-panels").props("keep-alive"):
                    with ui.tab_panel(TABS[0]).classes("w-full h-full"):
                        simulation_refresh, visual = build_scene(config)
                    with ui.tab_panel(TABS[1]).classes("w-full h-full controllers-panel"):
                        controllers_refresh = build_controllers()
                    with ui.tab_panel(TABS[2]).classes("w-full h-full settings-panel"):
                        build_settings(config)
            with workspace.separator:
                with ui.element("div").classes("log-resize-grip"):
                    ui.tooltip("Drag to resize System Log")
            with workspace.after:
                with ui.column().classes("system-log w-full"):
                    ui.label("System Log").classes("text-xs font-semibold")
                    log = ui.log(max_lines=2000)

    workspace.on_value_change(lambda _: visual.resize())
    tabs.on_value_change(lambda _: visual.resize())

    def stop_click() -> None:
        controller.submit(ControlCommand("STOP"))

    stop_button.on("click", lambda _=None: stop_click())
    start_button.on("click", lambda _=None: open_start_dialog())

    def ros_changed(event) -> None:
        if ros_syncing[0]:
            return
        controller.submit(ControlCommand("SET_ROS", enabled=bool(event.value)))

    ros_switch.on_value_change(ros_changed)

    def refresh() -> None:
        snapshot = store.read()
        system = snapshot.phase
        pico = _pico_state(snapshot.pico)
        left_ctrl = "TRACKED" if snapshot.controllers.get("left") else "LOST"
        right_ctrl = "TRACKED" if snapshot.controllers.get("right") else "LOST"
        left_arm = snapshot.arm_status["left"].state
        right_arm = snapshot.arm_status["right"].state
        if snapshot.ros.get("available") is False:
            ros = "UNAVAILABLE"
        elif snapshot.ros.get("enabled"):
            ros = "ON"
        else:
            ros = "OFF"
        for name, value in (
            ("System", system),
            ("PICO Input", pico),
            ("Left Control", left_ctrl),
            ("Right Control", right_ctrl),
            ("Left Arm", left_arm),
            ("Right Arm", right_arm),
            ("ROS 2", ros),
        ):
            chips[name].set_text(f"{name}: {value}")
            chips[name].style(f"background-color:{status_background(value)}")

        stopped = system == "STOPPED"
        fault = system == "FAULT"
        aligning = system == "ALIGNING"
        connect_button.set_enabled(not fault and not aligning)
        stop_button.set_visibility(not stopped)
        stop_button.set_enabled(not fault)
        start_button.set_visibility(stopped)
        start_button.set_enabled(bool(snapshot.can_reset))
        ros_syncing[0] = True
        ros_switch.set_value(bool(snapshot.ros.get("enabled")))
        ros_switch.set_enabled(not fault and snapshot.ros.get("available") is not False)
        ros_syncing[0] = False

        for side, (state_label, channel, button) in rows.items():
            status = snapshot.arm_status[side]
            state_label.set_text(status.state)
            state_label.style(f"background-color:{status_background(status.state)}")
            connected = status.state in ("READY", "TELEOP", "STOPPED")
            busy = status.state in ("CONNECTING", "ALIGNING")
            button.set_text("Disconnect" if connected else "Connect")
            button.set_enabled(
                REAL_ARM_SUPPORTED
                and not fault
                and not busy
                and not aligning
            )
            channel.set_enabled(not connected and not busy)

        if tabs.value == TABS[0]:
            simulation_refresh(snapshot)
        elif tabs.value == TABS[1]:
            controllers_refresh(snapshot)

        if snapshot.error and snapshot.error != last_error[0]:
            last_error[0] = snapshot.error
            ui.notify(snapshot.error, type="negative")

        new_lines, log_sequence[0] = log_buffer.read_since(log_sequence[0])
        for line in new_lines:
            log.push(line)

    refresh()
    ui.timer(1.0 / config.gui.refresh_hz, refresh)


def run_ui(controller, store, config, log_buffer: LogBuffer, port: int, show: bool = True) -> None:
    if not MESH_DIR.is_dir():
        raise RuntimeError(f"URDF mesh directory not found: {MESH_DIR}")
    app.add_static_files("/nero_meshes", str(MESH_DIR))

    @ui.page("/")
    def index() -> None:
        build_page(controller, store, config, log_buffer)

    app.on_shutdown(controller.stop)
    ui.run(
        host=config.gui.host,
        port=port,
        show=show,
        reload=False,
        title="AgileX Nero Teleop",
    )
