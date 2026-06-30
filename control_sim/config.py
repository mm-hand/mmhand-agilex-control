from __future__ import annotations

from dataclasses import dataclass


# =============================================================================
# Rotation sign convention (右手定则 / Right-hand rule)
# =============================================================================
# Point your right thumb along the positive axis. Your fingers curl in the
# positive rotation direction. Viewed along the positive axis toward the
# origin: counterclockwise is positive, clockwise is negative.
#
#   +X axis: thumb right      -> +roll  curls +Y toward +Z
#   +Y axis: thumb up         -> +pitch curls +Z toward +X
#   +Z axis: thumb toward you -> +yaw   curls +X toward +Y
#
# All angles in this file are in DEGREES (converted to radians at use site).
# All translations are in METERS.
# All transforms follow the naming convention:
#   transform_<target_frame>_from_<source_frame>
# so that chaining reads left-to-right: T_A_from_C = T_A_from_B @ T_B_from_C.
# =============================================================================


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "control_sim"
    poses_topic: str = "/read_pico/poses"
    tracked_topic: str = "/read_pico/tracked"
    max_pose_age_s: float = 0.25


@dataclass(frozen=True)
class FrameTransformConfig:
    # --- Left arm: PICO horizon <- left shoulder (pure translation) ---
    translation_pico_horizon_from_left_shoulder_m: tuple[float, float, float] = (-0.16, 0.0, 0.0)
    # Left shoulder <- left arm base: R_y(y_deg) then R_x(x_deg), no translation.
    # Result: arm_base Z axis in horizon = (-1, +1, -1)/sqrt(3), pointing upper-left.
    rotation_left_shoulder_from_left_arm_base_y_deg: float = -135.0
    rotation_after_y_left_shoulder_from_left_arm_base_x_deg: float = -35.264

    # --- Right arm: PICO horizon <- right shoulder (pure translation) ---
    # NOTE: placeholder values, edit here to match the real right shoulder.
    translation_pico_horizon_from_right_shoulder_m: tuple[float, float, float] = (0.16, 0.0, 0.0)
    # Right shoulder <- right arm base: R_y(y_deg) then R_x(x_deg), no translation.
    # Result: arm_base Z axis in horizon = (+1, +1, -1)/sqrt(3), pointing upper-right.
    # Mirror of the left arm about the horizon YZ plane.
    rotation_right_shoulder_from_right_arm_base_y_deg: float = 135.0
    rotation_after_y_right_shoulder_from_right_arm_base_x_deg: float = -35.264

    # --- PICO controller <- flange (fixed calibration, shared by both arms) ---
    translation_pico_controller_from_flange_m: tuple[float, float, float] = (0.0, -0.03, 0.065)
    rotation_pico_controller_from_flange_rpy_deg: tuple[float, float, float] = (0.0, 90.0, 0.0)

    # --- link7 <- flange (fixed offset from nero_with_revo2_flange_description.xacro) ---
    # Injected into robot.tool so IK solves the flange pose directly, avoiding the
    # xacro include chain (which uses $(find agx_arm_description), unresolvable here).
    translation_link7_from_flange_m: tuple[float, float, float] = (0.032, 0.0, -0.0235)
    rotation_link7_from_flange_rpy_deg: tuple[float, float, float] = (-90.0, 0.0, -90.0)


@dataclass(frozen=True)
class IkConfig:
    urdf_path: str = "control_sim/nero/urdf/nero_description.urdf"
    max_iter: int = 500
    tol: float = 1e-4


@dataclass(frozen=True)
class SimConfig:
    marker_radius_m: float = 0.02
    step_hz: float = 50.0


@dataclass(frozen=True)
class VisualizationConfig:
    axis_length_m: float = 0.08
    axis_line_width: float = 2.0
    refresh_interval_s: float = 0.02
    view_range_m: float = 0.35


@dataclass(frozen=True)
class RobotControlConfig:
    enabled: bool = False
    # Reserved for real-robot control: filter which arm(s) to send.
    #   "single_left" | "single_right" | "dual"
    mode: str = "dual"
