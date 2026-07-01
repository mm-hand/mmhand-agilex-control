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
#
# Transform naming: ^A T_B = "frame B expressed in frame A" = A_from_B.
# Chaining reads left-to-right: A_from_C = A_from_B @ B_from_C.
#
# Formula (left arm shown; right arm analogous):
#   ^B T_U(t) = ^B T_S · ^S T_P · ^P T_C(t) · ^C T_D
#
# Frames:
#   B = arm base       S = shoulder        P = PICO horizon
#   C = controller     U = revo2_flange (xacro end link, IK target)
#
# Terms:
#   ^B T_S       = arm_base_from_shoulder        (constant: rotation + translation)
#   ^S T_P       = shoulder_from_horizon         (constant: translation only, same yaw as head)
#   ^P T_C(t)    = horizon_from_controller       (TIME-VARYING: from PICO ROS topic)
#   ^C T_D       = controller_from_flange        (constant: controller-to-flange calibration)
#
# Euler angle order: R = R_z(yaw) @ R_y(pitch) @ R_x(roll) [Z-Y-X intrinsic].
# rpy tuples in this file are (roll, pitch, yaw) in that order.
# =============================================================================


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "control_sim"
    poses_topic: str = "/read_pico/poses"
    tracked_topic: str = "/read_pico/tracked"
    max_pose_age_s: float = 0.25


@dataclass(frozen=True)
class FrameTransformConfig:
    # === ^S T_B: shoulder_from_arm_base (defines ^B T_S = inv(^S T_B)) ===
    # rpy = (roll, pitch, yaw). R = R_z(yaw) @ R_y(pitch) @ R_x(roll) [Z-Y-X intrinsic].
    # Left arm: arm_base Z axis in horizon = (-1, +1, -1)/sqrt(3), pointing upper-left.
    translation_left_shoulder_from_left_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_left_shoulder_from_left_arm_base_rpy_deg: tuple[float, float, float] = (126.2063, -12.2002, -143.7937)

    # Right arm: mirror of left about horizon YZ plane. Z axis = (+1, +1, -1)/sqrt(3).
    translation_right_shoulder_from_right_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_right_shoulder_from_right_arm_base_rpy_deg: tuple[float, float, float] = (-159.8959, 52.0616, -110.1041)

    # === ^P T_S: horizon_from_shoulder (defines ^S T_P = inv(^P T_S)) ===
    # Shoulder position in PICO horizon frame. Headset hangs on neck,
    # left shoulder is ~16cm to the left, right shoulder ~16cm to the right.
    translation_pico_horizon_from_left_shoulder_m: tuple[float, float, float] = (-0.16, 0.0, 0.0)
    translation_pico_horizon_from_right_shoulder_m: tuple[float, float, float] = (0.16, 0.0, 0.0)

    # === ^C T_D: controller_from_flange (fixed calibration, shared by both arms) ===
    # rpy = (roll, pitch, yaw). R = R_z(yaw) @ R_y(pitch) @ R_x(roll) [Z-Y-X intrinsic].
    translation_pico_controller_from_flange_m: tuple[float, float, float] = (0.0, 0.03, 0.065)
    rotation_pico_controller_from_flange_rpy_deg: tuple[float, float, float] = (0.0, -180.0, 90.0)


@dataclass(frozen=True)
class IkConfig:
    xacro_path: str = "control_sim/nero/urdf/nero_with_revo2_flange_description.xacro"
    max_iter: int = 500
    tol: float = 1e-4


@dataclass(frozen=True)
class SimConfig:
    marker_radius_m: float = 0.02
    step_hz: float = 50.0


@dataclass(frozen=True)
class RobotControlConfig:
    enabled: bool = False
    # Reserved for real-robot control: filter which arm(s) to send.
    #   "single_left" | "single_right" | "dual"
    mode: str = "dual"
