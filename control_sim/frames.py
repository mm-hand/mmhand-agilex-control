from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .config import FrameTransformConfig
from .transforms import (
    rotation_matrix_from_quat_xyzw,
    rotation_matrix_from_rpy_deg,
    transform_from_rotation_and_translation,
    transform_from_translation,
)

@dataclass(frozen=True)
class ControllerPose:
    xyz_m: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float]
    received_at: float

    @classmethod
    def from_ros_pose(cls, pose, received_at: float) -> ControllerPose:
        p, o = pose.position, pose.orientation
        return cls(
            xyz_m=(float(p.x), float(p.y), float(p.z)),
            quat_xyzw=(float(o.x), float(o.y), float(o.z), float(o.w)),
            received_at=received_at,
        )


class ArmFrames(NamedTuple):
    transform_arm_base_to_horizon: np.ndarray
    transform_controller_to_horizon: np.ndarray
    transform_flange_to_horizon: np.ndarray
    transform_flange_to_arm_base: np.ndarray


class CoordinateFrames(NamedTuple):
    left: ArmFrames | None
    right: ArmFrames | None


def _invert_transform(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ p)
    return inv


def _arm_base_to_shoulder(
    translation_arm_base_to_shoulder_m: tuple[float, float, float],
    rotation_arm_base_to_shoulder_rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """^S T_B: arm_base_to_shoulder (rotation + translation)."""
    rotation = rotation_matrix_from_rpy_deg(rotation_arm_base_to_shoulder_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, translation_arm_base_to_shoulder_m
    )


def _shoulder_to_horizon(
    translation_shoulder_to_horizon_m: tuple[float, float, float],
) -> np.ndarray:
    """^P T_S: shoulder_to_horizon (translation only, same yaw as head)."""
    return transform_from_translation(translation_shoulder_to_horizon_m)


def _flange_to_controller(config: FrameTransformConfig) -> np.ndarray:
    """^C T_D: flange_to_controller (fixed calibration)."""
    rotation = rotation_matrix_from_rpy_deg(config.rotation_flange_to_controller_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, config.translation_flange_to_controller_m
    )


def _arm_frames(
    pose: ControllerPose | None,
    arm_base_to_shoulder: np.ndarray,
    shoulder_to_horizon: np.ndarray,
    config: FrameTransformConfig,
    scale: float,
) -> ArmFrames | None:
    if pose is None:
        return None

    # --- Formula (with shoulder-origin scaling):
    #   ^B T_D(t) = ^B T_S · scale_S( ^S T_P · ^P T_C(t) ) · ^C T_D
    # where scale_S scales the TRANSLATION of its operand expressed in the
    # shoulder frame (rotation unchanged). Shoulder position is the scaling
    # origin, so arm bases and shoulder offsets are not affected. ---

    # ^B T_S = inv(^S T_B)  (shoulder_to_arm_base, constant)
    shoulder_to_arm_base = _invert_transform(arm_base_to_shoulder)

    # ^S T_P = inv(^P T_S)  (horizon_to_shoulder, constant)
    horizon_to_shoulder = _invert_transform(shoulder_to_horizon)

    # ^P T_C(t)  (controller_to_horizon, TIME-VARYING from PICO)
    controller_to_horizon = transform_from_rotation_and_translation(
        rotation_matrix_from_quat_xyzw(pose.quat_xyzw), pose.xyz_m
    )

    # ^C T_D  (flange_to_controller, constant)
    flange_to_controller = _flange_to_controller(config)

    # --- Shoulder-frame scaling of the controller pose ---
    # ^S T_C(t) = ^S T_P · ^P T_C(t); scale its translation by s.
    controller_to_shoulder = horizon_to_shoulder @ controller_to_horizon
    controller_to_shoulder_scaled = controller_to_shoulder.copy()
    controller_to_shoulder_scaled[:3, 3] *= scale

    # --- Chain: ^B T_D(t) = ^B T_S · ^S T_C'(t) · ^C T_D ---
    flange_to_arm_base = (
        shoulder_to_arm_base @ controller_to_shoulder_scaled @ flange_to_controller
    )

    # --- For visualization: horizon_from_flange (using scaled controller pose) ---
    # ^P T_C'(t) = ^P T_S · ^S T_C'(t); then ^P T_D = ^P T_C' · ^C T_D.
    controller_to_horizon_scaled = shoulder_to_horizon @ controller_to_shoulder_scaled
    arm_base_to_horizon = shoulder_to_horizon @ arm_base_to_shoulder
    flange_to_horizon = controller_to_horizon_scaled @ flange_to_controller

    return ArmFrames(
        transform_arm_base_to_horizon=arm_base_to_horizon,
        transform_controller_to_horizon=controller_to_horizon_scaled,
        transform_flange_to_horizon=flange_to_horizon,
        transform_flange_to_arm_base=flange_to_arm_base,
    )


class ArmBases(NamedTuple):
    left: np.ndarray
    right: np.ndarray


def arm_base_in_horizon(config: FrameTransformConfig) -> ArmBases:
    """Compute arm_base_to_horizon for both arms (for swift base placement)."""
    left = _shoulder_to_horizon(
        config.translation_left_shoulder_to_horizon_m
    ) @ _arm_base_to_shoulder(
        config.translation_left_arm_base_to_left_shoulder_m,
        config.rotation_left_arm_base_to_left_shoulder_rpy_deg,
    )
    right = _shoulder_to_horizon(
        config.translation_right_shoulder_to_horizon_m
    ) @ _arm_base_to_shoulder(
        config.translation_right_arm_base_to_right_shoulder_m,
        config.rotation_right_arm_base_to_right_shoulder_rpy_deg,
    )
    return ArmBases(left=left, right=right)


def build_coordinate_frames(
    left_pose: ControllerPose | None,
    right_pose: ControllerPose | None,
    config: FrameTransformConfig,
) -> CoordinateFrames:
    left_arm_base_to_shoulder = _arm_base_to_shoulder(
        config.translation_left_arm_base_to_left_shoulder_m,
        config.rotation_left_arm_base_to_left_shoulder_rpy_deg,
    )
    right_arm_base_to_shoulder = _arm_base_to_shoulder(
        config.translation_right_arm_base_to_right_shoulder_m,
        config.rotation_right_arm_base_to_right_shoulder_rpy_deg,
    )
    left_shoulder_to_horizon = _shoulder_to_horizon(config.translation_left_shoulder_to_horizon_m)
    right_shoulder_to_horizon = _shoulder_to_horizon(config.translation_right_shoulder_to_horizon_m)

    scale = config.robot_size_m / config.human_size_m

    return CoordinateFrames(
        left=_arm_frames(left_pose, left_arm_base_to_shoulder, left_shoulder_to_horizon, config, scale),
        right=_arm_frames(right_pose, right_arm_base_to_shoulder, right_shoulder_to_horizon, config, scale),
    )
