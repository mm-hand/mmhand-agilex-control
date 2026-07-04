from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from config import FrameTransformConfig
from transforms import (
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
    transform_horizon_to_arm_base: np.ndarray
    transform_horizon_to_controller: np.ndarray
    transform_horizon_to_flange: np.ndarray
    transform_arm_base_to_flange: np.ndarray


class CoordinateFrames(NamedTuple):
    left: ArmFrames | None
    right: ArmFrames | None


def _invert_transform(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ p)
    return inv


def _shoulder_to_arm_base(
    translation_shoulder_to_arm_base_m: tuple[float, float, float],
    rotation_shoulder_to_arm_base_rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """^S T_B: shoulder_to_arm_base (rotation + translation)."""
    rotation = rotation_matrix_from_rpy_deg(rotation_shoulder_to_arm_base_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, translation_shoulder_to_arm_base_m
    )


def _horizon_to_shoulder(
    translation_horizon_to_shoulder_m: tuple[float, float, float],
) -> np.ndarray:
    """^P T_S: horizon_to_shoulder (translation only, same yaw as head)."""
    return transform_from_translation(translation_horizon_to_shoulder_m)


def _controller_to_flange(config: FrameTransformConfig) -> np.ndarray:
    """^C T_D: controller_to_flange (fixed calibration)."""
    rotation = rotation_matrix_from_rpy_deg(config.rotation_controller_to_flange_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, config.translation_controller_to_flange_m
    )


def _arm_frames(
    pose: ControllerPose | None,
    shoulder_to_arm_base: np.ndarray,
    horizon_to_shoulder: np.ndarray,
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

    # ^B T_S = inv(^S T_B)  (arm_base_to_shoulder, constant)
    arm_base_to_shoulder = _invert_transform(shoulder_to_arm_base)

    # ^S T_P = inv(^P T_S)  (shoulder_to_horizon, constant)
    shoulder_to_horizon = _invert_transform(horizon_to_shoulder)

    # ^P T_C(t)  (horizon_to_controller, TIME-VARYING from pose input)
    horizon_to_controller = transform_from_rotation_and_translation(
        rotation_matrix_from_quat_xyzw(pose.quat_xyzw), pose.xyz_m
    )

    # ^C T_D  (controller_to_flange, constant)
    controller_to_flange = _controller_to_flange(config)

    # --- Shoulder-frame scaling of the controller pose ---
    # ^S T_C(t) = ^S T_P · ^P T_C(t); scale its translation by s.
    shoulder_to_controller = shoulder_to_horizon @ horizon_to_controller
    shoulder_to_controller_scaled = shoulder_to_controller.copy()
    shoulder_to_controller_scaled[:3, 3] *= scale

    # --- Chain: ^B T_D(t) = ^B T_S · ^S T_C'(t) · ^C T_D ---
    arm_base_to_flange = (
        arm_base_to_shoulder @ shoulder_to_controller_scaled @ controller_to_flange
    )

    # --- For visualization: horizon_to_flange (using scaled controller pose) ---
    # ^P T_C'(t) = ^P T_S · ^S T_C'(t); then ^P T_D = ^P T_C' · ^C T_D.
    horizon_to_controller_scaled = horizon_to_shoulder @ shoulder_to_controller_scaled
    horizon_to_arm_base = horizon_to_shoulder @ shoulder_to_arm_base
    horizon_to_flange = horizon_to_controller_scaled @ controller_to_flange

    return ArmFrames(
        transform_horizon_to_arm_base=horizon_to_arm_base,
        transform_horizon_to_controller=horizon_to_controller_scaled,
        transform_horizon_to_flange=horizon_to_flange,
        transform_arm_base_to_flange=arm_base_to_flange,
    )


class ArmBases(NamedTuple):
    left: np.ndarray
    right: np.ndarray


def arm_base_in_horizon(config: FrameTransformConfig) -> ArmBases:
    """Compute horizon_to_arm_base for both arms (for swift base placement)."""
    left = _horizon_to_shoulder(
        config.translation_horizon_to_left_shoulder_m
    ) @ _shoulder_to_arm_base(
        config.translation_left_shoulder_to_left_arm_base_m,
        config.rotation_left_shoulder_to_left_arm_base_rpy_deg,
    )
    right = _horizon_to_shoulder(
        config.translation_horizon_to_right_shoulder_m
    ) @ _shoulder_to_arm_base(
        config.translation_right_shoulder_to_right_arm_base_m,
        config.rotation_right_shoulder_to_right_arm_base_rpy_deg,
    )
    return ArmBases(left=left, right=right)


def build_coordinate_frames(
    left_pose: ControllerPose | None,
    right_pose: ControllerPose | None,
    config: FrameTransformConfig,
) -> CoordinateFrames:
    left_shoulder_to_arm_base = _shoulder_to_arm_base(
        config.translation_left_shoulder_to_left_arm_base_m,
        config.rotation_left_shoulder_to_left_arm_base_rpy_deg,
    )
    right_shoulder_to_arm_base = _shoulder_to_arm_base(
        config.translation_right_shoulder_to_right_arm_base_m,
        config.rotation_right_shoulder_to_right_arm_base_rpy_deg,
    )
    left_horizon_to_shoulder = _horizon_to_shoulder(config.translation_horizon_to_left_shoulder_m)
    right_horizon_to_shoulder = _horizon_to_shoulder(config.translation_horizon_to_right_shoulder_m)

    scale = config.robot_size_m / config.human_size_m

    return CoordinateFrames(
        left=_arm_frames(left_pose, left_shoulder_to_arm_base, left_horizon_to_shoulder, config, scale),
        right=_arm_frames(right_pose, right_shoulder_to_arm_base, right_horizon_to_shoulder, config, scale),
    )
