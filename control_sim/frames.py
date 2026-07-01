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
    transform_pico_horizon_from_arm_base: np.ndarray
    transform_pico_horizon_from_pico_controller: np.ndarray
    transform_pico_horizon_from_flange: np.ndarray
    transform_arm_base_from_flange: np.ndarray


class CoordinateFrames(NamedTuple):
    left: ArmFrames | None
    right: ArmFrames | None


def _invert_transform(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ p)
    return inv


def _shoulder_from_arm_base(
    translation_shoulder_from_arm_base_m: tuple[float, float, float],
    rotation_shoulder_from_arm_base_rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """^S T_B: shoulder_from_arm_base (rotation + translation)."""
    rotation = rotation_matrix_from_rpy_deg(rotation_shoulder_from_arm_base_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, translation_shoulder_from_arm_base_m
    )


def _horizon_from_shoulder(
    translation_horizon_from_shoulder_m: tuple[float, float, float],
) -> np.ndarray:
    """^P T_S: horizon_from_shoulder (translation only, same yaw as head)."""
    return transform_from_translation(translation_horizon_from_shoulder_m)


def _controller_from_flange(config: FrameTransformConfig) -> np.ndarray:
    """^C T_D: controller_from_flange (fixed calibration)."""
    rotation = rotation_matrix_from_rpy_deg(config.rotation_pico_controller_from_flange_rpy_deg)
    return transform_from_rotation_and_translation(
        rotation, config.translation_pico_controller_from_flange_m
    )


def _arm_frames(
    pose: ControllerPose | None,
    shoulder_from_arm_base: np.ndarray,
    horizon_from_shoulder: np.ndarray,
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

    # ^B T_S = inv(^S T_B)  (arm_base_from_shoulder, constant)
    arm_base_from_shoulder = _invert_transform(shoulder_from_arm_base)

    # ^S T_P = inv(^P T_S)  (shoulder_from_horizon, constant)
    shoulder_from_horizon = _invert_transform(horizon_from_shoulder)

    # ^P T_C(t)  (horizon_from_controller, TIME-VARYING from PICO)
    horizon_from_controller = transform_from_rotation_and_translation(
        rotation_matrix_from_quat_xyzw(pose.quat_xyzw), pose.xyz_m
    )

    # ^C T_D  (controller_from_flange, constant)
    controller_from_flange = _controller_from_flange(config)

    # --- Shoulder-frame scaling of the controller pose ---
    # ^S T_C(t) = ^S T_P · ^P T_C(t); scale its translation by s.
    shoulder_from_controller = shoulder_from_horizon @ horizon_from_controller
    shoulder_from_controller_scaled = shoulder_from_controller.copy()
    shoulder_from_controller_scaled[:3, 3] *= scale

    # --- Chain: ^B T_D(t) = ^B T_S · ^S T_C'(t) · ^C T_D ---
    arm_base_from_flange = (
        arm_base_from_shoulder @ shoulder_from_controller_scaled @ controller_from_flange
    )

    # --- For visualization: horizon_from_flange (using scaled controller pose) ---
    # ^P T_C'(t) = ^P T_S · ^S T_C'(t); then ^P T_D = ^P T_C' · ^C T_D.
    horizon_from_controller_scaled = horizon_from_shoulder @ shoulder_from_controller_scaled
    horizon_from_arm_base = horizon_from_shoulder @ shoulder_from_arm_base
    horizon_from_flange = horizon_from_controller_scaled @ controller_from_flange

    return ArmFrames(
        transform_pico_horizon_from_arm_base=horizon_from_arm_base,
        transform_pico_horizon_from_pico_controller=horizon_from_controller_scaled,
        transform_pico_horizon_from_flange=horizon_from_flange,
        transform_arm_base_from_flange=arm_base_from_flange,
    )


class ArmBases(NamedTuple):
    left: np.ndarray
    right: np.ndarray


def arm_base_in_horizon(config: FrameTransformConfig) -> ArmBases:
    """Compute horizon_from_arm_base for both arms (for swift base placement)."""
    left = _horizon_from_shoulder(
        config.translation_pico_horizon_from_left_shoulder_m
    ) @ _shoulder_from_arm_base(
        config.translation_left_shoulder_from_left_arm_base_m,
        config.rotation_left_shoulder_from_left_arm_base_rpy_deg,
    )
    right = _horizon_from_shoulder(
        config.translation_pico_horizon_from_right_shoulder_m
    ) @ _shoulder_from_arm_base(
        config.translation_right_shoulder_from_right_arm_base_m,
        config.rotation_right_shoulder_from_right_arm_base_rpy_deg,
    )
    return ArmBases(left=left, right=right)


def build_coordinate_frames(
    left_pose: ControllerPose | None,
    right_pose: ControllerPose | None,
    config: FrameTransformConfig,
) -> CoordinateFrames:
    left_sfb = _shoulder_from_arm_base(
        config.translation_left_shoulder_from_left_arm_base_m,
        config.rotation_left_shoulder_from_left_arm_base_rpy_deg,
    )
    right_sfb = _shoulder_from_arm_base(
        config.translation_right_shoulder_from_right_arm_base_m,
        config.rotation_right_shoulder_from_right_arm_base_rpy_deg,
    )
    left_hfs = _horizon_from_shoulder(config.translation_pico_horizon_from_left_shoulder_m)
    right_hfs = _horizon_from_shoulder(config.translation_pico_horizon_from_right_shoulder_m)

    scale = config.robot_size_m / config.human_size_m

    return CoordinateFrames(
        left=_arm_frames(left_pose, left_sfb, left_hfs, config, scale),
        right=_arm_frames(right_pose, right_sfb, right_hfs, config, scale),
    )
