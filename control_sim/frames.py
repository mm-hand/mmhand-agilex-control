from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .config import FrameTransformConfig
from .transforms import (
    rotation_matrix_from_quat_xyzw,
    rotation_matrix_from_rpy_deg,
    rotation_matrix_x,
    rotation_matrix_y,
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


def _arm_base_in_horizon(
    translation_horizon_from_shoulder: tuple[float, float, float],
    rotation_shoulder_from_arm_base_y_deg: float,
    rotation_after_y_shoulder_from_arm_base_x_deg: float,
) -> np.ndarray:
    transform_horizon_from_shoulder = transform_from_translation(translation_horizon_from_shoulder)
    rotation_shoulder_from_arm_base = rotation_matrix_y(
        math.radians(rotation_shoulder_from_arm_base_y_deg)
    ) @ rotation_matrix_x(
        math.radians(rotation_after_y_shoulder_from_arm_base_x_deg)
    )
    transform_shoulder_from_arm_base = transform_from_rotation_and_translation(
        rotation_shoulder_from_arm_base, (0.0, 0.0, 0.0)
    )
    return transform_horizon_from_shoulder @ transform_shoulder_from_arm_base


def _arm_frames(
    pose: ControllerPose | None,
    base_in_horizon: np.ndarray,
    config: FrameTransformConfig,
) -> ArmFrames | None:
    if pose is None:
        return None

    rotation_horizon_from_controller = rotation_matrix_from_quat_xyzw(pose.quat_xyzw)
    transform_horizon_from_controller = transform_from_rotation_and_translation(
        rotation_horizon_from_controller, pose.xyz_m
    )

    rotation_controller_from_flange = rotation_matrix_from_rpy_deg(
        config.rotation_pico_controller_from_flange_rpy_deg
    )
    transform_controller_from_flange = transform_from_rotation_and_translation(
        rotation_controller_from_flange, config.translation_pico_controller_from_flange_m
    )
    transform_horizon_from_flange = transform_horizon_from_controller @ transform_controller_from_flange

    transform_arm_base_from_flange = _invert_transform(base_in_horizon) @ transform_horizon_from_flange

    return ArmFrames(
        transform_pico_horizon_from_arm_base=base_in_horizon,
        transform_pico_horizon_from_pico_controller=transform_horizon_from_controller,
        transform_pico_horizon_from_flange=transform_horizon_from_flange,
        transform_arm_base_from_flange=transform_arm_base_from_flange,
    )


class ArmBases(NamedTuple):
    left: np.ndarray
    right: np.ndarray


def arm_base_in_horizon(config: FrameTransformConfig) -> ArmBases:
    return ArmBases(
        left=_arm_base_in_horizon(
            config.translation_pico_horizon_from_left_shoulder_m,
            config.rotation_left_shoulder_from_left_arm_base_y_deg,
            config.rotation_after_y_left_shoulder_from_left_arm_base_x_deg,
        ),
        right=_arm_base_in_horizon(
            config.translation_pico_horizon_from_right_shoulder_m,
            config.rotation_right_shoulder_from_right_arm_base_y_deg,
            config.rotation_after_y_right_shoulder_from_right_arm_base_x_deg,
        ),
    )


def build_coordinate_frames(
    left_pose: ControllerPose | None,
    right_pose: ControllerPose | None,
    config: FrameTransformConfig,
) -> CoordinateFrames:
    bases = arm_base_in_horizon(config)
    return CoordinateFrames(
        left=_arm_frames(left_pose, bases.left, config),
        right=_arm_frames(right_pose, bases.right, config),
    )
