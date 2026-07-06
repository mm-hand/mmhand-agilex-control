from __future__ import annotations
from dataclasses import dataclass
from typing import NamedTuple
import numpy as np
from config import FrameTransformConfig
from transforms import (
    invert_transform,
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
        return cls((float(p.x), float(p.y), float(p.z)), (float(o.x), float(o.y), float(o.z), float(o.w)), received_at)
class ArmFrames(NamedTuple):
    transform_horizon_to_arm_base: np.ndarray
    transform_horizon_to_controller: np.ndarray
    transform_horizon_to_flange: np.ndarray
    transform_arm_base_to_flange: np.ndarray
class CoordinateFrames(NamedTuple):
    left: ArmFrames | None
    right: ArmFrames | None
class ArmBases(NamedTuple):
    left: np.ndarray
    right: np.ndarray
def _shoulder_to_arm_base(translation_m: tuple[float, float, float], rpy_deg: tuple[float, float, float]) -> np.ndarray:
    return transform_from_rotation_and_translation(rotation_matrix_from_rpy_deg(rpy_deg), translation_m)
def _controller_to_flange(config: FrameTransformConfig) -> np.ndarray:
    return transform_from_rotation_and_translation(
        rotation_matrix_from_rpy_deg(config.rotation_controller_to_flange_rpy_deg),
        config.translation_controller_to_flange_m,
    )
def _arm_frames(
    pose: ControllerPose | None,
    shoulder_to_arm_base: np.ndarray,
    horizon_to_shoulder: np.ndarray,
    controller_to_flange: np.ndarray,
    scale: float,
) -> ArmFrames | None:
    if pose is None:
        return None
    horizon_to_controller = transform_from_rotation_and_translation(rotation_matrix_from_quat_xyzw(pose.quat_xyzw), pose.xyz_m)
    shoulder_to_controller = invert_transform(horizon_to_shoulder) @ horizon_to_controller
    shoulder_to_controller_scaled = shoulder_to_controller.copy()
    shoulder_to_controller_scaled[:3, 3] *= scale
    horizon_to_controller_scaled = horizon_to_shoulder @ shoulder_to_controller_scaled
    return ArmFrames(
        transform_horizon_to_arm_base=horizon_to_shoulder @ shoulder_to_arm_base,
        transform_horizon_to_controller=horizon_to_controller_scaled,
        transform_horizon_to_flange=horizon_to_controller_scaled @ controller_to_flange,
        transform_arm_base_to_flange=invert_transform(shoulder_to_arm_base) @ shoulder_to_controller_scaled @ controller_to_flange,
    )
def arm_base_in_horizon(config: FrameTransformConfig) -> ArmBases:
    left_shoulder = transform_from_translation(config.translation_horizon_to_left_shoulder_m)
    right_shoulder = transform_from_translation(config.translation_horizon_to_right_shoulder_m)
    return ArmBases(
        left=left_shoulder @ _shoulder_to_arm_base(
            config.translation_left_shoulder_to_left_arm_base_m, config.rotation_left_shoulder_to_left_arm_base_rpy_deg
        ),
        right=right_shoulder @ _shoulder_to_arm_base(
            config.translation_right_shoulder_to_right_arm_base_m, config.rotation_right_shoulder_to_right_arm_base_rpy_deg
        ),
    )
def build_coordinate_frames(left_pose: ControllerPose | None, right_pose: ControllerPose | None, config: FrameTransformConfig) -> CoordinateFrames:
    left_shoulder = transform_from_translation(config.translation_horizon_to_left_shoulder_m)
    right_shoulder = transform_from_translation(config.translation_horizon_to_right_shoulder_m)
    left_base = _shoulder_to_arm_base(
        config.translation_left_shoulder_to_left_arm_base_m, config.rotation_left_shoulder_to_left_arm_base_rpy_deg
    )
    right_base = _shoulder_to_arm_base(
        config.translation_right_shoulder_to_right_arm_base_m, config.rotation_right_shoulder_to_right_arm_base_rpy_deg
    )
    tool = _controller_to_flange(config)
    scale = config.robot_size_m / config.human_size_m
    return CoordinateFrames(
        left=_arm_frames(left_pose, left_base, left_shoulder, tool, scale),
        right=_arm_frames(right_pose, right_base, right_shoulder, tool, scale),
    )
