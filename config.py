from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "simple_control"
    poses_topic: str = "/read_pico/poses"
    tracked_topic: str = "/read_pico/tracked"
    max_pose_age_s: float = 0.25


@dataclass(frozen=True)
class FrameTransformConfig:
    # All translations are meters. RPY tuples are degrees, ordered roll/pitch/yaw.
    translation_left_shoulder_to_left_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_left_shoulder_to_left_arm_base_rpy_deg: tuple[float, float, float] = (126.2063, -12.2002, -143.7937)
    translation_right_shoulder_to_right_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_right_shoulder_to_right_arm_base_rpy_deg: tuple[float, float, float] = (-159.8959, 52.0616, -110.1041)

    translation_horizon_to_left_shoulder_m: tuple[float, float, float] = (-0.16, 0.0, 0.0)
    translation_horizon_to_right_shoulder_m: tuple[float, float, float] = (0.16, 0.0, 0.0)

    translation_controller_to_flange_m: tuple[float, float, float] = (0.0, -0.03, 0.065)
    rotation_controller_to_flange_rpy_deg: tuple[float, float, float] = (0.0, -180.0, 90.0)

    human_size_m: float = 1.0
    robot_size_m: float = 1.45


@dataclass(frozen=True)
class IkConfig:
    urdf_path: str = "nero_urdf/nero.urdf"
    max_iter: int = 500
    tol: float = 1e-4
    slimit: int = 100
    method: str = "sugihara"
    k: float = 0.01


@dataclass(frozen=True)
class SimConfig:
    step_hz: float = 50.0
