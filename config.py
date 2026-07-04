from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RuntimeConfig:
    mode: str = "dual_sim"


@dataclass(frozen=True)
class PoseInputConfig:
    node_name: str = "analytic_control"
    poses_topic: str = "/read_pose/poses"
    tracked_topic: str = "/read_pose/tracked"
    max_pose_age_s: float = 0.25
    disconnect_s: float = 0.5


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
    n_psi: int = 181
    local_theta0_window: float = 1.0
    local_theta0_count: int = 41
    w_vel: float = 1.0
    w_acc: float = 0.25
    w_pose: float = 0.1
    w_theta0: float = 0.15
    hysteresis_margin: float = 0.5
    enable_initial_global_search: bool = True
    enable_global_fallback: bool = False


@dataclass(frozen=True)
class OneEuroConfig:
    enabled: bool = True
    min_cutoff: float = 1.0
    beta: float = 0.02
    d_cutoff: float = 1.0


@dataclass(frozen=True)
class FilterConfig:
    pose: OneEuroConfig = field(default_factory=lambda: OneEuroConfig(enabled=True, min_cutoff=1.2, beta=0.03, d_cutoff=1.0))


@dataclass(frozen=True)
class MotionConfig:
    enabled: bool = False
    command_hz: float = 50.0
    blend_trigger_rad: float = 0.05
    replan_trigger_rad: float = 0.08
    min_replan_interval_s: float = 0.25
    min_duration_s: float = 0.12
    max_duration_s: float = 2.0
    max_velocity_rad_s: tuple[float, ...] = (1.0, 1.0, 1.2, 1.2, 1.5, 1.5, 1.5)
    max_acceleration_rad_s2: tuple[float, ...] = (2.5, 2.5, 3.0, 3.0, 4.0, 4.0, 4.0)


@dataclass(frozen=True)
class SimConfig:
    step_hz: float = 50.0
    visual_z_offset_m: float = 0.6
    show_horizon_frame: bool = True
    horizon_frame_axis_length_m: float = 0.18


@dataclass(frozen=True)
class LogConfig:
    enable_file_log: bool = True
    directory: str = "logs"
    filename: str = "analytic_control.log"
    max_bytes: int = 2_000_000
    backup_count: int = 3


@dataclass(frozen=True)
class ControlConfig:
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    pose_input: PoseInputConfig = field(default_factory=PoseInputConfig)
    frames: FrameTransformConfig = field(default_factory=FrameTransformConfig)
    ik: IkConfig = field(default_factory=IkConfig)
    filters: FilterConfig = field(default_factory=FilterConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    log: LogConfig = field(default_factory=LogConfig)
