from __future__ import annotations
from dataclasses import dataclass, field
@dataclass(frozen=True)
class RuntimeConfig:
    mode: str = "left_real"  # dual_sim | left_real | right_real
@dataclass(frozen=True)
class PoseInputConfig:
    node_name: str = "analytic_control"
    poses_topic: str = "/read_pose/poses"
    tracked_topic: str = "/read_pose/tracked"
    max_pose_age_s: float = 0.25
    disconnect_s: float = 0.5
@dataclass(frozen=True)
class FrameTransformConfig:
    translation_left_shoulder_to_left_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_left_shoulder_to_left_arm_base_rpy_deg: tuple[float, float, float] = (0.0, -90.0, 0.0)
    translation_right_shoulder_to_right_arm_base_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_right_shoulder_to_right_arm_base_rpy_deg: tuple[float, float, float] = (0.0, -90.0, 180.0)
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
    local_theta0_window: float = 0.7
    local_theta0_count: int = 50
    w_vel: float = 0.0
    w_acc: float = 0.0
    w_pose: float = 0.7
    w_theta0: float = 0.1
    w_limit: float = 0.4
    branch_switch_max_joint_delta_rad: float = 0.2
    branch_switch_min_improvement: float = 0.5
    reach_max_ratio: float = 0.95
    reach_min_ratio: float = 1.05
    enable_initial_global_search: bool = True
    enable_global_fallback: bool = False
@dataclass(frozen=True)
class OneEuroConfig:
    enabled: bool = True
    min_cutoff: float = 1.2
    beta: float = 0.03
    d_cutoff: float = 1.0
@dataclass(frozen=True)
class InputLimiterConfig:
    max_linear_speed_m_s: float = 1.0
    max_angular_speed_rad_s: float = 4.0
    max_position_step_m: float = 0.04
    max_rotation_step_rad: float = 0.16
@dataclass(frozen=True)
class FilterConfig:
    pose: OneEuroConfig = field(default_factory=OneEuroConfig)
    ik_output: OneEuroConfig = field(default_factory=OneEuroConfig)
@dataclass(frozen=True)
class StartupConfig:
    left_pose_rad: tuple[float, ...] = (1.328929, 1.421239, -0.917729, 1.662461, -0.391128, 0.455112, -0.159994)
    enable_sim_pose: bool = True
    enable_real_move_j: bool = True
@dataclass(frozen=True)
class MotionConfig:
    command_hz: float = 50.0
    max_joint_step_rad: float = 0.02
    max_velocity_rad_s: tuple[float, ...] = (0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2)
    max_acceleration_rad_s2: tuple[float, ...] = (2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0)
    joint_limit_margin_rad: float = 0.035
@dataclass(frozen=True)
class SimConfig:
    visual_z_offset_m: float = 0.6
    joint_log_period_s: float = 1.0
@dataclass(frozen=True)
class RealArmConfig:
    sdk_path: str = "third_party/pyAgxArm"
    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = 1_000_000
    speed_percent: int = 20
    startup_timeout_s: float = 3.0
    enable_timeout_s: float = 5.0
    sdk_joint_limits_enabled: bool = False
@dataclass(frozen=True)
class ControlConfig:
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    pose_input: PoseInputConfig = field(default_factory=PoseInputConfig)
    frames: FrameTransformConfig = field(default_factory=FrameTransformConfig)
    ik: IkConfig = field(default_factory=IkConfig)
    input_limit: InputLimiterConfig = field(default_factory=InputLimiterConfig)
    filters: FilterConfig = field(default_factory=FilterConfig)
    startup: StartupConfig = field(default_factory=StartupConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    real: RealArmConfig = field(default_factory=RealArmConfig)
