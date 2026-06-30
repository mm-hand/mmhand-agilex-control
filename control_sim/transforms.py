from __future__ import annotations

import math

import numpy as np


def normalize_quat_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is too small")
    return quat / norm


def rotation_matrix_from_quat_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(quat_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_matrix_x(angle_rad: float) -> np.ndarray:
    cos_angle, sin_angle = math.cos(angle_rad), math.sin(angle_rad)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cos_angle, -sin_angle], [0.0, sin_angle, cos_angle]],
        dtype=float,
    )


def rotation_matrix_y(angle_rad: float) -> np.ndarray:
    cos_angle, sin_angle = math.cos(angle_rad), math.sin(angle_rad)
    return np.array(
        [[cos_angle, 0.0, sin_angle], [0.0, 1.0, 0.0], [-sin_angle, 0.0, cos_angle]],
        dtype=float,
    )


def rotation_matrix_z(angle_rad: float) -> np.ndarray:
    cos_angle, sin_angle = math.cos(angle_rad), math.sin(angle_rad)
    return np.array(
        [[cos_angle, -sin_angle, 0.0], [sin_angle, cos_angle, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def rotation_matrix_from_rpy_rad(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    return rotation_matrix_z(yaw_rad) @ rotation_matrix_y(pitch_rad) @ rotation_matrix_x(roll_rad)


def rotation_matrix_from_rpy_deg(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    roll_rad, pitch_rad, yaw_rad = (math.radians(value) for value in rpy_deg)
    return rotation_matrix_from_rpy_rad(roll_rad, pitch_rad, yaw_rad)


def transform_from_rotation_and_translation(
    rotation_target_from_source: np.ndarray,
    translation_target_from_source_m: tuple[float, float, float] | np.ndarray,
) -> np.ndarray:
    transform_target_from_source = np.eye(4, dtype=float)
    transform_target_from_source[:3, :3] = np.asarray(rotation_target_from_source, dtype=float).reshape(3, 3)
    transform_target_from_source[:3, 3] = np.asarray(translation_target_from_source_m, dtype=float).reshape(3)
    return transform_target_from_source


def transform_from_translation(
    translation_target_from_source_m: tuple[float, float, float] | np.ndarray,
) -> np.ndarray:
    return transform_from_rotation_and_translation(np.eye(3, dtype=float), translation_target_from_source_m)


def invert_transform(transform_target_from_source: np.ndarray) -> np.ndarray:
    transform_target_from_source = np.asarray(transform_target_from_source, dtype=float).reshape(4, 4)
    rotation_target_from_source = transform_target_from_source[:3, :3]
    translation_target_from_source = transform_target_from_source[:3, 3]
    transform_source_from_target = np.eye(4, dtype=float)
    transform_source_from_target[:3, :3] = rotation_target_from_source.T
    transform_source_from_target[:3, 3] = -(rotation_target_from_source.T @ translation_target_from_source)
    return transform_source_from_target


def assert_rotation_matrix(rotation_target_from_source: np.ndarray, atol: float = 1e-6) -> None:
    rotation = np.asarray(rotation_target_from_source, dtype=float).reshape(3, 3)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError("rotation matrix is not orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not np.isclose(determinant, 1.0, atol=atol):
        raise ValueError(f"rotation matrix determinant is {determinant}, expected +1")
