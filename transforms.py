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
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w), 2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w), 1.0 - 2.0 * (x*x + y*y)],
    ], dtype=float)
def rotation_matrix_from_rpy_deg(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    r, p, y = (math.radians(v) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ], dtype=float)
def transform_from_rotation_and_translation(rotation: np.ndarray, translation_m) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    t[:3, 3] = np.asarray(translation_m, dtype=float).reshape(3)
    return t
def transform_from_translation(translation_m) -> np.ndarray:
    return transform_from_rotation_and_translation(np.eye(3, dtype=float), translation_m)
def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    r, p = transform[:3, :3], transform[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ p)
    return inv
