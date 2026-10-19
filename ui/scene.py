from __future__ import annotations

import numpy as np

from config import ControlConfig
from frames import arm_base_in_horizon
from ik import _T_LINK7_TO_FLANGE
from ik_solver import NeroParams

_SCENE_FROM_HORIZON = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
_EYE = np.eye(4, dtype=float)
_EYE[:3, :3] = _SCENE_FROM_HORIZON


def joint_frames(q: np.ndarray) -> list[np.ndarray]:
    """Return the eight link frames (base plus seven joints) for joint angles q."""
    q = np.asarray(q, dtype=float).reshape(7)
    params = NeroParams.default()
    frames = [np.eye(4, dtype=float)]
    for index in range(7):
        theta = q[index] + float(params.theta_offset[index])
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(params.alpha_prev[index]), np.sin(params.alpha_prev[index])
        d = float(params.d_i[index])
        a = float(params.a_prev[index])
        transform = np.array(
            [
                [ct, -st, 0.0, a],
                [ca * st, ca * ct, -sa, -sa * d],
                [sa * st, sa * ct, ca, ca * d],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        frames.append(frames[-1] @ transform)
    return frames


def _align_y(direction: np.ndarray) -> np.ndarray:
    y = np.array([0.0, 1.0, 0.0])
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return np.eye(3)
    d = direction / norm
    dot = float(np.dot(y, d))
    if dot < -0.999999:
        return np.diag([1.0, -1.0, -1.0])
    cross = np.cross(y, d)
    skew = np.array([[0.0, -cross[2], cross[1]], [cross[2], 0.0, -cross[0]], [-cross[1], cross[0], 0.0]])
    return np.eye(3) + skew + skew @ skew * (1.0 / (1.0 + dot))


def _scene_transform(base: np.ndarray, frame: np.ndarray) -> np.ndarray:
    return _EYE @ base @ frame


class _ArmVisual:
    def __init__(self, scene, base: np.ndarray, color: str, opacity: float) -> None:
        self.base = base
        self.color = color
        self.opacity = opacity
        self.ghost = opacity < 1.0
        self.cylinders = []
        self.joints = []
        self.flange = None
        self.axes = None
        self.visible = True
        zero_frames = joint_frames(np.zeros(7))
        with scene:
            for index in range(7):
                start = _scene_transform(base, zero_frames[index])
                end = _scene_transform(base, zero_frames[index + 1])
                length = float(np.linalg.norm(end[:3, 3] - start[:3, 3]))
                self.cylinders.append(scene.cylinder(0.025, 0.025, max(0.01, length), 10))
            for _ in range(8):
                self.joints.append(scene.sphere(0.022, 10, 8))
            self.flange = scene.box(0.055, 0.055, 0.02)
            if self.ghost:
                self.axes = None
            else:
                self.axes = scene.axes_helper(0.12)
        for item in [*self.cylinders, *self.joints]:
            item.material(color, opacity)
        self.flange.material(color, opacity)

    def update(self, q: np.ndarray) -> None:
        frames = joint_frames(q)
        points = [_scene_transform(self.base, frame) for frame in frames]
        points.append(_scene_transform(self.base, frames[-1] @ _T_LINK7_TO_FLANGE))
        for index, cylinder in enumerate(self.cylinders):
            start, end = points[index][:3, 3], points[index + 1][:3, 3]
            direction = end - start
            length = float(np.linalg.norm(direction))
            cylinder.move(*(start + direction * 0.5))
            cylinder.rotate_R(_align_y(direction).tolist())
            cylinder.scale(1.0, max(0.01, length) / max(0.01, cylinder.args[2]), 1.0)
        for index, sphere in enumerate(self.joints):
            sphere.move(*points[index][:3, 3])
        flange = points[-1]
        self.flange.move(*flange[:3, 3])
        self.flange.rotate_R(flange[:3, :3].tolist())
        if self.axes is not None:
            base = points[0]
            self.axes.move(*base[:3, 3])
            self.axes.rotate_R(base[:3, :3].tolist())

    def set_visible(self, visible: bool) -> None:
        if visible == self.visible:
            return
        self.visible = visible
        for item in [*self.cylinders, *self.joints, self.flange]:
            item.visible(visible)
        if self.axes is not None:
            self.axes.visible(visible)


class TeleopScene:
    def __init__(self, scene, config: ControlConfig) -> None:
        bases = arm_base_in_horizon(config.frames)
        self.scene = scene
        self.left = _ArmVisual(scene, bases.left, "#2563eb", 1.0)
        self.right = _ArmVisual(scene, bases.right, "#f97316", 1.0)
        self.left_ghost = _ArmVisual(scene, bases.left, "#16a34a", 0.25)
        self.right_ghost = _ArmVisual(scene, bases.right, "#16a34a", 0.25)
        self.left_ghost.set_visible(False)
        self.right_ghost.set_visible(False)

    def update(self, snapshot) -> None:
        with self.scene._batch_calls():
            self.left.update(snapshot.sim_q["left"])
            self.right.update(snapshot.sim_q["right"])
            for side, ghost in (("left", self.left_ghost), ("right", self.right_ghost)):
                real = snapshot.real_q.get(side)
                if real is None:
                    ghost.set_visible(False)
                else:
                    ghost.set_visible(True)
                    ghost.update(real)

    def resize(self) -> None:
        self.scene.run_method("resize")
