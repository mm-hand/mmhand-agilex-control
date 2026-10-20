from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from config import ControlConfig
from frames import arm_base_in_horizon
from transforms import rotation_matrix_from_rpy_deg, transform_from_rotation_and_translation

ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT / "nero_urdf" / "nero.urdf"
MESH_DIR = ROOT / "nero_urdf" / "meshes"
SCENE_FROM_HORIZON = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
SKIP_JOINTS = {"revo2_flange_joint"}

_EYE = np.eye(4, dtype=float)
_EYE[:3, :3] = SCENE_FROM_HORIZON


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    parent: str
    child: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass(frozen=True)
class UrdfLink:
    name: str
    mesh: Path | None
    visual_origin: np.ndarray
    children: tuple[str, ...]


@dataclass(frozen=True)
class UrdfModel:
    base_link: str
    links: dict[str, UrdfLink]
    joints: dict[str, UrdfJoint]
    revolute_joints: tuple[str, ...]

    @classmethod
    def load(cls, path: Path = URDF_PATH) -> "UrdfModel":
        root = ET.parse(path).getroot()
        joint_elements = root.findall("joint")
        joints: dict[str, UrdfJoint] = {}
        child_links = set()
        for element in joint_elements:
            name = element.get("name", "")
            parent = element.find("parent").get("link")
            child = element.find("child").get("link")
            kind = element.get("type", "fixed")
            origin = _origin(element.find("origin"))
            axis_element = element.find("axis")
            axis = _vector(axis_element.get("xyz")) if axis_element is not None else np.array([0.0, 0.0, 1.0])
            joints[name] = UrdfJoint(name, parent, child, kind, origin, axis)
            child_links.add(child)

        children: dict[str, list[str]] = {}
        for joint in joints.values():
            children.setdefault(joint.parent, []).append(joint.name)

        links: dict[str, UrdfLink] = {}
        for element in root.findall("link"):
            name = element.get("name", "")
            visual = element.find("visual")
            mesh = None
            visual_origin = np.eye(4, dtype=float)
            if visual is not None:
                origin = visual.find("origin")
                if origin is not None:
                    visual_origin = _origin(origin)
                mesh_element = visual.find("geometry/mesh")
                if mesh_element is not None:
                    mesh = _mesh_path(path.parent, mesh_element.get("filename", ""))
            links[name] = UrdfLink(name, mesh, visual_origin, tuple(children.get(name, ())))

        base = next(name for name in links if name not in child_links)
        order: list[str] = []
        queue = [base]
        while queue:
            parent = queue.pop(0)
            for joint_name in links[parent].children:
                joint = joints[joint_name]
                if joint.name not in SKIP_JOINTS:
                    queue.append(joint.child)
                    if joint.kind != "fixed":
                        order.append(joint.name)
        return cls(base, links, joints, tuple(order))


def _vector(text: str) -> np.ndarray:
    return np.array([float(value) for value in text.split()], dtype=float)


def _origin(element) -> np.ndarray:
    if element is None:
        return np.eye(4, dtype=float)
    xyz = _vector(element.get("xyz", "0 0 0"))
    rpy = _vector(element.get("rpy", "0 0 0"))
    rotation = rotation_matrix_from_rpy_deg(tuple(math.degrees(value) for value in rpy))
    return transform_from_rotation_and_translation(rotation, xyz)


def _mesh_path(urdf_dir: Path, filename: str) -> Path:
    path = (urdf_dir / filename).resolve()
    if path.suffix.lower() != ".stl":
        path = path.parent.parent / f"{path.stem}.stl" if path.parent.name == "dae" else path.with_suffix(".stl")
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh not found: {path}")
    return path


def _axis_transform(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    one = 1.0 - cosine
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = [
        [x * x * one + cosine, x * y * one - z * sine, x * z * one + y * sine],
        [y * x * one + z * sine, y * y * one + cosine, y * z * one - x * sine],
        [z * x * one - y * sine, z * y * one + x * sine, z * z * one + cosine],
    ]
    return transform


def link_frames(model: UrdfModel, q: np.ndarray) -> dict[str, np.ndarray]:
    q = np.asarray(q, dtype=float).reshape(7)
    frames = {model.base_link: np.eye(4, dtype=float)}
    index = 0
    queue = [model.base_link]
    while queue:
        parent = queue.pop(0)
        for joint_name in model.links[parent].children:
            joint = model.joints[joint_name]
            transform = frames[parent] @ joint.origin
            if joint.kind != "fixed":
                transform = transform @ _axis_transform(joint.axis, q[index])
                index += 1
            frames[joint.child] = transform
            if joint.name not in SKIP_JOINTS:
                queue.append(joint.child)
    return frames


def _scene_transform(base: np.ndarray) -> np.ndarray:
    return _EYE @ base


class _UrdfArm:
    def __init__(self, scene, model: UrdfModel, base: np.ndarray, color: str, opacity: float) -> None:
        self.model = model
        self.color = color
        self.opacity = opacity
        self.meshes = []
        self.rotation_groups = {}
        self.visible = True
        with scene:
            self.base_group = scene.group()
        transform = _scene_transform(base)
        self.base_group.move(*transform[:3, 3])
        self.base_group.rotate_R(transform[:3, :3].tolist())
        self._build_link(scene, model.base_link, self.base_group)
        if opacity >= 1.0:
            with self.base_group:
                self.axes = scene.axes_helper(0.12)
        else:
            self.axes = None
        for mesh in self.meshes:
            mesh.material(color, opacity)

    def _build_link(self, scene, link_name: str, parent) -> None:
        link = self.model.links[link_name]
        self._add_visual(scene, parent, link)
        for joint_name in link.children:
            if joint_name in SKIP_JOINTS:
                continue
            joint = self.model.joints[joint_name]
            with parent:
                origin_group = scene.group()
            origin_group.move(*joint.origin[:3, 3])
            origin_group.rotate_R(joint.origin[:3, :3].tolist())
            if joint.kind == "fixed":
                self._build_link(scene, joint.child, origin_group)
            else:
                with origin_group:
                    rotation_group = scene.group()
                self.rotation_groups[joint.name] = rotation_group
                self._build_link(scene, joint.child, rotation_group)

    def _add_visual(self, scene, parent, link: UrdfLink) -> None:
        if link.mesh is None:
            return
        target = parent
        if not np.allclose(link.visual_origin, np.eye(4)):
            with parent:
                target = scene.group()
            target.move(*link.visual_origin[:3, 3])
            target.rotate_R(link.visual_origin[:3, :3].tolist())
        with target:
            mesh = scene.stl(f"/nero_meshes/{link.mesh.name}")
        self.meshes.append(mesh)

    def update(self, q: np.ndarray) -> None:
        for index, joint_name in enumerate(self.model.revolute_joints):
            self.rotation_groups[joint_name].rotate_R(
                _axis_transform(self.model.joints[joint_name].axis, q[index])[:3, :3].tolist()
            )

    def set_visible(self, visible: bool) -> None:
        if visible == self.visible:
            return
        self.visible = visible
        for mesh in self.meshes:
            mesh.visible(visible)
        if self.axes is not None:
            self.axes.visible(visible)


class TeleopScene:
    def __init__(self, scene, config: ControlConfig) -> None:
        self.scene = scene
        self.model = UrdfModel.load()
        bases = arm_base_in_horizon(config.frames)
        self.left = _UrdfArm(scene, self.model, bases.left, "#2563eb", 1.0)
        self.right = _UrdfArm(scene, self.model, bases.right, "#f97316", 1.0)
        self.left_ghost = None
        self.right_ghost = None
        self.bases = bases

    def update(self, snapshot) -> None:
        with self.scene._batch_calls():
            self.left.update(snapshot.sim_q["left"])
            self.right.update(snapshot.sim_q["right"])
            for side in ("left", "right"):
                real = snapshot.real_q.get(side)
                ghost = self.left_ghost if side == "left" else self.right_ghost
                if real is None:
                    if ghost is not None:
                        ghost.set_visible(False)
                    continue
                if ghost is None:
                    base = self.bases.left if side == "left" else self.bases.right
                    ghost = _UrdfArm(self.scene, self.model, base, "#16a34a", 0.25)
                    if side == "left":
                        self.left_ghost = ghost
                    else:
                        self.right_ghost = ghost
                ghost.set_visible(True)
                ghost.update(real)

    def resize(self) -> None:
        self.scene.run_method("resize")


def build_scene(config: ControlConfig):
    from nicegui import ui

    with ui.element("div").classes("sim-viewport w-full h-full"):
        scene = ui.scene(grid=False, background_color="#f8fafc", fps=10).classes("w-full h-full")
        scene.move_camera(
            x=1.7,
            y=-2.3,
            z=1.8,
            look_at_x=0.0,
            look_at_y=0.0,
            look_at_z=0.65,
            duration=0.0,
        )
        with ui.element("div").classes("scene-legend"):
            ui.label("Left Arm").classes("legend-left")
            ui.label("Right Arm").classes("legend-right")
            ui.label("Real Feedback").classes("legend-real")
    visual = TeleopScene(scene, config)

    def refresh(snapshot) -> None:
        visual.update(snapshot)

    return refresh, visual
