from __future__ import annotations

import ast
from pathlib import Path

UI = Path(__file__).resolve().parent / "ui"
FORBIDDEN = (
    "Return to Initial Pose",
    "Pause Teleop",
    "Sync Real Arm",
    "Base pose",
    "Iso",
    "Top",
    "Front",
    "Reset View",
)


def test_ui_strings_are_english_and_hide_removed_controls():
    strings = []
    for path in UI.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        strings.extend(
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        )
    joined = "\n".join(strings)
    assert not any("\u4e00" <= char <= "\u9fff" for char in joined)
    for text in FORBIDDEN:
        assert text not in joined
