"""XLeRobot catalog entry for dimos."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.robot.config import GripperConfig, RobotConfig

_DIMOS_ROOT = Path(__file__).resolve().parent.parent.parent.parent
XLEROBOT_SIM_PATH = _DIMOS_ROOT / "data" / "xlerobot_description" / "mujoco" / "scene.xml"

_RIGHT_ARM_JOINTS = ["Rotation_R", "Pitch_R", "Elbow_R", "Wrist_Pitch_R", "Wrist_Roll_R"]
_LEFT_ARM_JOINTS = ["Rotation_L", "Pitch_L", "Elbow_L", "Wrist_Pitch_L", "Wrist_Roll_L"]


def xlerobot_right_arm(
    name: str = "xlerobot",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """XLeRobot right arm (SO-ARM101, 5-DOF + gripper) on wheeled mobile base.

    Note: model_path points to the MJCF scene. URDF/Drake planning not yet
    available — convert MJCF to URDF when ManipulationModule integration needed.
    """
    addr = address or str(XLEROBOT_SIM_PATH)
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": XLEROBOT_SIM_PATH,
        "end_effector_link": "Fixed_Jaw_2",
        "adapter_type": adapter_type,
        "address": addr,
        "joint_names": _RIGHT_ARM_JOINTS,
        "base_link": "chassis",
        "home_joints": [0.0, 0.5, 0.5, 0.0, 0.0],
        "base_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "package_paths": {},
        "xacro_args": {},
        "auto_convert_meshes": False,
        "gripper": GripperConfig(
            type="xlerobot",
            joints=["Jaw_R"],
            open_position=1.7453,
            close_position=-0.3745,
        ),
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)
