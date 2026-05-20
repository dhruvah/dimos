"""dimos blueprints for XLeRobot simulation."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.catalog.xlerobot import XLEROBOT_SIM_PATH, xlerobot_right_arm
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_xlerobot_sim_cfg = xlerobot_right_arm(
    name="xlerobot",
    adapter_type="sim_mujoco",
    address=str(XLEROBOT_SIM_PATH),
)

# Basic sim: MuJoCo physics + wrist camera stream + right-arm joint control via SHM.
# ManipulationModule/Drake IK not included — requires URDF (convert from MJCF when needed).
# Note: full MJCF has 16 actuators; dof=5 exposes only the right arm through dimos.
# Other actuators (base wheels, left arm, head) hold home positions.
xlerobot_basic_sim = autoconnect(
    MujocoSimModule.blueprint(
        address=str(XLEROBOT_SIM_PATH),
        headless=False,
        dof=5,
        camera_name="wrist_camera",
        base_frame_id="Fixed_Jaw_2",
        width=640,
        height=480,
        fps=15,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xlerobot_sim_cfg.to_hardware_component()],
        tasks=[_xlerobot_sim_cfg.to_task_config()],
    ),
)
