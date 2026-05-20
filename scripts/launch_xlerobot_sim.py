"""Minimal XLeRobot sim launcher — opens MuJoCo viewer directly.

On macOS, must be run with mjpython (not python):
    mjpython scripts/launch_xlerobot_sim.py

mjpython is in the same venv bin as python:  .venv/bin/mjpython
Also requires GLFW3:  brew install glfw
"""

import sys
from pathlib import Path

import mujoco
import mujoco.viewer

SCENE = Path(__file__).resolve().parent.parent / "data/xlerobot_description/mujoco/scene.xml"


def main() -> None:
    print(f"Loading: {SCENE}")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    print(f"OK — {model.njnt} joints, {model.nu} actuators, {model.ncam} cameras")
    print("Actuators:", [model.actuator(i).name for i in range(model.nu)])
    print("Cameras:  ", [model.camera(i).name for i in range(model.ncam)])
    print()
    print("Tip: expand 'Control' in the right panel to drag joint sliders.")
    print("     Double-click any body to track it with the camera.")

    # Set a neutral home pose for both arms (slightly raised)
    arm_joints = ["Pitch_R", "Elbow_R", "Pitch_L", "Elbow_L"]
    for name in arm_joints:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id >= 0:
            data.ctrl[act_id] = 0.8  # Slightly raised elbow/pitch

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Viewer open. Close window to exit.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
