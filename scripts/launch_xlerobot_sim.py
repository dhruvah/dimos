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
    print("Cameras:", [model.camera(i).name for i in range(model.ncam)])

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Viewer open. Close window to exit.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
