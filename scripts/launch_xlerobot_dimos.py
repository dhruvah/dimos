"""Launch XLeRobot in dimos with camera streaming + joint control.

Run from repo root:  python scripts/launch_xlerobot_dimos.py
"""

from dimos.robot.xlerobot.blueprints import xlerobot_basic_sim

if __name__ == "__main__":
    xlerobot_basic_sim.run()
