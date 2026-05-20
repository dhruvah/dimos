"""Launch XLeRobot in dimos with camera streaming + joint control.

Run from repo root:  .venv/bin/mjpython scripts/launch_xlerobot_dimos.py

For the production run path (with watchdog, run registry, daemonization),
use ``dimos run xlerobot_basic_sim`` instead. This script is a minimal
development launcher.
"""

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.xlerobot.blueprints import xlerobot_basic_sim

if __name__ == "__main__":
    coordinator = ModuleCoordinator.build(xlerobot_basic_sim)
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.stop()
