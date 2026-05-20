# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Keyboard-based cartesian teleop module for arm teleoperation.

Wraps a pygame UI as a DimOS Module so it can be composed with coordinator
blueprints via autoconnect.

Keyboard controls:
    W/S: +X/-X (forward/backward)
    A/D: -Y/+Y (left/right)
    Q/E: +Z/-Z (up/down)
    R/F: +Roll/-Roll
    T/G: +Pitch/-Pitch
    Y/H: +Yaw/-Yaw
    SPACE: Reset to home pose
    ESC: Quit
"""

import os
import subprocess as _subprocess
import threading
import time
from typing import Any

from pydantic import field_validator

import numpy as np

try:
    import pygame
except ImportError:
    pygame = None  # type: ignore[assignment]

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.examples.cartesian_ik_jogger import JogState
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

# Force X11 driver on Linux to avoid OpenGL threading issues (not needed on macOS)
import sys as _sys
if _sys.platform != "darwin":
    os.environ["SDL_VIDEODRIVER"] = "x11"

# Jog speeds
LINEAR_SPEED = 0.05  # m/s
ANGULAR_SPEED = 0.5  # rad/s

# Workspace bounds
X_LIMITS = (-0.5, 0.5)
Y_LIMITS = (-0.5, 0.5)
Z_LIMITS = (-0.2, 0.6)


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def _run_pygame_subprocess(model_path: str, ee_joint_id: int, task_name: str) -> None:
    """Spawn target for macOS: runs pygame on the subprocess main thread.

    Creates its own LCM transport and publishes directly — same wire format
    as KeyboardTeleopModule._pygame_loop but self-contained.
    """
    import time as _time

    import numpy as _np
    import pygame as _pygame

    from dimos.control.examples.cartesian_ik_jogger import JogState as _JogState
    from dimos.core.transport import LCMTransport as _LCMTransport
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as _PoseStamped

    transport: _LCMTransport[_PoseStamped] = _LCMTransport(
        "/coordinator/cartesian_command", _PoseStamped
    )

    home_pose = _JogState.from_fk(model_path, ee_joint_id)
    current_pose = home_pose.copy()
    transport.publish(current_pose.to_pose_stamped(task_name))

    _pygame.init()
    screen = _pygame.display.set_mode((600, 400), _pygame.SWSURFACE)
    _pygame.display.set_caption(f"Keyboard Teleop — {task_name}")
    font = _pygame.font.Font(None, 28)
    clock = _pygame.time.Clock()
    last_time = _time.perf_counter()

    running = True
    while running:
        dt = _time.perf_counter() - last_time
        last_time = _time.perf_counter()

        for event in _pygame.event.get():
            if event.type == _pygame.QUIT:
                running = False
            elif event.type == _pygame.KEYDOWN:
                if event.key == _pygame.K_ESCAPE:
                    running = False
                elif event.key == _pygame.K_SPACE:
                    current_pose = home_pose.copy()

        keys = _pygame.key.get_pressed()
        if keys[_pygame.K_w]: current_pose.x += LINEAR_SPEED * dt
        if keys[_pygame.K_s]: current_pose.x -= LINEAR_SPEED * dt
        if keys[_pygame.K_a]: current_pose.y -= LINEAR_SPEED * dt
        if keys[_pygame.K_d]: current_pose.y += LINEAR_SPEED * dt
        if keys[_pygame.K_q]: current_pose.z += LINEAR_SPEED * dt
        if keys[_pygame.K_e]: current_pose.z -= LINEAR_SPEED * dt
        if keys[_pygame.K_r]: current_pose.roll += ANGULAR_SPEED * dt
        if keys[_pygame.K_f]: current_pose.roll -= ANGULAR_SPEED * dt
        if keys[_pygame.K_t]: current_pose.pitch += ANGULAR_SPEED * dt
        if keys[_pygame.K_g]: current_pose.pitch -= ANGULAR_SPEED * dt
        if keys[_pygame.K_y]: current_pose.yaw += ANGULAR_SPEED * dt
        if keys[_pygame.K_h]: current_pose.yaw -= ANGULAR_SPEED * dt

        current_pose.x = _clamp(current_pose.x, *X_LIMITS)
        current_pose.y = _clamp(current_pose.y, *Y_LIMITS)
        current_pose.z = _clamp(current_pose.z, *Z_LIMITS)

        transport.publish(current_pose.to_pose_stamped(task_name))

        screen.fill((30, 30, 30))
        y_pos = 20
        screen.blit(font.render(f"Keyboard Teleop — {task_name}", True, (255, 255, 255)), (20, y_pos)); y_pos += 40
        screen.blit(font.render(f"Position: X={current_pose.x:.3f}  Y={current_pose.y:.3f}  Z={current_pose.z:.3f}", True, (100, 255, 100)), (20, y_pos)); y_pos += 30
        screen.blit(font.render(f"Orientation: R={_np.degrees(current_pose.roll):.1f}°  P={_np.degrees(current_pose.pitch):.1f}°  Y={_np.degrees(current_pose.yaw):.1f}°", True, (100, 200, 255)), (20, y_pos)); y_pos += 40
        for key, desc in [("W/S","+X/-X"),("A/D","-Y/+Y"),("Q/E","+Z/-Z"),("R/F","+Roll/-Roll"),("T/G","+Pitch/-Pitch"),("Y/H","+Yaw/-Yaw"),("SPACE","Reset"),("ESC","Quit")]:
            screen.blit(font.render(f"{key}: {desc}", True, (180, 180, 180)), (20, y_pos)); y_pos += 25
        _pygame.display.flip()
        clock.tick(50)

    _pygame.quit()


class KeyboardTeleopConfig(ModuleConfig):
    model_path: str = ""
    ee_joint_id: int = 6
    task_name: str = "cartesian_ik_arm"

    @field_validator("model_path", mode="before")
    @classmethod
    def coerce_model_path(cls, v: Any) -> str:
        return str(v)


class KeyboardTeleopModule(Module):
    """Pygame-based cartesian keyboard teleop as a DimOS Module.

    Publishes absolute EE PoseStamped commands for CartesianIKTask.
    """

    config: KeyboardTeleopConfig

    cartesian_command: Out[PoseStamped]

    _stop_event: threading.Event
    _thread: threading.Thread | None = None
    _process: _subprocess.Popen | None = None  # type: ignore[type-arg]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._process = None

    @rpc
    def start(self) -> None:
        super().start()
        if pygame is None:
            raise ImportError("pygame not installed. Install with: pip install pygame")

        self._stop_event.clear()
        if _sys.platform == "darwin":
            # Daemon worker processes cannot spawn multiprocessing children (Python restriction).
            # subprocess.Popen bypasses this via OS-level fork+exec.
            self._process = _subprocess.Popen([
                _sys.executable, "-m",
                "dimos.teleop.keyboard.keyboard_teleop_module",
                str(self.config.model_path),
                str(self.config.ee_joint_id),
                self.config.task_name,
            ])
        else:
            self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
            self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            except _subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _pygame_loop(self) -> None:
        model_path = str(self.config.model_path)
        ee_joint_id = self.config.ee_joint_id
        task_name = self.config.task_name

        # Initialize pose from forward kinematics at zero configuration
        home_pose = JogState.from_fk(model_path, ee_joint_id)
        current_pose = home_pose.copy()

        # Publish initial pose
        self.cartesian_command.publish(current_pose.to_pose_stamped(task_name))

        pygame.init()
        screen = pygame.display.set_mode((600, 400), pygame.SWSURFACE)
        pygame.display.set_caption(f"Keyboard Teleop — {task_name}")
        font = pygame.font.Font(None, 28)
        clock = pygame.time.Clock()
        last_time = time.perf_counter()

        while not self._stop_event.is_set():
            dt = time.perf_counter() - last_time
            last_time = time.perf_counter()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop_event.set()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self._stop_event.set()
                    elif event.key == pygame.K_SPACE:
                        current_pose = home_pose.copy()

            keys = pygame.key.get_pressed()

            # Linear motion
            if keys[pygame.K_w]:
                current_pose.x += LINEAR_SPEED * dt
            if keys[pygame.K_s]:
                current_pose.x -= LINEAR_SPEED * dt
            if keys[pygame.K_a]:
                current_pose.y -= LINEAR_SPEED * dt
            if keys[pygame.K_d]:
                current_pose.y += LINEAR_SPEED * dt
            if keys[pygame.K_q]:
                current_pose.z += LINEAR_SPEED * dt
            if keys[pygame.K_e]:
                current_pose.z -= LINEAR_SPEED * dt

            # Angular motion
            if keys[pygame.K_r]:
                current_pose.roll += ANGULAR_SPEED * dt
            if keys[pygame.K_f]:
                current_pose.roll -= ANGULAR_SPEED * dt
            if keys[pygame.K_t]:
                current_pose.pitch += ANGULAR_SPEED * dt
            if keys[pygame.K_g]:
                current_pose.pitch -= ANGULAR_SPEED * dt
            if keys[pygame.K_y]:
                current_pose.yaw += ANGULAR_SPEED * dt
            if keys[pygame.K_h]:
                current_pose.yaw -= ANGULAR_SPEED * dt

            # Clamp to workspace limits
            current_pose.x = _clamp(current_pose.x, *X_LIMITS)
            current_pose.y = _clamp(current_pose.y, *Y_LIMITS)
            current_pose.z = _clamp(current_pose.z, *Z_LIMITS)

            # Publish
            self.cartesian_command.publish(current_pose.to_pose_stamped(task_name))

            # Draw UI
            screen.fill((30, 30, 30))
            y_pos = 20

            title = font.render(f"Keyboard Teleop — {task_name}", True, (255, 255, 255))
            screen.blit(title, (20, y_pos))
            y_pos += 40

            pos_text = (
                f"Position: X={current_pose.x:.3f}  Y={current_pose.y:.3f}  Z={current_pose.z:.3f}"
            )
            screen.blit(font.render(pos_text, True, (100, 255, 100)), (20, y_pos))
            y_pos += 30

            ori_text = (
                f"Orientation: R={np.degrees(current_pose.roll):.1f}°  "
                f"P={np.degrees(current_pose.pitch):.1f}°  "
                f"Y={np.degrees(current_pose.yaw):.1f}°"
            )
            screen.blit(font.render(ori_text, True, (100, 200, 255)), (20, y_pos))
            y_pos += 40

            controls = [
                ("W/S", "+X/-X (forward/back)"),
                ("A/D", "-Y/+Y (left/right)"),
                ("Q/E", "+Z/-Z (up/down)"),
                ("R/F", "+Roll/-Roll"),
                ("T/G", "+Pitch/-Pitch"),
                ("Y/H", "+Yaw/-Yaw"),
                ("SPACE", "Reset to home"),
                ("ESC", "Quit"),
            ]
            for key, desc in controls:
                screen.blit(font.render(f"{key}: {desc}", True, (180, 180, 180)), (20, y_pos))
                y_pos += 25

            pygame.display.flip()
            clock.tick(50)

        pygame.quit()


if __name__ == "__main__":
    import sys as _main_sys
    _run_pygame_subprocess(_main_sys.argv[1], int(_main_sys.argv[2]), _main_sys.argv[3])
