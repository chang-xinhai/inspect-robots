"""Native Inspect Robots Cartesian embodiment over the shared DepthUMI service."""

from __future__ import annotations

import copy
import json
import math
import sys
import time
import uuid
from pathlib import Path

import numpy as np
import yaml

from inspect_robots.embodiment import EmbodimentBase, EmbodimentInfo
from inspect_robots.errors import ConfigError, EmbodimentFault
from inspect_robots.scene import Scene
from inspect_robots.spaces import (
    ActionSemantics,
    Box,
    CameraSpec,
    ObservationSpace,
    StateField,
    StateSpec,
)
from inspect_robots.types import Action, Observation, StepResult

from ._transport.camera import GeminiCamera, MockCamera
from ._transport.mock import MockRpcServer
from ._transport.robot import Fr3Robot, RpcWorker


class FR3Embodiment(EmbodimentBase):
    """Absolute xyz + aperture, preserving the initial TCP orientation.

    Each step is one acknowledged, measured-arrival waypoint. The upstream
    agent owns chunk interpolation; DepthUMI interpolates only each waypoint
    at its servo rate. control_hz is nominal; settling and RGB add latency.
    """

    def __init__(
        self,
        site_config="configs/site/fr3.yaml",
        host=None,
        port=None,
        period_s=0.3,
        action_timeout_s=15.0,
        step_m=0.01,
        position_tolerance_m=0.005,
        rotation_tolerance_rad=0.05,
        max_duration_s=1800.0,
        image_size=640,
        read_only=False,
        mock=False,
        z_floor_m=None,
        log_dir="logs/fr3",
    ):
        self.mock, self.read_only = mock, read_only
        for name, value in {
            "period_s": period_s,
            "action_timeout_s": action_timeout_s,
            "step_m": step_m,
            "max_duration_s": max_duration_s,
            "position_tolerance_m": position_tolerance_m,
            "rotation_tolerance_rad": rotation_tolerance_rad,
        }.items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ConfigError(f"{name} must be finite and positive")
        if not isinstance(image_size, int) or image_size < 1:
            raise ConfigError("image_size must be a positive integer")
        if z_floor_m is not None and not math.isfinite(z_floor_m):
            raise ConfigError("z_floor_m must be finite or None")
        path = Path(site_config).expanduser()
        if path.exists():
            self.site = yaml.safe_load(path.read_text())
        elif mock:
            self.site = {
                "robot": {},
                "camera": {},
                "servo": {
                    "T_policy_from_base": np.eye(4).tolist(),
                    "T_ee_from_tool": np.eye(4).tolist(),
                },
            }
        else:
            raise ConfigError(f"missing FR3 site config: {path}; run scripts/fr3/setup.sh")
        if not isinstance(self.site, dict) or not isinstance(self.site.get("servo"), dict):
            raise ConfigError("site config requires a servo mapping")
        for key in ("T_policy_from_base", "T_ee_from_tool"):
            matrix = np.asarray(self.site["servo"].get(key), dtype=float)
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise ConfigError(f"servo.{key} must be a finite 4x4 matrix")
        robot = self.site.get("robot", {})
        self.host = host or robot.get("host", "127.0.0.1")
        self.port = port or robot.get("port", 4242)
        self.period_s, self.step_m = period_s, step_m
        self.max_duration_s, self.image_size = max_duration_s, image_size
        self.z_floor_m = z_floor_m
        self.robot_args = {
            "period_s": period_s,
            "action_timeout_s": action_timeout_s,
            "position_tolerance_m": position_tolerance_m,
            "rotation_tolerance_rad": rotation_tolerance_rad,
            "read_only": read_only,
        }
        self.log_root = Path(log_dir)
        self.run_dir = None
        self.robot = self.camera = None
        self._last = None
        self._last_command = None
        self._orientation = None
        self._started = 0.0
        self._instruction = None
        # Numerical command envelope, NOT a measured collision-free workspace.
        safety = self.site["servo"].get("safety", {})
        low = list(safety.get("workspace_low_m") or [-1.0, -1.0, -1.0])
        high = list(safety.get("workspace_high_m") or [1.0, 1.0, 1.5])
        if z_floor_m is not None:
            low[2] = max(low[2], z_floor_m)
        space = Box(
            (4,),
            low=np.array([*low, 0.0]),
            high=np.array([*high, 0.08]),
            semantics=ActionSemantics(
                "eef_abs_pose",
                rotation_repr="none",
                gripper="continuous",
                frame="base",
                dim_labels=("x", "y", "z", "gripper_width"),
                max_step=(step_m, step_m, step_m, 0.08),
            ),
        )
        self.info = EmbodimentInfo(
            name="fr3",
            action_space=space,
            observation_space=ObservationSpace(
                cameras=(CameraSpec("wrist", image_size, image_size),),
                state=StateSpec(
                    (
                        StateField("eef_state", (4,), "m"),
                        StateField("tcp_quat", (4,), "unit_quat"),
                        StateField("gripper_width", (1,), "m"),
                    )
                ),
            ),
            control_hz=1 / period_s,
            is_simulated=bool(mock),
            capabilities=frozenset({"self_paced", "renderable"}),
            docs=(
                "FR3 with one wrist Gemini RGB camera. move_to targets x,y,z in "
                "the configured policy/base frame, in metres; image axes are NOT base axes. "
                "The initial TCP orientation is held fixed; no rotation tool is available. "
                "gripper_width is metres: 0 closes, 0.08 opens. eef_state uses the last "
                "commanded aperture to retain the grasp across Cartesian moves; the separate "
                "gripper_width state is measured aperture. Measured aperture alone "
                "does not prove a grasp; empty-grasp recovery can reopen the hand. "
                "Start from the manually prepared pose; reset never homes or opens the hand. "
                "Keep the cup upright. Use small motions, then inspect the changed view. "
                "Bounds are a numerical command envelope, not a collision-free workspace. "
                "No calibrated camera extrinsics or metric depth are available. "
                "Give up when the view cannot support a motion. "
                "Nominal playout time excludes measured settling and RGB acquisition."
            ),
        )

    def connect_operator_session(self, session):
        """Let the upstream console own input; this adapter never reads stdin."""

    def reset(self, scene: Scene, *, seed=None) -> Observation:
        """Acquire a fresh frame before starting an owned, current-pose session."""
        self.close()
        self._instruction = scene.instruction
        self.run_dir = self.log_root / (time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
        self.run_dir.mkdir(parents=True)
        (self.run_dir / "site.json").write_text(json.dumps(self.site, indent=2))
        try:
            self.camera = (
                MockCamera()
                if self.mock
                else GeminiCamera(
                    sys.executable, self.run_dir / "camera.log", **self.site.get("camera", {})
                )
            )
            rpc = RpcWorker(
                self.host, self.port, client_factory=MockRpcServer if self.mock else None
            )
            self.robot = Fr3Robot(rpc, copy.deepcopy(self.site["servo"]), **self.robot_args)
            snapshot = self.robot.preflight()
            state_age = snapshot.get("state_age_s", 0.0)
            if not np.isfinite(state_age) or state_age > self.site["servo"].get(
                "maximum_state_age_s", 0.1
            ):
                raise EmbodimentFault(
                    "FR3 state is stale; inspect launch_comm and robot connection"
                )
            if snapshot.get("control_active") and not self.read_only:
                raise EmbodimentFault("FR3 already controlled; stop the other baseline first")
            self.robot.start()
            self._started = time.monotonic()
            observation = self.observe()
            self._orientation = observation.state["tcp_quat"].copy()
            self._last = observation.state["eef_state"].copy()
            self._last_command = self._last.copy()
            return observation
        except BaseException:
            self.close()
            raise

    def observe(self, *, after_ns=None) -> Observation:
        """Read measured TCP and a fresh RGB frame without commanding motion."""
        import cv2

        from ._transport.robot import pose6_to_pose7

        rgb, packet = self.camera.snapshot(after_ns=after_ns)
        state = self.robot.state()
        stamp = time.monotonic()
        pose = pose6_to_pose7(state["pose"])
        height, width = rgb.shape[:2]
        scale = min(self.image_size / height, self.image_size / width)
        resized = cv2.resize(rgb, (round(width * scale), round(height * scale)))
        canvas = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        y, x = (self.image_size - resized.shape[0]) // 2, (self.image_size - resized.shape[1]) // 2
        canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        return Observation(
            images={"wrist": canvas},
            state={
                "eef_state": np.r_[
                    pose[:3],
                    self.robot.commanded_width
                    if self.robot.commanded_width is not None
                    else state["gripper_width_m"],
                ],
                "tcp_quat": pose[3:],
                "gripper_width": np.array([state["gripper_width_m"]]),
            },
            instruction=self._instruction,
            image_times={"wrist": packet["host_monotonic_ns"] / 1e9},
            state_time=stamp,
            extra={
                "camera": packet,
                "read_only": self.read_only,
                "mock": self.mock,
                "transport_log_dir": str(self.run_dir),
            },
        )

    def step(self, action: Action) -> StepResult:
        """Execute one bounded waypoint, verify arrival, and observe after arrival."""
        if self.read_only:
            raise EmbodimentFault("read_only=true: motion is disabled")
        try:
            if self.robot is None or self._last is None:
                raise EmbodimentFault("reset must succeed before step")
            if time.monotonic() - self._started > self.max_duration_s:
                raise EmbodimentFault("FR3 wall-clock run limit reached")
            data = np.asarray(action.data, dtype=float)
            box = self.info.action_space
            if data.shape != (4,) or not np.isfinite(data).all():
                raise EmbodimentFault("expected finite [x,y,z,gripper_width]")
            if np.any(data < box.low) or np.any(data > box.high):
                raise EmbodimentFault("action outside FR3 command envelope")
            if np.any(np.abs(data[:3] - self._last_command[:3]) > self.step_m + 1e-7):
                raise EmbodimentFault("action exceeds per-axis FR3 step limit")
            # Refresh before any move: stale/dropout cameras must stop buffered chunks too.
            current = self.observe()
            if np.linalg.norm(current.state["eef_state"][:3] - self._last[:3]) > 0.01:
                raise EmbodimentFault("TCP moved more than 1 cm since the last observation")
            from scipy.spatial.transform import Rotation

            rotation_error = (
                Rotation.from_quat(current.state["tcp_quat"]).inv()
                * Rotation.from_quat(self._orientation)
            ).magnitude()
            if rotation_error > 0.1:
                raise EmbodimentFault("TCP orientation changed since reset")
            pose = np.r_[data[:3], self._orientation]
            self._record({"event": "started", "action": data.tolist(), "meta": dict(action.meta)})
            self.robot._execute(pose, float(data[3]))
            observation = self.observe(after_ns=time.monotonic_ns())
            self._last = observation.state["eef_state"].copy()
            self._last_command = data.copy()
            receipt = self.robot.last_receipt
            self._record({"event": "completed", "receipt": receipt})
            return StepResult(
                observation,
                info={
                    "execution_receipt": receipt,
                    "success_verified": False,
                    "transport_log_dir": str(self.run_dir),
                },
            )
        except BaseException:
            self.close()
            raise

    def _record(self, value):
        with (self.run_dir / "execution.jsonl").open("a") as stream:
            stream.write(json.dumps({"host_time": time.time(), **value}) + "\n")

    def close(self):
        """Stop only the owned RPC session, then release the camera process."""
        try:
            if self.robot is not None:
                self.robot.close()
        finally:
            self.robot = None
            self._last = None
            self._last_command = None
            if self.camera is not None:
                self.camera.close()
                self.camera = None
