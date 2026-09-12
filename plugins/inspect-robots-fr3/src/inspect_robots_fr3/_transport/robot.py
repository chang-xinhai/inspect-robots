"""Pose7 transport interface over the robot-owned DepthUMI deploy queue."""

from __future__ import annotations

from concurrent.futures import Future
import queue
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation


def pose6_to_pose7(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (6,) or not np.isfinite(p).all():
        raise ValueError("expected finite xyz + rotation vector [6]")
    return np.r_[p[:3], Rotation.from_rotvec(p[3:]).as_quat()]


def pose7_to_pose6(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all() or np.linalg.norm(p[3:]) < 1e-8:
        raise ValueError("expected finite xyz + xyzw quaternion [7]")
    return np.r_[p[:3], Rotation.from_quat(p[3:]).as_rotvec()]


class RpcWorker:
    """All ZeroRPC calls share one OS thread/gevent hub, including keep-alive.

    Heartbeats run independently of vision, inference and action waits. A fault
    latches and prevents further motion; close stops only the session we own.
    """

    def __init__(self, host="127.0.0.1", port=4242, client_factory=None):
        self.address = f"tcp://{host}:{port}"
        self.factory = client_factory
        self.requests = queue.Queue()
        self.stop = threading.Event()
        self.ready = Future()
        self.token = None
        self.fault = None
        self.thread = threading.Thread(target=self._run, name="fr3-rpc", daemon=True)
        self.thread.start()
        self.ready.result(timeout=5)

    def _run(self):
        client = None
        try:
            if self.factory:
                client = self.factory()
            else:
                import zerorpc

                client = zerorpc.Client(timeout=2, heartbeat=20)
                client.connect(self.address)
            self.ready.set_result(True)
            heartbeat = 0.0
            while not self.stop.is_set():
                if self.token and time.monotonic() >= heartbeat:
                    client.keep_deploy_alive(self.token)
                    heartbeat = time.monotonic() + 0.1
                try:
                    name, args, future = self.requests.get(timeout=0.02)
                except queue.Empty:
                    continue
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    value = getattr(client, name)(*args)
                    if name == "start_deploy":
                        self.token = value["token"]
                    if name == "stop_deploy":
                        self.token = None
                    future.set_result(value)
                except Exception as exc:
                    future.set_exception(exc)
                    raise
        except BaseException as exc:
            self.fault = exc
            if not self.ready.done():
                self.ready.set_exception(exc)
        finally:
            if client is not None:
                if self.token:
                    try:
                        client.stop_deploy(self.token)
                    except Exception:
                        pass  # server lease/native watchdog still apply
                try:
                    client.close()
                except Exception:
                    pass
            self.stop.set()
            while not self.requests.empty():
                _, _, future = self.requests.get_nowait()
                if not future.done():
                    future.set_exception(RuntimeError(f"FR3 RPC stopped: {self.fault}"))

    def call(self, name, *args):
        if self.stop.is_set() or self.fault:
            raise RuntimeError(f"FR3 RPC stopped: {self.fault}")
        future = Future()
        self.requests.put((name, args, future))
        try:
            return future.result(timeout=10)
        except BaseException:
            future.cancel()
            self.stop.set()
            raise

    def close(self):
        self.stop.set()
        self.thread.join(timeout=6)


class Fr3Robot:
    """Poses exposed here are the configured TCP in policy/base coordinates.

    The server applies T_ee_from_tool, IK and velocity limits. Never apply the
    transform twice or call legacy impedance/gripper methods during this session.
    """

    def __init__(
        self,
        rpc,
        servo_config,
        *,
        period_s=0.3,
        action_timeout_s=15,
        open_width_m=0.08,
        close_width_m=0.0,
        read_only=False,
        position_tolerance_m=0.005,
        rotation_tolerance_rad=0.05,
    ):
        self.rpc, self.servo_config = rpc, servo_config
        self.period_s, self.action_timeout_s = period_s, action_timeout_s
        self.open_width_m, self.close_width_m = open_width_m, close_width_m
        self.token = None
        self.last_receipt = None
        self.commanded_width = None
        # Measured aperture cannot tell whether a close command was issued.
        self.gripper_command = None
        self.read_only = read_only
        self.position_tolerance_m = position_tolerance_m
        self.rotation_tolerance_rad = rotation_tolerance_rad
        self.action_receipts = []

    def preflight(self):
        if not self.rpc.call("get_deploy_capabilities").get("policy_action_queue"):
            raise RuntimeError("launch_ik server lacks policy_action_queue support")
        return self.rpc.call("get_robot_state_snapshot")

    def start(self):
        if self.read_only:
            return self.state()
        result = self.rpc.call("start_deploy", self.servo_config)
        self.token = result["token"]
        self.commanded_width = float(result["gripper_width_m"])
        return self.state()

    def state(self):
        if self.read_only:
            raw = self.rpc.call("get_robot_state_snapshot")
            ee = np.asarray(raw["O_T_EE"], dtype=float).reshape(4, 4, order="F")
            tcp = (
                np.asarray(self.servo_config["T_policy_from_base"])
                @ ee
                @ np.asarray(self.servo_config["T_ee_from_tool"])
            )
            pose = np.r_[tcp[:3, 3], Rotation.from_matrix(tcp[:3, :3]).as_rotvec()]
            return {
                **raw,
                "pose": pose.tolist(),
                "running": True,
                "fault": None,
                "gripper_width_m": self.rpc.call("get_gripper_width"),
                "read_only": True,
            }
        if self.token is None:
            raise RuntimeError("FR3 deploy session has not started")
        state = self.rpc.call("get_deploy_state", self.token)
        if not state["running"] or state.get("fault"):
            raise RuntimeError(state.get("fault") or "FR3 servo stopped")
        pose6_to_pose7(state["pose"])
        return state

    def get_ee_pose(self):
        return pose6_to_pose7(self.state()["pose"])

    def get_gripper_position(self):
        return np.array([self.state()["gripper_width_m"]])

    def _execute(self, pose, width):
        if self.read_only:
            raise RuntimeError("motion is disabled in read-only mode")
        state = self.state()
        deadline = int(state["server_now_ns"] + self.action_timeout_s * 1e9)
        queued = self.rpc.call(
            "exec_policy_actions",
            self.token,
            [pose7_to_pose6(pose).tolist()],
            [float(width)],
            self.period_s,
            deadline,
        )
        queue_id = queued["queue_id"]
        end = time.monotonic() + self.action_timeout_s
        while time.monotonic() < end:
            receipt = self.rpc.call("get_policy_action_status", self.token, queue_id)
            self.last_receipt = receipt
            if (
                receipt.get("queue_id") != queue_id
                or receipt.get("fault")
                or not receipt.get("running")
            ):
                raise RuntimeError(f"FR3 action queue failed: {receipt}")
            if receipt.get("completed"):
                if (
                    not receipt.get("completed_timestamp_ns")
                    or not receipt.get("steps")
                    or not all(s.get("completed") for s in receipt["steps"])
                ):
                    raise RuntimeError("FR3 action completion lacks execution acknowledgements")
                self.commanded_width = float(width)
                # The queue acknowledges streamed commands, not physical arrival.
                # Wait for measured convergence before capturing the next view.
                while time.monotonic() < end:
                    measured = self.state()
                    actual = pose6_to_pose7(measured["pose"])
                    position_error = float(np.linalg.norm(actual[:3] - np.asarray(pose)[:3]))
                    rotation_error = float(
                        (
                            Rotation.from_quat(actual[3:]).inv() * Rotation.from_quat(pose[3:])
                        ).magnitude()
                    )
                    if (
                        position_error <= self.position_tolerance_m
                        and rotation_error <= self.rotation_tolerance_rad
                        and not measured.get("gripper_busy", False)
                    ):
                        receipt = dict(
                            receipt,
                            measured_position_error_m=position_error,
                            measured_rotation_error_rad=rotation_error,
                        )
                        self.last_receipt = receipt
                        self.action_receipts.append(receipt)
                        return
                    time.sleep(0.02)
                self.close()
                raise TimeoutError("FR3 acknowledged command but measured TCP did not converge")
            time.sleep(0.02)
        self.close()
        raise TimeoutError("FR3 action did not complete before its deadline")

    def update_desired_ee_pose(self, pose):
        self._execute(pose, self.commanded_width)

    def control_gripper(self, close):
        self._execute(self.get_ee_pose(), self.close_width_m if close else self.open_width_m)
        self.gripper_command = bool(close)

    def close(self):
        # RpcWorker owns cleanup; no terminate_current_policy on foreign controllers.
        self.rpc.close()
        self.token = None
