"""In-memory RPC-contract fixture. No dynamics and no task-success claim."""

from __future__ import annotations

import time


class MockRpcServer:
    def __init__(self):
        self.pose = [0.45, 0.0, 0.3, 3.141592653589793, 0.0, 0.0]
        self.width = 0.08
        self.active = False
        self.queue_id = 0
        self.receipt = {}
        self.heartbeats = 0

    def get_deploy_capabilities(self):
        return {"policy_action_queue": True}

    def get_robot_state_snapshot(self):
        import numpy as np
        from scipy.spatial.transform import Rotation

        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_rotvec(self.pose[3:]).as_matrix()
        matrix[:3, 3] = self.pose[:3]
        return {
            "q": [0.0] * 7,
            "dq": [0.0] * 7,
            "control_active": self.active,
            "mock": True,
            "O_T_EE": matrix.ravel(order="F").tolist(),
        }

    def get_gripper_width(self):
        return self.width

    def start_deploy(self, config):
        if self.active:
            raise RuntimeError("mock session already active")
        self.active = True
        return self.get_deploy_state("mock")

    def get_deploy_state(self, token):
        stamp = time.monotonic_ns()
        return {
            "token": "mock",
            "running": self.active,
            "fault": None,
            "pose": list(self.pose),
            "q": [0.0] * 7,
            "dq": [0.0] * 7,
            "gripper_width_m": self.width,
            "gripper_busy": False,
            "server_now_ns": stamp,
            "timestamp_ns": stamp,
            "gripper_timestamp_ns": stamp,
        }

    def keep_deploy_alive(self, token):
        self.heartbeats += 1

    def exec_policy_actions(self, token, poses, widths, period_s, deadline_ns):
        self.queue_id += 1
        self.pose, self.width = list(poses[-1]), float(widths[-1])
        stamp = time.monotonic_ns()
        self.receipt = {
            "queue_id": self.queue_id,
            "running": self.active,
            "fault": None,
            "completed": True,
            "completed_timestamp_ns": stamp,
            "steps": [
                {
                    "completed": True,
                    "completed_timestamp_ns": stamp,
                    "target_pose_6d": self.pose,
                    "gripper_width_m": self.width,
                }
            ],
        }
        return self.receipt

    def get_policy_action_status(self, token, queue_id):
        return self.receipt

    def stop_deploy(self, token):
        self.active = False
        return {"fault": None, "policy_actions": self.receipt}

    def close(self):
        pass
