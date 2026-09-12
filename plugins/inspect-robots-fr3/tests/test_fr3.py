"""Transport safety and native action integration tests; never use hardware."""

import time

import numpy as np
import pytest
from inspect_robots_agent._tools import build_toolset
from inspect_robots_fr3 import FR3Embodiment

from inspect_robots.errors import EmbodimentFault
from inspect_robots.scene import Scene
from inspect_robots.types import Action


def rig(tmp_path, **kwargs):
    return FR3Embodiment(
        site_config=str(tmp_path / "missing"), mock=True, log_dir=tmp_path, **kwargs
    )


def test_native_tool_waypoints_and_measured_observations(tmp_path):
    body = rig(tmp_path)
    try:
        obs = body.reset(Scene(id="test", instruction="move"))
        toolset = build_toolset(
            body.info.action_space, body.info.observation_space, body.info.control_hz
        )
        assert any(s["function"]["name"] == "move_to" for s in toolset.schemas())
        initial = obs.state["eef_state"].copy()
        target = initial.copy()
        target[0] += 0.009
        result = body.step(Action(target))
        assert result.info["execution_receipt"]["completed"]
        np.testing.assert_allclose(result.observation.state["eef_state"], target)
        assert result.observation.image_times["wrist"] >= obs.image_times["wrist"]
        np.testing.assert_allclose(result.observation.state["tcp_quat"], obs.state["tcp_quat"])
        rpc = body.robot.rpc
        time.sleep(0.15)
        assert rpc.token == "mock"
    finally:
        body.close()
    assert rpc.stop.is_set()


def test_read_only_never_starts_or_moves(tmp_path):
    body = rig(tmp_path, read_only=True)
    try:
        obs = body.reset(Scene(id="test", instruction="observe"))
        assert body.robot.token is None
        assert body.robot.rpc.token is None
        with pytest.raises(EmbodimentFault, match="disabled"):
            body.step(Action(obs.state["eef_state"]))
    finally:
        body.close()


@pytest.mark.parametrize(
    "data", [[float("nan"), 0, 0.3, 0, 0, 0, 0.08], [0.8, 0, 0.3, 0, 0, 0, 0.08], [0, 0]]
)
def test_invalid_step_stops_owned_session(tmp_path, data):
    body = rig(tmp_path)
    body.reset(Scene(id="test", instruction="move"))
    rpc = body.robot.rpc
    with pytest.raises(EmbodimentFault):
        body.step(Action(np.array(data)))
    assert rpc.stop.is_set()
    assert body.camera is None


def test_camera_failure_prevents_move_and_closes(tmp_path):
    body = rig(tmp_path)
    obs = body.reset(Scene(id="test", instruction="move"))
    rpc = body.robot.rpc

    def fail(**kwargs):
        raise RuntimeError("camera unplugged")

    body.camera.snapshot = fail
    with pytest.raises(RuntimeError, match="unplugged"):
        body.step(Action(obs.state["eef_state"]))
    assert rpc.stop.is_set()
    assert body.camera is None


def test_holding_grasp_does_not_command_measured_aperture(tmp_path):
    body = rig(tmp_path)
    try:
        obs = body.reset(Scene(id="test", instruction="grasp"))
        target = obs.state["eef_state"].copy()
        target[6] = 0
        body.step(Action(target))
        # Physical contact leaves a nonzero aperture although the close target is zero.
        original = body.robot.state
        body.robot.state = lambda: {**original(), "gripper_width_m": 0.035}
        obs = body.observe()
        assert obs.state["gripper_width"][0] == 0.035
        assert obs.state["eef_state"][6] == 0
    finally:
        body.close()


def test_full_pose_native_tools_use_base_axes_and_hold_rotation(tmp_path):
    import json

    from inspect_robots_agent._llm import ToolCall
    from scipy.spatial.transform import Rotation

    from inspect_robots.approver import DeltaLimitApprover

    body = rig(tmp_path)
    try:
        obs = body.reset(Scene(id="rotate", instruction="6D motion"))
        initial = Rotation.from_quat(obs.state["tcp_quat"])
        tools = build_toolset(
            body.info.action_space, body.info.observation_space, body.info.control_hz
        )
        assert tools.state_labels()[0] == "eef_state"  # Not same-shaped joint_pos/joint_vel.
        DeltaLimitApprover(body.info.action_space)  # Native guardrails accept explicit chart.
        for vector in ([0, 0.2, 0], [0.15, 0.3, -0.12], [0, 0, 0]):
            result = tools.execute(
                ToolCall(
                    "r",
                    "move_to",
                    json.dumps(
                        {
                            "targets": dict(zip(("rx", "ry", "rz"), vector, strict=True)),
                            "note": "bounded rotation test",
                        }
                    ),
                ),
                obs,
            )
            assert result.error is None
            for action in result.chunk.actions:
                previous = Rotation.from_quat(obs.state["tcp_quat"])
                obs = body.step(action).observation
                measured = Rotation.from_quat(obs.state["tcp_quat"])
                assert (measured * previous.inv()).magnitude() <= body.rotation_step_rad + 1e-7
            expected = Rotation.from_rotvec(vector) * initial
            np.testing.assert_allclose(measured.as_matrix(), expected.as_matrix(), atol=1e-10)
            np.testing.assert_allclose(obs.state["eef_state"][3:6], vector, atol=1e-10)
            # Translation + closing must retain the measured orientation, not reset it.
            result = tools.execute(
                ToolCall(
                    "t",
                    "move_to",
                    json.dumps(
                        {
                            "targets": {
                                "x": float(obs.state["eef_state"][0]) + 0.005,
                                "gripper_width": 0,
                            },
                            "note": "translate while retaining attitude and closing command",
                        }
                    ),
                ),
                obs,
            )
            assert result.error is None
            for action in result.chunk.actions:
                obs = body.step(action).observation
            np.testing.assert_allclose(
                Rotation.from_quat(obs.state["tcp_quat"]).as_matrix(),
                expected.as_matrix(),
                atol=1e-10,
            )
            assert obs.state["eef_state"][6] == 0
        events = [
            json.loads(line) for line in (body.run_dir / "execution.jsonl").read_text().splitlines()
        ]
        assert any(e["event"] == "rotation_reference" for e in events)
        assert all("q" in e and "dq" in e for e in events if e["event"] == "observation")
    finally:
        body.close()


def test_large_rotation_and_unexpected_orientation_drift_stop_before_command(tmp_path):
    from scipy.spatial.transform import Rotation

    for drift in (False, True):
        body = rig(tmp_path)
        obs = body.reset(Scene(id="reject", instruction="stop"))
        rpc = body.robot.rpc
        target = obs.state["eef_state"].copy()
        if drift:
            original = body.robot.state
            state = original()
            attitude = Rotation.from_rotvec([0, 0.2, 0]) * Rotation.from_rotvec(state["pose"][3:])
            body.robot.state = lambda state=state, attitude=attitude: {
                **state,
                "pose": [*state["pose"][:3], *attitude.as_rotvec()],
            }
        else:
            target[4] = 0.3
        with pytest.raises(EmbodimentFault, match=r"rotation step|orientation drifted"):
            body.step(Action(target))
        assert rpc.stop.is_set()
        assert body.camera is None


def test_reset_reanchors_current_attitude_and_rotations_start_at_zero(tmp_path):
    body = rig(tmp_path)
    try:
        for _ in range(2):
            obs = body.reset(Scene(id="reset", instruction="hold"))
            np.testing.assert_allclose(obs.state["eef_state"][3:6], 0, atol=1e-12)
            target = obs.state["eef_state"].copy()
            target[3] = 0.02
            body.step(Action(target))
    finally:
        body.close()


def test_queue_ack_does_not_bypass_measured_rotation_settling(tmp_path):
    from scipy.spatial.transform import Rotation

    body = rig(tmp_path)
    try:
        obs = body.reset(Scene(id="settle", instruction="wait for orientation"))
        original = body.robot.state
        old_pose = original()["pose"]
        reads = []

        def lagged_state():
            state = original()
            reads.append(state)
            return {**state, "pose": old_pose} if len(reads) < 4 else state

        body.robot.state = lagged_state
        target = Rotation.from_rotvec([0, 0.02, 0]) * Rotation.from_quat(obs.state["tcp_quat"])
        body.robot._execute(np.r_[obs.state["eef_state"][:3], target.as_quat()], 0.08)
        assert len(reads) >= 4
        assert body.robot.last_receipt["measured_rotation_error_rad"] < 1e-10
    finally:
        body.close()


def test_native_motion_chunk_executes_all_waypoints(tmp_path):
    import json

    from inspect_robots_agent._llm import ToolCall

    body = rig(tmp_path)
    try:
        obs = body.reset(Scene(id="test", instruction="move"))
        tools = build_toolset(
            body.info.action_space, body.info.observation_space, body.info.control_hz
        )
        target = float(obs.state["eef_state"][0]) + 0.025
        result = tools.execute(
            ToolCall(
                "move",
                "move_to",
                json.dumps({"targets": {"x": target}, "note": "small translation test"}),
            ),
            obs,
        )
        assert result.error is None
        assert len(result.chunk.actions) == 3
        for action in result.chunk.actions:
            step = body.step(action)
        assert step.observation.state["eef_state"][0] == pytest.approx(target)
        assert len(body.robot.action_receipts) == 3
    finally:
        body.close()
