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


@pytest.mark.parametrize("data", [[float("nan"), 0, 0.3, 0.08], [0.8, 0, 0.3, 0.08], [0, 0]])
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
        target[3] = 0
        body.step(Action(target))
        # Physical contact leaves a nonzero aperture although the close target is zero.
        original = body.robot.state
        body.robot.state = lambda: {**original(), "gripper_width_m": 0.035}
        obs = body.observe()
        assert obs.state["gripper_width"][0] == 0.035
        assert obs.state["eef_state"][3] == 0
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
