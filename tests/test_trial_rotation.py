"""Explicit bounded rotation charts preserve existing absolute-pose refusals."""

from dataclasses import replace

import numpy as np
import pytest

from inspect_robots.approver import DeltaLimitApprover
from inspect_robots.compat import check_compatibility
from inspect_robots.mock import CubePickEmbodiment, ScriptedPolicy
from inspect_robots.policy import PolicyInfo
from inspect_robots.spaces import ActionSemantics, Box
from inspect_robots.types import Action


def _space(radius: float = 1.5) -> Box:
    return Box(
        (7,),
        low=np.array([-1, -1, -1, -radius, -radius, -radius, 0]),
        high=np.array([1, 1, 1, radius, radius, radius, 0.08]),
        semantics=ActionSemantics(
            "eef_abs_pose",
            rotation_repr="axis_angle",
            rotation_reference="trial_start",
            dim_labels=("x", "y", "z", "rx", "ry", "rz", "gripper_width"),
            max_step=(0.01, 0.01, 0.01, 0.02, 0.02, 0.02, 0.08),
        ),
    )


def test_chart_limiter_and_absolute_reference_compatibility() -> None:
    space = _space()
    assert space.semantics is not None
    limiter = DeltaLimitApprover(space)
    store: dict[str, object] = {}
    limiter.review(Action(np.zeros(7)), store)
    result = limiter.review(Action(np.array([0, 0, 0, 0.1, -0.1, 0.1, 0])), store)
    np.testing.assert_allclose(result.data[3:6], [0.02, -0.02, 0.02])
    assert result.meta["delta_clamped"]
    absolute = replace(space, semantics=replace(space.semantics, rotation_reference="frame"))
    with pytest.raises(ValueError, match="rotation_repr"):
        DeltaLimitApprover(absolute)
    policy, body = ScriptedPolicy(), CubePickEmbodiment()
    policy.info = PolicyInfo(name="absolute", action_space=absolute)
    body.info = replace(body.info, action_space=space)
    assert any(
        issue.code == "rotation_reference" for issue in check_compatibility(policy, body).errors
    )


@pytest.mark.parametrize("radius", [2.0, float("inf"), float("nan")])
def test_chart_rejects_wraparound_and_nonfinite_bounds(radius: float) -> None:
    with pytest.raises(ValueError, match="inside pi"):
        _space(radius)


def test_chart_requires_origin_and_named_components() -> None:
    space = _space()
    assert space.low is not None and space.semantics is not None
    low = space.low.copy()
    low[3] = 0.1
    with pytest.raises(ValueError, match="include zero"):
        replace(space, low=low)
    with pytest.raises(ValueError, match="rx, ry, rz"):
        replace(space, semantics=replace(space.semantics, dim_labels=None))
    with pytest.raises(ValueError, match="requires eef_abs_pose"):
        ActionSemantics("eef_abs_pose", rotation_repr="quat_xyzw", rotation_reference="trial_start")
