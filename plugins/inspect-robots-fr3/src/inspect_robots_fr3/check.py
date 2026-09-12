"""No-motion FR3 setup, preview, and native agent checks."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PIL import Image

from inspect_robots.defaults import load_defaults
from inspect_robots.registry import resolve
from inspect_robots.scene import Scene
from inspect_robots.types import Observation

from ._transport.camera import GeminiCamera, MockCamera
from .embodiment import FR3Embodiment


def main():
    """Run local, camera-only, or read-only robot plus model checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--preview", action="store_true")
    modes.add_argument("--vision-check", action="store_true")
    modes.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--config", default="configs/fr3.ini")
    parser.add_argument("--model")
    args = parser.parse_args()
    defaults = load_defaults({"INSPECT_ROBOTS_CONFIG": str(Path(args.config).resolve())})
    embodiment_args = defaults.embodiment_args if defaults.embodiment_args_owner == "fr3" else {}
    rig = FR3Embodiment(**{**embodiment_args, "read_only": True, "mock": args.mock})
    policy_args = defaults.policy_args if defaults.policy_args_owner == "agent" else {}
    if args.model:
        policy_args = {**policy_args, "model": args.model}
    output = Path("logs/preflight")
    output.mkdir(parents=True, exist_ok=True)
    if args.check:
        policy = resolve("policy", "agent", **policy_args)
        policy.bind(rig.info)
        print(
            json.dumps(
                {
                    "ok": True,
                    "robot": rig.info.name,
                    "policy": asdict(policy.config),
                    "motion": False,
                },
                indent=2,
            )
        )
        return
    if args.preview or args.vision_check:
        camera = (
            MockCamera()
            if args.mock
            else GeminiCamera(sys.executable, output / "camera.log", **rig.site.get("camera", {}))
        )
        try:
            rgb, metadata = camera.snapshot()
            Image.fromarray(rgb).save(output / "preview.png")
            (output / "camera.json").write_text(json.dumps(metadata, indent=2))
            print(f"RGB OK {rgb.shape}; {(output / 'preview.png').resolve()}")
        finally:
            camera.close()
        if args.vision_check:
            from inspect_robots_agent.policy import _observation_content

            policy = resolve("policy", "agent", **policy_args)
            policy.bind(rig.info)
            policy.on_trial_start("vision-only", 0, str(output.resolve()), "vision")
            policy.reset(
                Scene(
                    id="vision-only",
                    instruction=(
                        "Describe this real camera frame, then call give_up. This is a "
                        "camera-only preflight. No robot state is available; request no motion."
                    ),
                )
            )
            try:
                message = policy._client.complete(
                    [
                        *policy._messages,
                        {
                            "role": "user",
                            "content": _observation_content(Observation(images={"wrist": rgb})),
                        },
                    ],
                    policy._toolset.schemas(),
                    reasoning_effort=policy._effort,
                )
                (output / "vision-decision.json").write_text(json.dumps(message.raw(), indent=2))
                print(json.dumps(message.raw(), indent=2))
                print("RGB + model wire OK; no robot connection, no tool executed.")
            finally:
                if policy._capture is not None:
                    policy._capture.end_trial()
        return
    scene = Scene(
        id="fr3-preflight",
        instruction=(
            "No-motion preflight. Describe what the wrist camera shows and the measured robot "
            "state. Then call give_up with reason that this is a read-only check; "
            "do not request motion."
        ),
    )
    policy = resolve("policy", "agent", **policy_args)
    policy.bind(rig.info)
    policy.on_trial_start(scene.id, 0, str(output.resolve()), "preflight")
    policy.reset(scene)
    try:
        try:
            observation = rig.reset(scene)
        except Exception as exc:
            (output / "hardware-error.txt").write_text(str(exc))
            raise SystemExit(
                "FR3 read-only check failed; no control session was started. "
                "Check the existing launch_comm / launch_ik terminals and robot/gripper "
                "connection. Details: "
                + str((output / "hardware-error.txt").resolve())
                + "\n"
                + str(exc).splitlines()[-1]
            ) from None
        Image.fromarray(observation.images["wrist"]).save(output / "dry-run.png")
        (output / "robot.json").write_text(
            json.dumps(
                {key: np.asarray(value).tolist() for key, value in observation.state.items()},
                indent=2,
            )
        )
        chunk = policy.act(observation)
        (output / "decision.json").write_text(
            json.dumps(
                {
                    "meta": dict(chunk.meta),
                    "actions": [a.data.tolist() for a in chunk.actions],
                    "motion_executed": False,
                },
                indent=2,
            )
        )
        print(f"Native agent decision OK; no step called. Results: {output.resolve()}")
    finally:
        rig.close()
        (output / "transcript.json").write_text(json.dumps(policy.transcript(), indent=2))
        if policy._capture is not None:
            policy._capture.end_trial()


if __name__ == "__main__":
    main()
