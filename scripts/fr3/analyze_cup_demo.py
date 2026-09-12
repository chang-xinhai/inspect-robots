"""Offline episode-0 trajectory evidence; never imports or connects to hardware.

Requires h5py, numpy, scipy and Pillow (available in the DepthUMI environment).
Phase boundaries are manual visual annotations for this specific cup episode.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

PHASES = (
    (0.0, 2.2, "approach_and_reorient"),
    (2.2, 2.4, "final_rim_approach"),
    (2.4, 3.0, "close_on_rim"),
    (3.0, 4.0, "lift_and_transport"),
    (4.0, 4.8, "position_over_plate"),
    (4.8, 5.4, "release"),
    (5.4, 6.0, "initial_withdrawal"),
    (6.0, 8.9, "withdraw_and_inspect"),
)


def main() -> None:
    """Export measured phase statistics and a timestamped demo storyboard."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    with h5py.File(args.source, "r") as source:
        dataset = source["state/ee_poses"]
        if dataset.attrs.get("order") != "xyz+qwqxqyqz":
            raise ValueError("Expected explicitly labelled xyz+qwqxqyqz poses")
        poses = dataset[:].astype(float)
        timestamps = source["state/timestamp_ns"][:]
        times = (timestamps - timestamps[0]) / 1e9
        widths = source["state/ee_joint_states"][:, 0]
        if (
            poses.shape != (len(times), 7)
            or len(widths) != len(times)
            or not np.isfinite(poses).all()
            or not np.isfinite(widths).all()
            or not np.all(np.diff(times) > 0)
            or abs(times[-1] - 8.9) > 0.05
        ):
            raise ValueError("Expected the reviewed 8.9-second aligned cup episode")
        rotations = Rotation.from_quat(poses[:, [4, 5, 6, 3]])
        # Independently cross-check the pose convention against stored actions.
        delta = source["action/delta_ee_poses"]
        if delta.attrs.get("order") != "xyz+qwqxqyqz":
            raise ValueError("Unexpected delta quaternion convention")
        recorded_delta = delta[:-1]
        calculated_translation = rotations[:-1].inv().apply(np.diff(poses[:, :3], axis=0))
        translation_error = float(np.max(np.abs(calculated_translation - recorded_delta[:, :3])))
        recorded_rotation = Rotation.from_quat(recorded_delta[:, [4, 5, 6, 3]])
        calculated_rotation = rotations[:-1].inv() * rotations[1:]
        rotation_error = float(np.max((recorded_rotation.inv() * calculated_rotation).magnitude()))
        if translation_error > 1e-5 or rotation_error > 1e-5:
            raise ValueError("State poses disagree with recorded local delta actions")

        phases = []
        for start, end, name in PHASES:
            a, b = (int(np.argmin(abs(times - time))) for time in (start, end))
            displacement = poses[b, :3] - poses[a, :3]
            phases.append(
                {
                    "phase": name,
                    "start_index": a,
                    "end_index": b,
                    "start_s": float(times[a]),
                    "end_s": float(times[b]),
                    "net_displacement_m": float(np.linalg.norm(displacement)),
                    "sampled_path_length_m": float(
                        np.linalg.norm(np.diff(poses[a : b + 1, :3], axis=0), axis=1).sum()
                    ),
                    "displacement_in_phase_start_tcp_m": rotations[a]
                    .inv()
                    .apply(displacement)
                    .tolist(),
                    "endpoint_orientation_change_deg": float(
                        np.rad2deg((rotations[a].inv() * rotations[b]).magnitude())
                    ),
                    "measured_width_start_end_m": [float(widths[a]), float(widths[b])],
                }
            )

        selection = sorted(
            set(
                [*range(0, len(times), 10), len(times) - 1]
                + [
                    int(np.argmin(abs(times - time)))
                    for time in (2.2, 2.4, 2.6, 2.8, 4.8, 5.2, 5.4, 5.6)
                ]
            )
        )
        args.out.mkdir(parents=True, exist_ok=True)
        samples = []
        sheet = Image.new("RGB", (4 * 384, ((len(selection) + 3) // 4) * 250), "white")
        draw = ImageDraw.Draw(sheet)
        for tile, index in enumerate(selection):
            frame_name = f"frame_{index:03d}.png"
            encoded = np.asarray(source["vision/cam_wrist/colors"][index], dtype=np.uint8).tobytes()
            with Image.open(io.BytesIO(encoded)) as decoded:
                frame = decoded.convert("RGB")
            frame.save(args.out / frame_name)
            frame.thumbnail((384, 216))
            x, y = (tile % 4) * 384, (tile // 4) * 250
            sheet.paste(frame, (x, y))
            draw.text(
                (x + 5, y + 220),
                f"t={times[index]:.1f}s  measured width={widths[index] * 1000:.1f}mm",
                fill="black",
            )
            samples.append(
                {
                    "index": index,
                    "time_s": float(times[index]),
                    "image": frame_name,
                    "measured_width_m": float(widths[index]),
                }
            )
        sheet.save(args.out / "storyboard.jpg")
        arrays_hash = hashlib.sha256()
        for array in (poses, timestamps, widths):
            arrays_hash.update(array.tobytes())
        evidence = {
            "source": str(args.source.resolve()),
            "numeric_arrays_sha256": arrays_hash.hexdigest(),
            "pose_order": "xyz+qwqxqyqz",
            "coordinate_frame": (
                "Vive source world; phase deltas use each phase's initial TCP axes, not FR3 base"
            ),
            "calibration_status": str(source.attrs.get("calibration_status")),
            "hardware_deployment_validated": bool(
                source.attrs.get("hardware_deployment_validated", False)
            ),
            "phase_boundary_source": (
                "manual visual review of this episode; not automatic contact detection"
            ),
            "sample_count": len(times),
            "duration_s": float(times[-1]),
            "max_orientation_change_from_start_deg": float(
                np.rad2deg((rotations[0].inv() * rotations).magnitude()).max()
            ),
            "delta_translation_crosscheck_max_error_m": translation_error,
            "delta_rotation_crosscheck_max_error_rad": rotation_error,
            "phases": phases,
            "selected_frames": samples,
            "motion_commanded": False,
        }
    (args.out / "trajectory.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({"output": str(args.out), "phases": phases}, indent=2))


if __name__ == "__main__":
    main()
