"""Copy hardware settings once; never import or modify DepthUMI runtime code."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import yaml


def main():
    """Copy local calibration without changing the source deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("depthumi", type=Path)
    parser.add_argument(
        "--refresh", action="store_true", help="replace the local copy after calibration changes"
    )
    args = parser.parse_args()
    root = args.depthumi.expanduser().resolve()
    target = Path(__file__).resolve().parents[2] / "configs/site/fr3.yaml"
    if target.exists() and not args.refresh:
        print(f"Keeping existing local site config: {target}")
        return
    servo_path = root / "data/deploy/fr3.yaml"
    camera_path = root / "configs/record/real/v11_1.yaml"
    servo = yaml.safe_load(servo_path.read_text())
    source = yaml.safe_load(camera_path.read_text())["sources"]["gemini"]["options"]
    document = {
        "robot": {"host": "127.0.0.1", "port": 4242},
        "camera": {
            "serial": source["serial"],
            "width": source.get("color_width", 1280),
            "height": source.get("color_height", 720),
            "fps": 15,
            "maximum_age_s": 0.5,
        },
        "servo": servo,
        "site_provenance": {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (servo_path, camera_path)
        },
    }
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        "# Machine-local rig settings copied from DepthUMI; gitignored.\n"
        "# Copying does not validate estimated transforms or physical calibration.\n"
        + yaml.safe_dump(document, sort_keys=False)
    )
    target.chmod(0o600)
    print(f"Copied rig settings to {target}; DepthUMI files unchanged.")


if __name__ == "__main__":
    main()
