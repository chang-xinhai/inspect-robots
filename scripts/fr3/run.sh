#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
if [[ "${1:-}" == --check || "${1:-}" == --preview || "${1:-}" == --vision-check || "${1:-}" == --dry-run ]]; then
    exec .venv/bin/inspect-robots-fr3 "$@"
fi
exec .venv/bin/inspect-robots run --config configs/fr3.ini \
    --instruction "Pick up the cup and place it upright in the plate. Release it, lift the gripper clear, and verify that the cup rests in the plate." "$@"
