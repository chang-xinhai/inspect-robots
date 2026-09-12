#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
if [[ ! -x .venv/bin/python ]]; then
    "${PYTHON_BIN:-python3}" -m venv .venv
fi
.venv/bin/python -m pip install uv
.venv/bin/uv sync --locked --package inspect-robots-fr3 --package inspect-robots-agent --inexact
.venv/bin/python scripts/fr3/import_site.py "${1:-$HOME/projects/DepthUMI}"
.venv/bin/inspect-robots-fr3 --check
