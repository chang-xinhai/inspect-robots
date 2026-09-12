# Local migration validation

2026-09-13 (Asia/Shanghai), Codex CLI 0.154.0, Python 3.12.12.
Upstream: `7e4d1b7aee1c0d3cfc3a05a7492b9d12cda666f9`.
Runtime installation follows committed `uv.lock` for core + agent + FR3.

- Core: 1720 passed, 6 skipped (optional Rerun SDK absent).
- Agent: 526 passed, including 4 Codex wire tests.
- FR3: 8 passed, including all native waypoints in a move chunk, command-vs-measured grip, camera failure cleanup, read-only mode and invalid-action rejection.
- Ruff check and format use the upstream locked Ruff 0.15.20.
- Native CLI mock + real ChatGPT/Codex request: completed, `give_up` as instructed; no physical robot connection.
- Native eval JSON, frames, wire capture, transcript and HTML report generated for that mock run.
- Real Gemini RGB 1280x720: passed. Real RGB + Codex image interpretation and structured native tool-call envelope: passed (`--vision-check`).
- Real read-only FR3 check: RPC capabilities and robot snapshot returned, but gripper read failed in the existing DepthUMI server with `gripper SDK worker: RuntimeError: Net Exception`. Existing native comm log also reports network exceptions. No control session or motion was started.
- Physical movement, direction, gripper contact and cup/plate task success: NOT tested.

Tests are software/transport checks, not a physics simulation or measured task success rate.
