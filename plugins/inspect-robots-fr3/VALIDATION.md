# Local migration validation

## Full TCP and visual demonstration extension

2026-09-13: default FR3 action is now seven-dimensional
`[x,y,z,rx,ry,rz,gripper_width]`, with a bounded trial-start rotation-vector
chart and `R_target=Exp(rotvec)@R_start` in policy/base axes. The existing
native move tool and delta limiter accept this explicitly declared chart;
other previously unsupported absolute rotation representations still fail.

- Core: 1725 passed, 6 skipped (optional Rerun SDK absent).
- Agent: 529 passed, including persistent historical-image attachment tests.
- FR3: 12 passed. Coverage includes nonidentity starting attitude, base-axis
  rotation composition, mixed rotations and return to the reference, translation
  after rotation, grip preservation, excessive rotation/drift rejection, measured
  rotational settling after queue acknowledgment, and recorded joint state.
- Ruff and targeted mypy checks passed.
- Actual Astra medium + native tools + mock RPC: one seven-dimensional rotation
  target generated, five waypoints executed in memory, measured mock ry=0.12.
  Evidence: `logs/validation/fr3-full-tcp/result.json` and transcript/CLI capture.
- Current DepthUMI `ServoConfig` / `CartesianTrajectory` tested offline with this
  machine's transforms: TCP→EE→TCP roundtrip verified; a 0.12-rad rotation has a
  0.06-rad midpoint while TCP position stays fixed. EE position changes as required
  by the tool offset. Evidence: `logs/validation/fr3-full-tcp/transport-math.json`.
- Historical 18-frame demo is attached separately from rolling live observations;
  source path/hash are logged. Prior notes now describe the available rotation
  interface instead of the obsolete fixed-attitude restriction.
- No physical robot connection or motion was used for these checks. Previously
  observed real joint-limit faults are not claimed fixed. Cartesian interpolation
  is not an IK/collision feasibility planner. Rotation requires clearance for the
  swept wrist/camera/fingers even at a stationary TCP target.

## Initial migration checks (before the full TCP extension)

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
