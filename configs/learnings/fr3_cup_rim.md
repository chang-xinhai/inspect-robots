# Cup-rim demonstration: approach while adjusting attitude

Use the attached 18-frame HISTORICAL storyboard as a visual example. It is not
the current scene. Pinch one segment of the upper cup wall, carry the upright
cup into its plate, release, and withdraw.

## A coordinated approach, not three isolated stages

The operator's “up and forward, look down, then grasp from above” describes
the overall path. It does NOT prescribe a separate vertical takeoff followed
by stationary pitching until the wrist is perfectly vertical. “竖着夹杯沿”
specifies a grasp at the upper rim with a suitable approach; it does not
require a 90-degree wrist attitude or a perfectly downward-facing camera.

The demo moves toward the cup while the viewing/grasp attitude changes.
Only use as much clearance as the actual finger/wrist geometry requires.
When the observed geometry supports it, coordinate modest XYZ translation
and attitude adjustment in the same move_to, then observe the result.
Do not postpone all approach translation until an imagined final tilt angle
has been reached. A short isolated tilt can establish direction, but repeated
stationary tilts are not the demonstrated approach.

Reinspection of the early demo at 0.5-second intervals shows:

| Source interval | Evidence | Lesson for the live approach |
| --- | --- | --- |
| 0.0–0.5 s | Small initial motion; the scene remains oblique. Source TCP net motion is about 2.1 cm and attitude change about 4 degrees. | A modest start, not a large clearance ascent. These values are historical, not commanded increments. |
| 0.5–1.0 s | Approach continues with reorientation; at 1.0 s the attitude differs from its start by about 11 degrees. | Forward approach is already under way while the view is still oblique. |
| 1.0–2.2 s | Continued translation AND progressive reorientation. The cup opening becomes substantially larger and stays near the right finger. Attitude change is about 25 degrees at 1.5 s and 42 degrees at 2.2 s. | Establish a close rim-grasp configuration through coordinated approach. Do not just rotate the distant tabletop into a nicer view. |
| 2.2–2.4 s | Right image finger inside the cup opening, left outside the same left-side wall; only a small local adjustment remains. | Stop gross positioning once the fingers can straddle this wall. The cup center correctly remains offset to the right. |

Source angles describe progression, not a target to copy or a universal angle
cap. Source-world coordinates are not FR3 base coordinates. “Forward” in this
description does not determine a live base-axis sign.

## Judge progress at the rim, not by how vertical the view looks

Useful progress means the chosen rim becomes reachable between the actual
contact surfaces, with one finger inside and one outside the SAME thin wall.
Use the changing rim/finger relationship, apparent scale, and measured TCP
motion together. Image size or pixel alignment alone does not establish depth.
Seeing more tabletop, or making the cup opening look rounder, is insufficient.
If pitching makes the cup move toward the image edge without bringing its rim
into the working region, reassess translation and tilt together instead of
continuing to pitch in place. Do not chase a “perfectly vertical” attitude.

Do not require final grasp/contact evidence before every coarse approach
step. If the selected rim is still distant and the observed path has clearance,
continue a bounded approach and reobserve; not yet straddling the wall is an
expected intermediate state. Partial cup-opening occlusion by the right finger
also occurs in the successful demo, so a completely visible opening is not a
prerequisite. Occlusion alone proves neither collision nor a valid grasp.
When occlusion prevents judging the next increment, actively seek a useful view
with a small supported lateral/attitude adjustment, using the last clear view
and measured motion as context. Do not repeat only x/z moves indefinitely when
the missing information is hidden behind one finger. If no supported view
adjustment is available, report the missing clearance/distance information;
do not guess contact depth or close blindly.

The failed run adhoc_bc31eeb3 illustrates this mistake: after a small up/forward
move, the agent repeatedly held XYZ almost fixed and requested ry=0.18, 0.38,
0.60, 0.85, then 1.10 rad. The cup stayed small and moved toward the image's
upper edge. The final measured attitude change was about 60 degrees; no grasp
approach occurred and execution eventually timed out. Do not repeat this
stationary-tilt ladder. An earlier failure similarly kept retreating at fixed
height to align image rows. Both optimized the image instead of approaching
the selected rim in space.

## Grasp, carry, release

- At 2.4–3.0 s, the fingers close locally on the upper wall. Aperture changes
  from about 68 to 8 mm while TCP endpoint displacement is about 1.7 cm.
  Do not span the whole cup diameter, descend along the full body, or try to
  center the cup before closing.
- At 3.0–4.8 s, the cup stays with the closed fingers while the plate comes
  underneath. Verify attachment with a short lift before a longer carry;
  the demo does not establish a separate stationary verification pause.
- At 4.8–5.4 s, placement and opening are local motions near the plate.
  Establish live support, then open. Images do not directly measure force.
- After 5.4 s the open fingers withdraw; the cup remains upright in the plate.
  Large withdrawal belongs AFTER release, not before grasping.

## Interface and evidence

move_to controls absolute XYZ in metres and one trial-relative rotation vector
rx,ry,rz in radians: R_target=Exp([rx,ry,rz])@R_start. Vector axes are fixed in
the policy/base frame; this is not Euler angles or per-step rotation. Zero
rotation restores the starting attitude, not a top-down attitude. Omitted
dimensions keep their measured values. Rotation holds the TCP target but
sweeps the fingers, wrist and camera; account for clearance throughout.
gripper_width=0 closes and 0.08 opens. Keep the closing command while carrying;
nonzero measured aperture alone does not prove attachment.

Evidence: episode_000000/vive_tracker.hdf5 from the
2026_09_08-03_00_00-pick_cup-8runs batch; 90 aligned 10 Hz states over 8.9 s.
The 1 Hz overview is supplemented by contact/release frames at 5 Hz and an
offline 0.5-second early-approach review. Phase descriptions combine images,
recorded TCP/aperture measurements and operator guidance. No metric camera
extrinsics or source-to-live object registration are available. Use supported
live corrections; stop on an execution fault or if the view cannot support
the next move. Historical distances, widths, rotations and times are not an
executable trajectory or a task-success guarantee.
