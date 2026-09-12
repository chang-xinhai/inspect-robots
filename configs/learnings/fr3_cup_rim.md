# Cup-to-plate experience: vertical rim pinch (竖着夹杯沿)

Scope: apply these notes when the current task is placing the upright cup in
its plate with this two-finger wrist-camera rig. The supervising operator
explicitly prefers approaching from above and pinching a small segment of the
cup's upper wall/rim. Current observations and operator feedback take priority.

The operator clarified the demonstrated approach: first move up and forward,
then reorient to look down at the cup, then approach the rim from above.
Do not replace that sequence with an immediate descent at the initial oblique
attitude or horizontal motion close to the table. The attached historical
18-frame storyboard is visual evidence of the sequence; inspect its EARLY
frames as well as the grasp frames. The source world's axes are not the
robot base axes, so the operator's qualitative directions are not copied XYZ
commands. The current tool DOES support reorientation through move_to targets
rx,ry,rz. Establish clearance, then use small observed rotations to prepare a
top-down rim approach. Do not give_up solely because the initial view is
oblique or because older attempts had no rotation tool. If the current view
or an execution fault prevents a supported move, explain that actual obstacle.

## Grasp geometry

Choose a reachable, unobstructed segment of the cup rim. Position the two
finger contact surfaces on opposite sides of that SAME thin wall segment:
one finger inside the cup opening, the other outside the cup. The local rim
segment belongs in the closing gap. The whole cup opening may remain offset
from the gripper center; that is compatible with this grasp. Do not treat
centering the whole cup between the fingers as the goal, span its full outer
diameter, descend along the full cup body, or seek the handle by default.

"Vertical" describes approaching the upper rim from above while the cup stays
upright. It does not mean rotating the image or changing the closing axis.
The current FR3 embodiment exposes translations AND rotation. rx,ry,rz are
one rotation vector in radians about the fixed policy/base axes, relative to
the trial's initial attitude: R_target=Exp([rx,ry,rz])@R_start. They are not
Euler roll/pitch/yaw or per-step increments. The vector norm is the angle;
its direction is the axis. Zeros restore the starting attitude, not top-down.
Use tcp_quat (absolute xyzw) and observed image changes to reason about tilt.
Prepare the grasp attitude before descending close to the rim; the camera,
wrist and long fingers sweep space even when the TCP position is held fixed.
Once grasped, preserve an upright cup while transporting it.

## Episode 0: what the demonstrated trajectory actually does

This is a historical demonstration, not the current observation. Times identify
its frames; they are not deadlines for the robot. Phase boundaries below were
annotated by visual inspection, not measured contact sensors. TCP motion and
apertures come from all 90 aligned 10 Hz samples, not just the 1 Hz overview.

1. **0.0-2.2 s: approach while changing the viewing/grasp attitude.** Initially
   the scene is oblique: the table and background are visible, the cup is on
   the right and the plate on the left. By 2.0-2.2 s the table fills the view,
   the cup opening is much larger, and the right image finger overlaps the
   opening near its left wall. The cup is still offset to the right; that
   offset is expected for the demonstrated rim grasp. The measured aperture
   stays about 75-76 mm. This is a substantial 3D approach with reorientation,
   not repeated fixed-height retreat to match image rows. Endpoint TCP
   displacement is 46.9 cm and orientation change is 42.1 degrees in the
   source recording. These describe the demonstrator's particular starting
   distance and attitude, NOT a distance or rotation to command on FR3.
2. **2.2-2.4 s: finish placing the open fingers across one rim segment.**
   The right image finger is inside the cup opening and the left is outside
   the same left-side wall segment. This identifies which local wall will be
   pinched; the cup center is not the grasp center. The recorded TCP endpoint
   displacement over this interval is only 1.9 cm, with 2.7 degrees of attitude
   change. The aperture begins decreasing, reaching 68.4 mm at 2.4 s.
3. **2.4-3.0 s: close with little additional TCP travel.** Measured apertures
   at 2.4, 2.6, 2.8 and 3.0 s are 68.4, 17.3, 11.3 and 7.9 mm. TCP endpoint
   displacement is 1.7 cm (sampled path length 2.9 cm) and attitude changes
   2.4 degrees. The fingers converge around that thin upper wall; the cup
   stays to the right of the nearly closed fingers. The useful lesson is to
   stop gross positioning once the rim is straddled, then close locally.
   Do not continue a long descent or try to center the entire cup first.
4. **3.0-4.8 s: carry the held cup toward the plate.** The cup maintains its
   relationship to the closed fingers while the plate moves into position
   behind/under it in the wrist image. Aperture remains roughly 6-8 mm.
   There is substantial transport: 32.5 cm endpoint displacement and
   23.4 degrees attitude change from 3.0 to 4.0 s, then 8.9 cm and 7.2 degrees
   from 4.0 to 4.8 s. Transfer the stage ordering and attachment check, not
   these layout-specific distances. A short verification lift before a long
   carry is an execution adaptation; the sampled demo does not establish a
   separate stationary verification pause.
5. **4.8-5.4 s: finish placement and open near the destination.** The plate
   is around/behind the cup, and the fingers spread apart while the cup
   remains there. Aperture grows from 6.8 mm at 4.8 s to 12.5 at 5.0,
   50.6 at 5.2 and 78.4 mm at 5.4 s. TCP endpoint displacement is only
   2.4 cm during this interval. This contrasts with the larger preceding
   carry: placement/release is a local action. The images support the
   placement interpretation but do not directly measure supporting force;
   establish live support before release.
6. **5.4-8.9 s: withdraw with the fingers open and inspect the result.** The
   cup remains upright within the plate as the wrist view pulls back to show
   the table again. Withdrawal is after release, not the earlier approach
   behavior. At 6.0 s the opening is about 75.9 mm; at the final frame it is
   74.9 mm. Use the cup remaining in the plate after the fingers leave as the
   visual completion condition.

The full demo reaches a 55.2-degree orientation difference from its first
pose. In particular, the large attitude change already occurs BEFORE closure.
Older FR3 runs fixed the starting attitude and omitted this observed part of
the approach. That restriction has now been removed. Include orientation
preparation in the plan and use the available rotation targets; a similar
initial-looking image alone does not determine the required rotation sign or
angle. The demo's 42.1/55.2-degree figures are evidence of a meaningful change,
not angles to copy blindly in a particular live base axis.

Sampling lesson: 1 Hz is useful for stage order, but would jump from the open
hand at 2.0 s to nearly closed at 3.0 s, hiding how it straddles and closes.
Use the 5 Hz contact/release descriptions above for those transitions. Neither
sampling frequency is a requested robot-control or LLM-call frequency.

## Apply the demonstration to the live task

1. Identify the actual finger tips/contact surfaces, cup opening, chosen rim
   segment, and destination plate. Move above the chosen rim with the gripper
   open and sufficient clearance. Use small observed corrections to relate
   image motion to base-frame motion.
2. Align the gap across that rim segment. Lower only enough to place one tip
   inside the opening and the other outside its wall. Near contact, use short
   vertical increments and inspect again. You do not need to descend to the
   middle or bottom of the cup. Alignment here means a feasible approach to
   straddle the wall in 3D, not making the distant rim and nearby finger tips
   share a pixel row before any descent.
3. Close after the two fingers visibly straddle the wall without pushing it.
   In this interface gripper_width=0 requests closing; 0.08 requests opening.
   Retain the closing command while carrying. Contact may leave a nonzero
   measured aperture; a width value alone does not prove a successful grasp.
4. Make a short upward verification lift and inspect whether the cup travels
   with the fingers while staying upright. If it stays on the table or slips,
   stop transport and reassess rather than assuming it was picked up.
5. Once attachment and table clearance are visible, carry the cup above the
   plate. Lower until the cup base is supported in the plate, open the fingers,
   lift clear, and inspect the final placement. Do not release above the plate
   merely because its outline appears behind the cup.

## Avoid the observed fixed-height retreat loop

In the recorded attempt adhoc_a3d685d9, the agent repeatedly reduced base x
at almost unchanged height because the cup rim moved upward in the wrist
image. It never explicitly commanded a lower z. The measured x moved from
0.4472 to 0.3655 m after its forward probe, without reaching a grasp; the
native controller subsequently reported joint_position_limits_violation.
These are observations from a failed attempt, not waypoints to repeat.

The wrist camera moves with the fingers. A rim below the tips in the image
does not by itself imply that the robot should retreat until their image
rows coincide. Image-row improvement is not evidence of decreasing distance
to the rim. Account for height, depth and perspective, and explain what each
probe establishes about an actual approach. After two alignment probes that
still do not establish a supported approach, reassess and ask for operator
guidance via give_up rather than repeating another same-direction probe.
This is a planning heuristic, not a hardware safety boundary.

When clearance and the prepared attitude support approaching the upper rim,
include a deliberate, small downward approach and inspect the new view;
do not postpone descent indefinitely to optimize a distant pixel alignment.
Do not turn this into an unconditional +x/-z rule: the current interface has
no calibrated camera-to-base transform or metric rim height. If the view
cannot support the approach, give_up with the missing information. A fixed
starting attitude is not necessarily a suitable rim-grasp attitude, and
translations cannot correct an unsuitable wrist orientation.

## Evidence and limits

These notes combine the operator's instruction with offline visual review of
DepthUMI overfit episode_000000/vive_tracker.hdf5, pick_cup, from the
2026_09_08-03_00_00-pick_cup-8runs processing batch. The aligned sample span is
8.9 s. Images at approximately 2.4–3.0 s show the rim pinch; 3–5 s show carrying
toward the plate; 5.0–5.4 s show opening; later frames show the cup in the plate
and the open gripper withdrawing. This supports a grasp strategy, not an
exact timing schedule or a verified success guarantee for the current robot.

The demonstration's measured opening changes from about 75 mm before grasp to
6–8 mm while holding, then back to about 78 mm on release. Those are historical
sensor readings, not target widths or the cup wall's measured thickness.
Do not copy those widths, source world coordinates, timestamps, or rotations
as commands. The demonstration includes orientation changes; the present
6D embodiment can adjust attitude, but still needs live geometric feedback
rather than replaying the source world coordinates. Keep the live camera as
the source of alignment and progress, and stop on an execution fault.
