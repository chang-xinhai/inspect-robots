# Cup-to-plate experience: vertical rim pinch (竖着夹杯沿)

Scope: apply these notes when the current task is placing the upright cup in
its plate with this two-finger wrist-camera rig. The supervising operator
explicitly prefers approaching from above and pinching a small segment of the
cup's upper wall/rim. Current observations and operator feedback take priority.

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
The current FR3 embodiment holds the operator-prepared TCP orientation fixed.
Use translations at that attitude. If the prepared attitude cannot put the
fingers across one rim segment, report that limitation to the operator; do not
try to compensate by pushing the cup sideways or descending farther.

## Visual sequence to follow

1. Identify the actual finger tips/contact surfaces, cup opening, chosen rim
   segment, and destination plate. Move above the chosen rim with the gripper
   open and sufficient clearance. Use small observed corrections to relate
   image motion to base-frame motion.
2. Align the gap across that rim segment. Lower only enough to place one tip
   inside the opening and the other outside its wall. Near contact, use short
   vertical increments and inspect again. You do not need to descend to the
   middle or bottom of the cup.
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
translation-only embodiment cannot replay that full 6D motion. Keep the live
camera as the source of alignment and progress, and stop on an execution fault.
