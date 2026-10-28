# Shackle station robot

You control the left Viscous arm. The right arm is parked and its commands are disabled. Each arm has six joints and a parallel gripper. The shackle stands upright, glued into a printed holder that is glued to the table. The loose pin starts standing on its threaded tip. Pick it up, pass its threaded end down through the clear upper ear, and screw it clockwise into the tapped lower ear until the collar rests on the upper ear.

World coordinates are metres; +Z is up. Tool coordinates refer to the pad midpoint, not the distal fingertips. The fingers extend beyond this point. Use the RGB cameras and depth picks to find the parts and choose clear approaches. Work slowly near the ears. The fixture does not move. Parts and arms have contact physics and finite joint travel; motion is not collision planned.

Use only these commands, README.md and obs/:

- `./robot look`: saves fresh top, wrist_l, wrist_r RGB images (960×540) and returns camera calibration.
- `./robot point top|wrist_l|wrist_r U V`: depth at a pixel, in world metres.
- `./robot status`: joint state, tool pose, opening and measured jaw loads.
- `./robot move left X Y Z [AX AY AZ]`: absolute tool position in metres, optional approach direction. It preserves roll while aligning approach.
- `./robot nudge left DX DY DZ`: world displacement in millimetres.
- `./robot grip left open|close|0..1`: 0 closes, 1 opens. Opening above 0.65 releases. Check actual opening and jaw loads.
- `./robot turn left DEGREES [--feed-mm N] [--rate-deg-s R]`: roll around current approach, with optional feed along approach. A downward approach makes positive roll clockwise from above. The right-hand pitch is 2.5 mm: use feed `degrees * 2.5 / 360` along a downward approach.
- `./robot screw left DEGREES [--rate-deg-s R]`: after aligning and starting the thread, executes measured roll/feed strokes and contact-sensed release/reset/regrasp cycles. Inspect its stop reason; it cannot certify seating. A downward approach is needed for axial wrist roll. Start with bilateral contact. Use small motions and look to check progress.
- `./robot last`: recover the last response without motion.
- `./robot done "one-line claim"`: finish.

Camera views use the station top mount and Maker-style wrist mounts. The right wrist remains at its parked pose. Calibration below includes the top crop; wrist extrinsics change with the hand and are updated by look. The inspection camera is not an operator input.

There are at most 1000 commands and 21600 wall seconds. Move/turn commands have a 90-second wall limit; screw has 300 seconds. Commands stop on sustained actuator saturation or stalled tracking. Inspect residuals and choose a reachable target after a stop. Do not interrupt the wrapper; it reports progress while working. Simulation pauses between commands. The six arm actuator limits are 25 Nm for this simulation; the real arm is rated 5 Nm continuous. Status reports sensed robot state, not object poses or task outcomes.
