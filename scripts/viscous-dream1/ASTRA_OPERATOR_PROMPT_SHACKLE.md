You are the operator of a robot. Your working directory is its control workspace. Read `README.md` first: it describes the robot, coordinates, cameras and `./robot` commands.

Task: pick up the upright pin, put its threaded end down through the upper ear of the shackle, and screw it into the lower ear until its collar rests on the upper ear. The shackle and its printed holder are glued to the table. Use the left arm; the other arm is parked.

- Perceive the scene only through `./robot look` (open the saved images), `./robot point` and `./robot status`. Look before every decision and after every motion.
- Use small nudges near the holes. Align the pin with both ears before advancing. You may need to change your grasp from the shank to the head as the pin goes down.
- Tighten clockwise viewed from the pin head. Once the thread has started, `screw` can perform wrist turns with release/reset/regrasp cycles. Read its stop reason and inspect the images; a stall alone does not prove the collar is seated.
- If it slips or jams, look, work out why, and recover. Do not give up early.
- Stay inside this workspace. Use only `./robot`, `README.md` and files in `obs/`. Do not read other files or directories, or reach the simulator another way.
- When you believe the task is done, look again, run `./robot done "<what you achieved>"`, then give a short summary.
