In this directory, `previous_attempt.log` is the full transcript of your own previous episode as the operator of a
two-arm robot: the task prompt, the README you were given, every `./robot` command you ran with its output, and
your own progress notes. In it you threaded a nut onto a bolt at a pipe flange and ran it down snug, but only on
the sixth try: the thread failed to catch several times, the `screw` skill stalled, and you dropped the nut three
times (once under the flange). That scene was a tabletop.

Your next episode is the same task (bolt already through the flange, nut held in the right gripper, thread it on
and run it down snug) in a different scene: the two arms are mounted on an underwater ROV, so positions,
camera views and reach will differ. The goal is to succeed on the first try without dropping the nut.

Read the transcript carefully, then write `LESSONS.md` in this directory: concise, practical lessons for your next
attempt. Cover what finally worked (grasp, alignment, how you started the thread, how you used `screw` and read its
results), what caused each failure and drop and how to avoid it, how to check progress from the cameras and
sensors, and a step-by-step procedure. Do not include absolute coordinates or joint angles (the scene will be
different); relative quantities (millimetres of offset, degrees of tilt, jaw openings, loads) are fine. Keep it
under about 80 lines. Only read and write files in this directory.
