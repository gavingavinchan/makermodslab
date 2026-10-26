"""Social cut of Astra flange episode 6: commentary panel per beat, speed ramp, title and end cards."""
import subprocess
from PIL import Image, ImageDraw, ImageFont

FF = "/home/gavin/dreamtrial/.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2"
SRC = "/home/gavin/dreamtrial/viscous_sim/runs/astra_flange/astra_g3_20261025_006/run.mp4"
OUT = "/home/gavin/dreamtrial/viscous_sim/runs/astra_flange/astra_g3_20261025_006/astra_flange_social.mp4"
TMP = "/tmp/a6/social"
D = "/usr/share/fonts/truetype/dejavu/"
F = lambda n, s: ImageFont.truetype(D + n, s)
BG, W, G, A, RED, GRN = (15, 19, 24), (240, 244, 248), (160, 170, 182), (255, 196, 92), (255, 110, 100), (120, 220, 140)

SPLIT, FAST, SLOW, END = 1138.0, 16.0, 6.0, 1406.6
def out_t(s):
    return s / FAST if s < SPLIT else SPLIT / FAST + (s - SPLIT) / SLOW

# (start_s, end_s, headline, colour, body, drops, fails)
BEATS = [
    (0, 185, "Attempt 1", A, "Lining the nut up with the bolt,\nusing only the robot's cameras.", 0, 0),
    (185, 255, "Attempt 1  ✗", RED, "Turned it. Nothing caught.\nDropped the nut on the table.", 1, 1),
    (255, 360, "Recovery", A, "Picks it back up. Hands it to the\nleft hand and back for a better grip.", 1, 1),
    (360, 479, "Attempt 2  ✗", RED, "Calls the screw skill.\nIt stalls after 0.6 degrees.", 1, 2),
    (479, 533, "Attempt 3  ✗", RED, "0.1 turns...\nthen drops the nut. Again.", 2, 3),
    (533, 656, "Recovery", A, "Picks it up. Another hand-off.\nRe-measures with depth.", 2, 3),
    (656, 710, "Attempt 4", A, "0.2 turns. Personal best.", 2, 3),
    (710, 875, "Attempt 4  ✗", RED, "Drops it UNDER the flange.\nFinds it with the other arm's camera\nand pokes it out with the left hand.", 3, 4),
    (875, 1010, "Recovery", A, "Got it back. Another hand-off.\nRe-aims at the bolt.", 3, 4),
    (1010, 1105, "Attempt 5  ✗", RED, "Screw skill stalls. Again.", 3, 5),
    (1105, 1160, "Attempt 6", A, "Tries a different wrist angle.", 3, 5),
    (1160, 1215, "Attempt 6", GRN, "It's turning.\n3 full turns.", 3, 5),
    (1215, 1274, "Attempt 6", GRN, "5 turns...", 3, 5),
    (1274, 1401, "Attempt 6", GRN, "Running it down to the flange...", 3, 5),
    (1401, END + 1, "Seated  ✓", GRN, "Nut snug against the flange.", 3, 5),
]

def panel(i, head, col, body, drops, fails):
    im = Image.new("RGB", (640, 360), BG); d = ImageDraw.Draw(im)
    d.text((32, 26), "GPT-6 Astra  \u00b7  Fully autonomous", font=F("DejaVuSans.ttf", 18), fill=G)
    d.text((32, 66), head, font=F("DejaVuSans-Bold.ttf", 40), fill=col)
    d.multiline_text((32, 130), body, font=F("DejaVuSans.ttf", 24), fill=W, spacing=8)
    d.line((32, 290, 608, 290), fill=(50, 58, 68), width=2)
    d.text((32, 304), f"Nut drops: {drops}     Failed attempts: {fails}", font=F("DejaVuSans-Bold.ttf", 20), fill=G)
    d.text((32, 332), "Simulation  |  sped up", font=F("DejaVuSans.ttf", 16), fill=G)
    p = f"{TMP}/beat_{i:02d}.png"; im.save(p); return p

def card(lines, path):
    im = Image.new("RGB", (1280, 720), BG); d = ImageDraw.Draw(im)
    y = sum(g for *_, g in lines); y = (720 - y) // 2
    for text, f, col, gap in lines:
        w = d.textlength(text, font=f); d.text(((1280 - w) / 2, y), text, font=f, fill=col); y += gap
    im.save(path); return path

subprocess.run(["mkdir", "-p", TMP], check=True)
beats = [(out_t(s), out_t(min(e, END + 1)), panel(i, h, c, b, dr, fa)) for i, (s, e, h, c, b, dr, fa) in enumerate(BEATS)]
title = card([
    ("We gave an AI a robot and one job:", F("DejaVuSans-Bold.ttf", 44), W, 64),
    ("put a nut on a bolt.", F("DejaVuSans-Bold.ttf", 44), A, 90),
    ("GPT-6 Astra sees only through the robot's cameras. Zero human input.", F("DejaVuSans.ttf", 24), G, 40),
], f"{TMP}/title.png")
end = card([
    ("6 attempts. 3 dropped nuts. 743 commands.", F("DejaVuSans-Bold.ttf", 40), W, 70),
    ("Nut: seated  ✓", F("DejaVuSans-Bold.ttf", 52), GRN, 100),
    ("Fully autonomous.*", F("DejaVuSans-Bold.ttf", 36), A, 70),
    ("*in simulation, with the motor torque limit turned way up", F("DejaVuSans.ttf", 22), G, 34),
], f"{TMP}/end.png")

TITLE_S, HOLD_S, END_S = 0, 2.5, 0
inputs = ["-i", SRC]
for _, _, p in beats:
    inputs += ["-loop", "1", "-framerate", "30", "-i", p]
fc = [
    "[0]split=3[a][b][c];[a]crop=640:360:0:0[p0];[b]crop=640:360:640:0[p1];[c]crop=640:360:1280:0[p2]",
    "color=c=0x0f1318:s=640x360:r=10[bl];[p0][p1][p2][bl]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0:shortest=1,split[g1][g2]",
    f"[g1]trim=0:{SPLIT},setpts=(PTS-STARTPTS)/{FAST}[s1];[g2]trim={SPLIT},setpts=(PTS-STARTPTS)/{SLOW}[s2]",
    "[s1][s2]concat=n=2:v=1,fps=30[m0]",
]
last = "m0"
for k, (t0, t1, _) in enumerate(beats):
    fc.append(f"[{last}][{k + 1}]overlay=640:360:shortest=0:eof_action=pass:enable='between(t,{t0:.3f},{t1:.3f})'[m{k + 1}]")
    last = f"m{k + 1}"
dur = out_t(END)
fc.append(f"[{last}]trim=0:{dur:.3f},setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={HOLD_S},format=yuv420p,setsar=1[v]")
cmd = [FF, "-v", "error", "-y", *inputs, "-filter_complex", ";".join(fc), "-map", "[v]",
       "-c:v", "libx264", "-crf", "18", "-preset", "slow", "-profile:v", "high", "-pix_fmt", "yuv420p",
       "-movflags", "+faststart", OUT]
subprocess.run(cmd, check=True)
print("body", round(dur, 1), "total", round(TITLE_S + dur + HOLD_S + END_S, 1))
