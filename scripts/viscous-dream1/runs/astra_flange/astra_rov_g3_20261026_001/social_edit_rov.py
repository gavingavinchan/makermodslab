"""Social cut of Astra flange episode 6: commentary panel per beat, speed ramp, title and end cards."""
import subprocess
from PIL import Image, ImageDraw, ImageFont

FF = "/home/gavin/dreamtrial/.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2"
SRC = "/home/gavin/dreamtrial/viscous_sim/runs/astra_flange/astra_rov_g3_20261026_001/run.mp4"
OUT = "/home/gavin/dreamtrial/viscous_sim/runs/astra_flange/astra_rov_g3_20261026_001/astra_rov_social.mp4"
TMP = "/tmp/a6/social"
D = "/usr/share/fonts/truetype/dejavu/"
F = lambda n, s: ImageFont.truetype(D + n, s)
BG, W, G, A, RED, GRN = (15, 19, 24), (240, 244, 248), (160, 170, 182), (255, 196, 92), (255, 110, 100), (120, 220, 140)

SPLIT, FAST, SLOW, END = 100000.0, 4.0, 4.0, 412.2
def out_t(s):
    return s / FAST if s < SPLIT else SPLIT / FAST + (s - SPLIT) / SLOW

# (start_s, end_s, headline, colour, body, drops, fails)
BEATS = [
    (0, 24, "Attempt 1", A, "Measuring the nut and the bolt\nwith the wrist camera's depth.", 0, 0),
    (24, 95, "Attempt 1", A, "Fingers bump the flange hardware.\nBacks off, finds a clearer wrist angle.", 0, 0),
    (95, 125, "Attempt 1", A, "Lines the nut up on the bolt axis again.", 0, 0),
    (125, 163, "Attempt 1", A, "Starts the thread with a short\n20 degree turn.", 0, 0),
    (163, 261, "Attempt 1", GRN, "It caught. 1 turn...\n4 turns...", 0, 0),
    (261, 401, "Attempt 1", GRN, "Running it down to the flange...", 0, 0),
    (401, END + 1, "Seated  \u2713", GRN, "Nut snug against the flange.\nFirst try.", 0, 0),
]

def panel(i, head, col, body, drops, fails):
    im = Image.new("RGB", (640, 360), BG); d = ImageDraw.Draw(im)
    d.text((32, 26), "GPT-6 Astra  \u00b7  Fully autonomous", font=F("DejaVuSans.ttf", 18), fill=G)
    d.text((32, 66), head, font=F("DejaVuSans-Bold.ttf", 40), fill=col)
    d.multiline_text((32, 130), body, font=F("DejaVuSans.ttf", 24), fill=W, spacing=8)
    d.line((32, 290, 608, 290), fill=(50, 58, 68), width=2)
    d.text((32, 304), f"Nut drops: {drops}     Failed attempts: {fails}", font=F("DejaVuSans-Bold.ttf", 20), fill=G)
    d.text((32, 332), "Simulation  |  underwater  |  4x speed", font=F("DejaVuSans.ttf", 16), fill=G)
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
    "color=c=0x0f1318:s=640x360:r=10[bl];[p0][p1][p2][bl]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0:shortest=1[g1]",
    f"[g1]setpts=(PTS-STARTPTS)/{FAST},fps=30[m0]",
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
