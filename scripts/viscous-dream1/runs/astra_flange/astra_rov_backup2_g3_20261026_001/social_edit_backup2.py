"""Social cut of Astra two-handed ROV episode: 3x2 grid (four camera views + wide commentary panel)."""
import subprocess, sys
from PIL import Image, ImageDraw, ImageFont

EP = sys.argv[1] if len(sys.argv) > 1 else "astra_rov_backup2_g3_20261026_001"
FF = "/home/gavin/dreamtrial/.venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2"
ROOT = f"/home/gavin/dreamtrial/viscous_sim/runs/astra_flange/{EP}"
SRC, OUT, TMP = f"{ROOT}/run.mp4", f"{ROOT}/astra_rov_twohand_social.mp4", "/tmp/a6/social_backup"
D = "/usr/share/fonts/truetype/dejavu/"
F = lambda n, s: ImageFont.truetype(D + n, s)
BG, W, G, A, GRN = (15, 19, 24), (240, 244, 248), (160, 170, 182), (255, 196, 92), (120, 220, 140)
SPEED, END, HOLD = 6.0, 585.5, 3.0

BEATS = [
    (0, 57, A, "Backup wrench", "Reaches round with the LEFT hand to grip the bolt head,\nso the bolt can't spin while the nut turns."),
    (57, 70, A, "Lining up", "Bolt head held. Measuring the bolt end and the nut with depth."),
    (70, 157, GRN, "Thread caught", "Short starting turn, then the screw routine.\nLeft hand keeps holding the bolt."),
    (157, 321, GRN, "3 ... 7 turns", "The bolt hasn't moved. Fixes its grip between batches."),
    (321, 443, GRN, "7 ... 9 turns", "Gap to the flange: 10 mm ... 4 mm."),
    (443, 547, GRN, "Finishing", "Slow turns as the nut reaches the flange."),
    (547, 574, GRN, "Seated  \u2713", "Nut snug against the flange (0.1 mm). 10 turns."),
    (574, END + 1, GRN, "Seated  \u2713", "Lets go with both hands. The nut stays put.\nOne try, no drops, bolt held the whole time."),
]

def panel(i, col, head, body):
    im = Image.new("RGB", (1280, 360), BG); d = ImageDraw.Draw(im)
    d.text((40, 30), "GPT-6 Astra  ·  Fully autonomous  ·  two hands", font=F("DejaVuSans.ttf", 22), fill=G)
    d.text((40, 80), head, font=F("DejaVuSans-Bold.ttf", 56), fill=col)
    d.multiline_text((40, 170), body, font=F("DejaVuSans.ttf", 30), fill=W, spacing=10)
    d.line((40, 290, 1240, 290), fill=(50, 58, 68), width=2)
    d.text((40, 306), "Nut drops: 0     Bolt held by the left hand", font=F("DejaVuSans-Bold.ttf", 22), fill=G)
    d.text((880, 308), "Simulation | underwater | 6x", font=F("DejaVuSans.ttf", 20), fill=G)
    p = f"{TMP}/beat_{i:02d}.png"; im.save(p); return p

subprocess.run(["mkdir", "-p", TMP], check=True)
beats = [(s / SPEED, min(e, END + 1) / SPEED, panel(i, c, h, b)) for i, (s, e, c, h, b) in enumerate(BEATS)]
inputs = ["-i", SRC]
for *_, p in beats:
    inputs += ["-loop", "1", "-framerate", "30", "-i", p]
fc = [
    "[0]split=4[a][b][c][d];[a]crop=640:360:0:0[p0];[b]crop=640:360:640:0[p1];[c]crop=640:360:1280:0[p2];[d]crop=640:360:1920:0[p3]",
    "color=c=0x0f1318:s=1280x360:r=10[bl];[p0][p1][p2][p3][bl]xstack=inputs=5:layout=0_0|640_0|1280_0|0_360|640_360:shortest=1[g]",
    f"[g]setpts=(PTS-STARTPTS)/{SPEED},fps=30[m0]",
]
last = "m0"
for k, (t0, t1, _) in enumerate(beats):
    fc.append(f"[{last}][{k + 1}]overlay=640:360:shortest=0:eof_action=pass:enable='between(t,{t0:.3f},{t1:.3f})'[m{k + 1}]")
    last = f"m{k + 1}"
fc.append(f"[{last}]trim=0:{END / SPEED:.3f},setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={HOLD},format=yuv420p,setsar=1[v]")
subprocess.run([FF, "-v", "error", "-y", *inputs, "-filter_complex", ";".join(fc), "-map", "[v]", "-c:v", "libx264",
                "-crf", "18", "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], check=True)
print("out", OUT, "duration", round(END / SPEED + HOLD, 1))
