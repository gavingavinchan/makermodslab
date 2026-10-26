#!/usr/bin/env python3
# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Local-only social exports. Run with Python + Pillow and ffmpeg on PATH.

All paths derive from this run directory. No network, raw run, or hardware.
The original script is parsed as data (never imported: importing executes it).
Camera panes retain exactly 16:9. Rebuilt caption transitions use out_t(),
with the final caption held through the source's ending. All important text
is inside y=220..1540. Re-run with --assets-only to inspect before encoding.
"""
import argparse
import ast
import json
import math
import shutil
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

RUN = Path(__file__).resolve().parent.parent
OUT = RUN / "social"
ASSETS = OUT / "assets"
SOURCE = RUN / "astra_flange_social.mp4"
FF = shutil.which("ffmpeg") or "ffmpeg"
FP = shutil.which("ffprobe") or "ffprobe"
BG = (15, 19, 24)
WHITE = (240, 244, 248)
GRAY = (160, 170, 182)
AMBER = (255, 196, 92)
GREEN = (120, 220, 140)
FONT = "/System/Library/Fonts/SFNS.ttf"
FPS = 30


def call(args, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, **kwargs)


def probe(path):
    return json.loads(subprocess.check_output([
        FP, "-v", "error", "-show_streams", "-show_format", "-of", "json", str(path)
    ]))


def font(size, bold=False):
    f = ImageFont.truetype(FONT, size)
    f.set_variation_by_axes([100, 28, 400, 700 if bold else 400])
    return f


def text(draw, xy, value, size, color=WHITE, bold=False):
    # Top-left anchors make safe-area placement independent of font ascenders.
    draw.text(xy, value, font=font(size, bold), fill=color, anchor="lt")


def source_data():
    tree = ast.parse((RUN / "social_edit.py").read_text())
    env = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {n.id for n in ast.walk(node.targets[0]) if isinstance(n, ast.Name)}
            if names & {"BG", "SPLIT", "BEATS"}:
                exec(compile(ast.Module(body=[node], type_ignores=[]), "beat data", "exec"), env)
        elif isinstance(node, ast.FunctionDef) and node.name == "out_t":
            exec(compile(ast.Module(body=[node], type_ignores=[]), "time mapping", "exec"), env)
    return env


def base_image():
    im = Image.new("RGB", (1080, 1920), BG)
    d = ImageDraw.Draw(im)
    hook = "We gave an AI a robot and one job:"
    assert d.textlength(hook, font=font(47, True)) <= 896
    text(d, (92, 244), hook, 47, bold=True)
    text(d, (92, 306), "put a nut on a bolt.", 62, AMBER, True)
    im.save(ASSETS / "base.png")
    return im


def panel(beat, path):
    _, _, head, col, body, drops, fails = beat
    im = Image.new("RGB", (896, 378), BG)
    d = ImageDraw.Draw(im)
    text(d, (0, 8), "GPT-6 Astra  ·  Fully autonomous", 27, GRAY)
    text(d, (0, 52), head, 54, col, True)
    for i, line in enumerate(body.splitlines()):
        assert d.textlength(line, font=font(33)) < 896
        text(d, (0, 127 + i * 42), line, 33)
    d.line((0, 273, 896, 273), fill=(50, 58, 68), width=2)
    text(d, (0, 291), f"Nut drops: {drops}     Failed attempts: {fails}", 31, GRAY, True)
    text(d, (0, 339), "Simulation  |  sped up", 25, GRAY)
    im.save(path)


def extract(path, time, target):
    call([FF, "-v", "error", "-y", "-ss", time, "-i", path,
          "-frames:v", 1, "-update", 1, target])


def covers():
    # Last half-second: source visibly shows the seated nut and released jaws.
    extract(SOURCE, 115.5, ASSETS / "seated_source.png")
    extract(OUT / "astra_flange_reel_9x16.mp4", 115.5, ASSETS / "seated_reel.png")
    Image.open(ASSETS / "seated_reel.png").convert("RGB").save(
        OUT / "cover_reel_1080x1920.jpg", quality=95, subsampling=0)
    src = Image.open(ASSETS / "seated_source.png").convert("RGB")
    # Full 16:9 wrist image, scaled uniformly. A translucent top scrim preserves
    # the flange/nut in the lower half while providing a high-contrast hook.
    im = src.crop((0, 360, 640, 720)).resize((1920, 1080), Image.Resampling.LANCZOS).convert("RGBA")
    scrim = Image.new("RGBA", im.size)
    sd = ImageDraw.Draw(scrim)
    for y in range(500):
        alpha = 245 if y < 300 else round(245 * (500 - y) / 200)
        sd.line((0, y, 1919, y), fill=(*BG, alpha))
    im = Image.alpha_composite(im, scrim).convert("RGB")
    d = ImageDraw.Draw(im)
    text(d, (80, 60), "We gave an AI a robot and one job:", 77, bold=True)
    text(d, (80, 158), "put a nut on a bolt.", 104, AMBER, True)
    # A small opaque footer keeps the result legible over the pale flange.
    d.rounded_rectangle((64, 946, 1856, 1040), radius=14, fill=BG)
    text(d, (90, 971), "6 attempts. 3 dropped nuts. Seated ✓", 50, GREEN, True)
    text(d, (1460, 983), "IN SIMULATION", 29, GRAY, True)
    im.save(OUT / "cover_linkedin_1920x1080.jpg", quality=95, subsampling=0)


def verify(source_probe):
    output_probe = probe(OUT / "astra_flange_reel_9x16.mp4")
    v, a = output_probe["streams"]
    assert (v["width"], v["height"], v["codec_name"], v["profile"], v["pix_fmt"]) == (
        1080, 1920, "h264", "High", "yuv420p")
    assert v["avg_frame_rate"] == "30/1"
    assert (a["codec_name"], a["sample_rate"], a["channels"]) == ("aac", "48000", 2)
    duration = float(source_probe["format"]["duration"])
    assert abs(float(output_probe["format"]["duration"]) - duration) < .002
    assert abs(float(v["duration"]) - duration) <= 1 / FPS
    # Confirm progressive-download atom order, rather than trusting flags alone.
    atoms = []
    with (OUT / "astra_flange_reel_9x16.mp4").open("rb") as f:
        while header := f.read(8):
            size, kind = int.from_bytes(header[:4], "big"), header[4:].decode("ascii")
            if size == 1:
                size = int.from_bytes(f.read(8), "big")
                header_size = 16
            else:
                header_size = 8
            atoms.append(kind)
            if size == 0:
                break
            f.seek(size - header_size, 1)
    assert atoms.index("moov") < atoms.index("mdat")
    report = {"source": source_probe, "reel": output_probe, "top_level_atoms": atoms,
              "safe_text_y": [244, 1522], "pane_sizes": [[896, 504], [432, 243]],
              "notes": "Captions re-rendered from BEATS/out_t; nearest CFR frame count; silent audio and container retain source duration."}
    (OUT / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    times = [2, 20, 45, 70, 95, 112, 115.5]
    sheet = Image.new("RGB", (4 * 432, 2 * 814), (27, 32, 39))
    d = ImageDraw.Draw(sheet)
    for i, t in enumerate(times):
        path = ASSETS / f"check_{t:g}.jpg"
        extract(OUT / "astra_flange_reel_9x16.mp4", t, path)
        frame = Image.open(path).resize((432, 768), Image.Resampling.LANCZOS)
        x, y = (i % 4) * 432, (i // 4) * 814
        sheet.paste(frame, (x, y + 46))
        text(d, (x + 12, y + 10), f"{t:g} s", 25)
        # Guides on QA sheet only; they do not appear in deliverables.
        d.line((x, y + 46 + 88, x + 431, y + 46 + 88), fill=(220, 75, 80), width=2)
        d.line((x, y + 46 + 616, x + 431, y + 46 + 616), fill=(220, 75, 80), width=2)
    sheet.save(OUT / "reel_contact_sheet.jpg", quality=95, subsampling=0)
    print(json.dumps({"source_s": duration, "reel_s": output_probe["format"]["duration"],
                      "video_s": v["duration"], "frames": v["nb_frames"],
                      "files": {p.name: p.stat().st_size for p in OUT.glob("*") if p.is_file()}}, indent=2))


def main():
    args = argparse.ArgumentParser()
    args.add_argument("--assets-only", action="store_true")
    args = args.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    data = source_data()
    info = probe(SOURCE)
    duration = float(info["format"]["duration"])
    frames = round(duration * FPS)
    base = base_image()
    concat = ["ffconcat version 1.0"]
    beat_frames = [math.ceil(round(data["out_t"](b[0]), 3) * FPS - 1e-8) for b in data["BEATS"]]
    for i, b in enumerate(data["BEATS"]):
        path = ASSETS / f"panel_{i:02d}.png"
        panel(b, path)
        stop = beat_frames[i + 1] if i + 1 < len(beat_frames) else frames
        concat += [f"file '{path.name}'", "option framerate 30", f"duration {(stop - beat_frames[i]) / FPS:.9f}"]
    concat += [f"file 'panel_{len(data['BEATS']) - 1:02d}.png'", "option framerate 30"]
    (ASSETS / "panels.ffconcat").write_text("\n".join(concat) + "\n")
    # Asset preview includes the longest caption and both status symbols.
    src_frame = ASSETS / "preview_source.png"
    extract(SOURCE, 45, src_frame)
    src = Image.open(src_frame)
    base.paste(src.crop((0, 360, 640, 720)).resize((896, 504), Image.Resampling.LANCZOS), (92, 370))
    base.paste(src.crop((0, 0, 640, 360)).resize((432, 243), Image.Resampling.LANCZOS), (92, 890))
    base.paste(src.crop((640, 0, 1280, 360)).resize((432, 243), Image.Resampling.LANCZOS), (556, 890))
    base.paste(Image.open(ASSETS / "panel_07.png"), (92, 1152))
    base.save(ASSETS / "layout_preview.jpg", quality=95)
    glyphs = Image.new("RGB", (800, 160), BG)
    text(ImageDraw.Draw(glyphs), (24, 36), "Attempt 4 ✗    Seated ✓", 55, GREEN, True)
    glyphs.save(ASSETS / "glyph_check.png")
    if args.assets_only:
        return
    fc = (
        "[0:v]fps=30,split=3[w][t][i];"
        "[w]crop=640:360:0:360,scale=896:504:flags=lanczos,setsar=1[wr];"
        "[t]crop=640:360:0:0,scale=432:243:flags=lanczos,format=yuv444p,setsar=1[top];"
        "[i]crop=640:360:640:0,scale=432:243:flags=lanczos,format=yuv444p,setsar=1[ins];"
        "[1:v]format=yuv444p[bg];"
        "[bg][wr]overlay=92:370:format=yuv444:eof_action=repeat[b1];"
        "[b1][top]overlay=92:890:format=yuv444:eof_action=repeat[b2];"
        "[b2][ins]overlay=556:890:format=yuv444:eof_action=repeat[b3];"
        "[2:v]fps=30,format=yuv444p[panel];"
        f"[b3][panel]overlay=92:1152:format=yuv444:eof_action=repeat,trim=end_frame={frames},"
        "setpts=PTS-STARTPTS,format=yuv420p,setsar=1[v];"
        f"[3:a]atrim=duration={duration},asetpts=PTS-STARTPTS[a]"
    )
    cmd = [FF, "-hide_banner", "-y", "-filter_complex_threads", "2",
           "-i", SOURCE, "-loop", "1", "-framerate", FPS, "-i", ASSETS / "base.png",
           "-f", "concat", "-safe", "0", "-i", ASSETS / "panels.ffconcat",
           "-f", "lavfi", "-i", "anullsrc=r=48000:cl=stereo",
           "-filter_complex", fc, "-map", "[v]", "-map", "[a]",
           "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-profile:v", "high",
           "-pix_fmt", "yuv420p", "-r", FPS, "-fps_mode", "cfr", "-threads", "4",
           "-c:a", "aac", "-ar", "48000", "-ac", "2", "-b:a", "128k",
           "-movflags", "+faststart", "-map_metadata", "-1",
           OUT / "astra_flange_reel_9x16.mp4"]
    (OUT / "build_command.json").write_text(json.dumps([str(c) for c in cmd], indent=2) + "\n")
    with (OUT / "build.log").open("w") as log:
        call(cmd, stdout=log, stderr=subprocess.STDOUT)
    covers()
    verify(info)


if __name__ == "__main__":
    main()
