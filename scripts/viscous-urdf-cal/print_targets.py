# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generate vector PDFs and 254 dpi (10 px/mm) monochrome target prints."""

import subprocess
from pathlib import Path

from PIL import Image
from reportlab import rl_config
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas
from targets import GRID, PITCH, target_xy

OUT = Path(__file__).resolve().parent


def draw(size, width, height):
    rl_config.useA85 = False
    cols, rows = GRID[size]
    path = OUT / f"targets-{size}"
    canvas = Canvas(str(path.with_suffix(".pdf")), pagesize=(width * mm, height * mm), invariant=True)
    canvas.setAuthor("Copyright 2026 MakerMods. All rights reserved. Apache-2.0")
    canvas.setTitle(f"Viscous touch targets / {size}")
    canvas.scale(mm, mm)
    x0, y0 = (width - (cols - 1) * PITCH) / 2, 28
    canvas.setFont("Helvetica-Bold", 6)
    canvas.drawString(x0, height - 16, "Viscous   /   Touch calibration")
    canvas.setFont("Helvetica", 3.4)
    canvas.drawString(x0, height - 24, f"{size}   /   30 mm grid   /   Print at actual size")
    for n in range(1, cols * rows + 1):
        x, y = target_xy(size, n)
        x, y = x0 + x, y0 + y
        canvas.setLineWidth(0.24)
        canvas.circle(x, y, 2.2)
        canvas.line(x - 3.4, y, x + 3.4, y)
        canvas.line(x, y - 3.4, x, y + 3.4)
        canvas.setFont("Helvetica-Bold", 3.4)
        canvas.drawString(x + 4, y + 3, str(n))
    for dx, dy, label in [(1, 0, "+X"), (0, 1, "+Y")]:
        x, y = x0 + dx * 16, y0 + dy * 16
        canvas.line(x0 + 4 * dx, y0 + 4 * dy, x, y)
        for s in [-1, 1]:
            canvas.line(x, y, x - 2 * dx + s * dy, y - 2 * dy + s * dx)
        canvas.drawString(x + 1, y + 1, label)
    canvas.setFont("Helvetica", 3)
    canvas.drawString(x0, y0 - 7, "1 = origin")
    canvas.line(x0, 11, x0 + 100, 11)
    for i in range(11):
        canvas.line(x0 + 10 * i, 11, x0 + 10 * i, 13 if i % 5 else 15)
    canvas.drawString(x0 + 104, 10, "100 mm check")
    canvas.showPage()
    canvas.save()
    subprocess.run(
        [
            "pdftoppm",
            "-scale-to-x",
            str(width * 10),
            "-scale-to-y",
            str(height * 10),
            "-singlefile",
            "-gray",
            "-png",
            str(path.with_suffix(".pdf")),
            str(path),
        ],
        check=True,
    )
    with Image.open(path.with_suffix(".png")) as image:
        image.load()
        image.save(path.with_suffix(".png"), dpi=(254, 254))


if __name__ == "__main__":
    draw("A4", 210, 297)
    draw("A3", 297, 420)
