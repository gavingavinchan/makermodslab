# Copyright 2026 MakerMods. All rights reserved.
"""Printable ChArUco board for measuring where the real tank's cameras sit, relative to the arm base.

Gavin sets up the arm, cameras and shackle in the (dry) aquarium, lays this board at a measured spot and grabs one
frame per camera; solving the board's pose in each frame gives each camera's position and aim, which the sim scene
then copies (2026-11-06: build the sim from the real setup, not the other way round, because Dream-1 lives or dies by
what its cameras see).

The board is 7 x 5 squares of 30 mm (210 x 150 mm), which fits A4 and US Letter in landscape with margin. Print at
100% / "actual size" and check the 100 mm scale bar with a ruler: the pose solve assumes 30.0 mm squares. Laminate or
sleeve it before it goes anywhere near water. Writes the PDF and a JSON spec beside it.

    ~/dreamtrial/.venv/bin/python charuco_target.py [out_dir]
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_MM, MARKER_MM = 30.0, 22.0
DICT_NAME = "DICT_5X5_100"
PX_PER_MM = 20  # 508 dpi raster, crisp at any printer resolution
PAGE_W_MM, PAGE_H_MM = 279.4, 210.0  # the overlap of A4 and Letter, landscape
MM_PER_IN = 25.4


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_MM / 1000, MARKER_MM / 1000, dictionary)
    board_w, board_h = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    img = board.generateImage((int(board_w * PX_PER_MM), int(board_h * PX_PER_MM)), marginSize=0, borderBits=1)

    # The whole page as one raster at an exact DPI, so the PDF prints at true scale.
    page = np.full((int(PAGE_H_MM * PX_PER_MM), int(PAGE_W_MM * PX_PER_MM)), 255, np.uint8)
    left, top = (PAGE_W_MM - board_w) / 2, 22.0

    def px(mm):
        return int(round(mm * PX_PER_MM))

    page[px(top) : px(top) + img.shape[0], px(left) : px(left) + img.shape[1]] = img
    font, black = cv2.FONT_HERSHEY_SIMPLEX, 0
    cv2.putText(
        page,
        f"ChArUco {SQUARES_X}x{SQUARES_Y}, {SQUARE_MM:g} mm squares, {MARKER_MM:g} mm markers, {DICT_NAME}. "
        "Print at 100% (actual size).",
        (px(left), px(top - 6)), font, 1.6, black, 3, cv2.LINE_AA,
    )
    # 100 mm scale bar under the board
    y = px(top + board_h + 14)
    cv2.line(page, (px(left), y), (px(left + 100), y), black, 6)
    for x_mm in (0, 50, 100):
        cv2.line(page, (px(left + x_mm), y - px(2)), (px(left + x_mm), y + px(2)), black, 6)
    cv2.putText(page, "100 mm: check with a ruler", (px(left + 105), y + px(1.5)), font, 1.6, black, 3, cv2.LINE_AA)
    pdf = OUT / "charuco_7x5_30mm.pdf"
    dpi = PX_PER_MM * MM_PER_IN
    Image.fromarray(page).save(pdf, resolution=dpi)
    Image.fromarray(page).resize((page.shape[1] // 4, page.shape[0] // 4)).save(OUT / "charuco_7x5_30mm.png")
    spec = {
        "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y,
        "square_mm": SQUARE_MM,
        "marker_mm": MARKER_MM,
        "dictionary": DICT_NAME,
        "page_mm": [PAGE_W_MM, PAGE_H_MM],
    }
    (OUT / "charuco_7x5_30mm.json").write_text(json.dumps(spec, indent=1) + "\n")

    # Self-check: the rendered board must detect and give every inner corner.
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detector.detectBoard(img)
    expected = (SQUARES_X - 1) * (SQUARES_Y - 1)
    found = 0 if ids is None else len(ids)
    print(f"{pdf}: {found}/{expected} ChArUco corners detected in the source image")
    if found != expected:
        sys.exit(1)


if __name__ == "__main__":
    main()
