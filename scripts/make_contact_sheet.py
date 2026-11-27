#!/usr/bin/env python3
"""make_contact_sheet.py -- compose a labelled contact sheet from one sortie's figure set.

  python3 scripts/make_contact_sheet.py --tag phase102a-01-01 \
      --out docs/evidence/final/sheet-BEST-102a.png

Every image is read from docs/evidence/final/, so the sheet is a view of the evidence that is already
in the repo (nothing is re-rendered here).  Titles are drawn on a dark bar so the sheet stays readable
when the individual PNGs have white backgrounds.
"""
from __future__ import annotations

import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EV = os.path.join(ROOT, "docs", "evidence", "final")


def _font(size):
    for p in ("C:/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/msyh.ttc",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="sortie tag, e.g. phase102a-01-01")
    ap.add_argument("--out", required=True)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--cell", type=int, default=760)
    a = ap.parse_args()
    tag = a.tag
    names = [
        ("%s_topdown.png" % tag, "topdown 全趟（真值/指令/LIO）"),
        ("%s_laps-launch.png" % tag, "launch 段"),
        ("%s_laps-lap1.png" % tag, "lap 1"),
        ("%s_laps-lap2.png" % tag, "lap 2"),
        ("%s_xyz-t.png" % tag, "x/y/z - t"),
        ("%s_-vnorm-t.png" % tag, "各轴速度 - t"),
        ("%s_-speed-t.png" % tag, "速度 2-范数 - t"),
        ("phase%s-clearance.png" % tag.replace("phase", "").replace("-01-01", ""),
         "clearance（真值到柱面）"),
        ("planexec-%s.png" % tag.split("-")[0].replace("phase", ""), "P1 规划-执行偏差"),
        ("handover-audit-%s.png" % tag.split("-")[0].replace("phase", ""), "换段/腿起点 audit"),
        ("%s_corner-1.png" % tag, "corner 1"),
        ("%s_corner-2.png" % tag, "corner 2"),
        ("%s_corner-3.png" % tag, "corner 3"),
        ("%s_corner-4.png" % tag, "corner 4"),
        ("%s_corner-5.png" % tag, "corner 5"),
        ("%s_corner-6.png" % tag, "corner 6"),
    ]
    items = []
    for fn, title in names:
        p = os.path.join(EV, fn)
        if os.path.isfile(p):
            items.append((p, title))
    if not items:
        raise SystemExit("no figures matched tag %s" % tag)
    cell, cols = a.cell, max(1, a.cols)
    bar = 34
    rows = (len(items) + cols - 1) // cols
    pad = 10
    W = cols * cell + (cols + 1) * pad
    H = rows * (cell + bar) + (rows + 1) * pad
    sheet = Image.new("RGB", (W, H), (18, 18, 20))
    draw = ImageDraw.Draw(sheet)
    f, fb = _font(22), _font(30)
    draw.text((pad, 6), "super_super_apm — %s 全套图" % tag, fill=(240, 240, 240), font=fb)
    for i, (p, title) in enumerate(items):
        r, c = divmod(i, cols)
        x = pad + c * (cell + pad)
        y = pad + r * (cell + bar + pad) + 30
        im = Image.open(p).convert("RGB")
        im.thumbnail((cell, cell), Image.LANCZOS)
        bx = x + (cell - im.width) // 2
        by = y + (cell - im.height) // 2
        sheet.paste(im, (bx, by))
        draw.rectangle([x, y + cell, x + cell, y + cell + bar], fill=(32, 32, 36))
        draw.text((x + 8, y + cell + 6), title, fill=(230, 230, 230), font=f)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    sheet.save(a.out)
    print("written:", a.out, sheet.size)


if __name__ == "__main__":
    main()
