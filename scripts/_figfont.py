#!/usr/bin/env python3
"""_figfont.py -- make the figure scripts find a font that has BOTH Han glyphs and Latin punctuation.

The three figure scripts set

    rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", ...]

which assumes a Windows box (previous rounds drew the figures on the Windows side with anaconda's
Python) or a distro with Noto CJK.  This machine's WSL has neither: `fc-list` finds no CJK family, and
matplotlib's own list contains only **Droid Sans Fallback**, which has the Han glyphs but NOT basic
Latin punctuation -- and matplotlib 3.10 does not fall back per glyph here, it picks one family for the
whole string.  So either the Chinese titles or the commas/parentheses come out as boxes.

`ensure_cjk_font()` fixes that without touching the scripts' look: if none of the requested families is
registered, it registers a real CJK font straight out of the Windows font directory (reachable inside
WSL at /mnt/c/Windows/Fonts) and returns its family name.  Callers just put the returned name first.
"""
import os

import matplotlib
import matplotlib.font_manager as fm

CANDIDATES = [
    ("/mnt/c/Windows/Fonts/msyh.ttc", "Microsoft YaHei"),      # has Han + Latin, the round-83 look
    ("/mnt/c/Windows/Fonts/msyhbd.ttc", "Microsoft YaHei"),
    ("/mnt/c/Windows/Fonts/simhei.ttf", "SimHei"),
    ("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", "Droid Sans Fallback"),
]
WANT = ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei")


def ensure_cjk_font(verbose=True):
    """Register a CJK font if needed and return the family name to put first in font.sans-serif."""
    have = {f.name for f in fm.fontManager.ttflist}
    for name in WANT:
        if name in have:
            return name
    for path, fam in CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            fm.fontManager.addfont(path)
        except Exception:                                    # noqa: BLE001
            continue
        # the family a .ttc reports may differ from the file name; take what was just added
        new = {f.name for f in fm.fontManager.ttflist} - have
        pick = fam if fam in new else (sorted(new)[0] if new else None)
        if pick:
            if verbose:
                print("  figure font: registered %s from %s" % (pick, path))
            return pick
    if verbose:
        print("  figure font: NO CJK family found -- Chinese titles will be boxes")
    return "DejaVu Sans"


def apply(verbose=True):
    """Set the rcParams chain so Han and Latin both render, and return the chosen family."""
    fam = ensure_cjk_font(verbose=verbose)
    matplotlib.rcParams["font.sans-serif"] = [fam, "Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                                              "WenQuanYi Zen Hei", "DejaVu Sans",
                                              "Droid Sans Fallback"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    return fam
