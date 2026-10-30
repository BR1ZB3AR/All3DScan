#!/usr/bin/env python3
"""Printable marker sheets for scanning with marker tracking.

Markers are black rings with the paper showing through the middle. The
scanner finds the bright middle and takes its centre; the ring is what makes
that middle stand out from the page around it.

    python3 tools/make_marker_sheet.py out/            # both sheets
    python3 tools/make_marker_sheet.py out/ --grid     # only the regular one

## Why the layout is scattered rather than a grid

Marker tracking does not know one marker from another. It recognises the
*constellation*: it measures the distances between the markers it can see
and looks for the same set of distances in the map it has built. Those
distances do not change however the scanner is held, which is what makes
them usable.

A regular grid defeats that. Every cell has the same distances to its
neighbours as every other cell, so a patch of grid matches the map in many
places at once and the scan can lock onto the wrong one. Scattering the
markers makes each neighbourhood's distances unique, so there is one answer.

The grid sheet is here to compare against, not to scan with.

## Printing

Print at 100%. "Fit to page" or "shrink to fit" scales the sheet, and then
every distance the scanner measures is wrong by that factor — which comes
back as a scan that is the wrong size or the wrong shape. The sheet carries
a 100 mm bar: measure it after printing, and if it is not 100 mm the scale
is wrong and nothing else on the page can be trusted.

Matte paper, and a laser printer if there is one. Glossy paper throws the
scanner's own light straight back at it in one spot and blinds it there.
"""

from __future__ import annotations

import argparse
import math
import random
import subprocess
import shutil
from pathlib import Path

# A4, in millimetres.
PAGE_W, PAGE_H = 210.0, 297.0

# The marker. The ring's outer edge, and the clear middle the scanner
# actually finds; Revopoint's own adhesive markers are 6 mm across the
# middle, so a sheet made to this size can be mixed with them.
RING_OUTER_MM = 10.0
CENTRE_MM = 6.0

MARGIN_MM = 14.0
#: Centres no closer than this, so two markers never merge into one blob.
LEAST_APART_MM = 15.0
#: The grid the scattered sheet starts from, before it is jittered.
PITCH_MM = 24.0
#: How far a marker may wander from its cell, as a share of the pitch. Up to
#: a half would let two markers from neighbouring cells meet.
JITTER = 0.38


def grid_centres() -> list[tuple[float, float]]:
    """A regular grid, filling the page inside the margins."""
    usable_w = PAGE_W - 2 * MARGIN_MM
    usable_h = PAGE_H - 2 * MARGIN_MM - 16.0  # room for the footer
    columns = int(usable_w // PITCH_MM) + 1
    rows = int(usable_h // PITCH_MM) + 1
    span_x = (columns - 1) * PITCH_MM
    span_y = (rows - 1) * PITCH_MM
    left = (PAGE_W - span_x) / 2
    top = MARGIN_MM + (usable_h - span_y) / 2
    return [(left + c * PITCH_MM, top + r * PITCH_MM)
            for r in range(rows) for c in range(columns)]


def scattered_centres(seed: int = 20261030) -> list[tuple[float, float]]:
    """The same grid, with every marker pushed off its cell.

    Starting from a grid rather than scattering freely keeps the sheet
    evenly covered — the scanner needs markers wherever it looks — while
    the jitter is what makes each neighbourhood's distances its own.
    """
    rng = random.Random(seed)
    kept: list[tuple[float, float]] = []
    reach = PITCH_MM * JITTER
    for x, y in grid_centres():
        for _ in range(40):  # a few tries before leaving this cell empty
            nudged = (x + rng.uniform(-reach, reach), y + rng.uniform(-reach, reach))
            if not inside(nudged):
                continue
            if all(math.dist(nudged, other) >= LEAST_APART_MM for other in kept):
                kept.append(nudged)
                break
    return kept


def inside(centre: tuple[float, float]) -> bool:
    """Whether a marker at this centre sits wholly on the page, clear of
    the margin and the footer."""
    edge = RING_OUTER_MM / 2
    x, y = centre
    return (MARGIN_MM + edge <= x <= PAGE_W - MARGIN_MM - edge
            and MARGIN_MM + edge <= y <= PAGE_H - MARGIN_MM - 16.0 - edge)


def closest_pair(centres: list[tuple[float, float]]) -> float:
    return min((math.dist(a, b) for i, a in enumerate(centres) for b in centres[i + 1:]),
               default=0.0)


def how_unique(centres: list[tuple[float, float]], tolerance: float = 1.5) -> float:
    """How well the constellation can be told apart from itself.

    Takes each marker's distances to its six nearest neighbours as its
    signature, and reports the share of markers whose signature no other
    marker shares within `tolerance` mm. On a regular grid almost every
    marker has the same signature; scattered, almost none do.
    """
    if len(centres) < 8:
        return 0.0
    signatures = []
    for centre in centres:
        near = sorted(math.dist(centre, other) for other in centres if other != centre)[:6]
        signatures.append(near)
    alone = 0
    for at, mine in enumerate(signatures):
        if all(
            any(abs(m - t) > tolerance for m, t in zip(mine, theirs))
            for other, theirs in enumerate(signatures)
            if other != at
        ):
            alone += 1
    return alone / len(signatures)


def svg(centres: list[tuple[float, float]], title: str, note: str) -> str:
    """The sheet, in real millimetres so it prints at the right size."""
    ring = RING_OUTER_MM / 2
    middle = CENTRE_MM / 2
    marks = "\n".join(
        f'  <circle cx="{x:.3f}" cy="{y:.3f}" r="{ring:.3f}" fill="#000000"/>\n'
        f'  <circle cx="{x:.3f}" cy="{y:.3f}" r="{middle:.3f}" fill="#ffffff"/>'
        for x, y in centres
    )
    footer_y = PAGE_H - MARGIN_MM - 7.0
    bar_y = footer_y - 7.0
    bar_left = MARGIN_MM
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_W}mm" height="{PAGE_H}mm"
     viewBox="0 0 {PAGE_W} {PAGE_H}">
  <rect width="{PAGE_W}" height="{PAGE_H}" fill="#ffffff"/>
{marks}
  <!-- Measure this after printing. If it is not 100 mm, the print was
       scaled and every distance on the page is wrong by that factor. -->
  <g stroke="#000000" stroke-width="0.35" fill="none">
    <line x1="{bar_left}" y1="{bar_y}" x2="{bar_left + 100}" y2="{bar_y}"/>
    <line x1="{bar_left}" y1="{bar_y - 2}" x2="{bar_left}" y2="{bar_y + 2}"/>
    <line x1="{bar_left + 50}" y1="{bar_y - 1.4}" x2="{bar_left + 50}" y2="{bar_y + 1.4}"/>
    <line x1="{bar_left + 100}" y1="{bar_y - 2}" x2="{bar_left + 100}" y2="{bar_y + 2}"/>
  </g>
  <text x="{bar_left + 103}" y="{bar_y + 1.6}" font-family="DejaVu Sans, sans-serif"
        font-size="3.4" fill="#000000">100 mm — measure me; print at 100%</text>
  <text x="{bar_left}" y="{footer_y}" font-family="DejaVu Sans, sans-serif"
        font-size="3.2" fill="#000000">All3DScan · {title} · {len(centres)} markers · {CENTRE_MM:.0f} mm centres in {RING_OUTER_MM:.0f} mm rings</text>
  <text x="{bar_left}" y="{footer_y + 4.2}" font-family="DejaVu Sans, sans-serif"
        font-size="3.2" fill="#555555">{note}</text>
</svg>
"""


def write_sheet(into: Path, name: str, centres: list[tuple[float, float]],
                title: str, note: str) -> Path:
    drawing = into / f"{name}.svg"
    drawing.write_text(svg(centres, title, note))
    pdf = into / f"{name}.pdf"
    for tool in (["rsvg-convert", "-f", "pdf", "-o", str(pdf), str(drawing)],
                 ["inkscape", str(drawing), "--export-type=pdf", f"--export-filename={pdf}"]):
        if shutil.which(tool[0]) and subprocess.run(tool, capture_output=True).returncode == 0:
            return pdf
    return drawing  # no converter here; the SVG prints from a browser


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("into", type=Path, help="where to write the sheets")
    parser.add_argument("--grid", action="store_true",
                        help="only the regular grid, for comparing against")
    parser.add_argument("--scattered", action="store_true",
                        help="only the scattered sheet, which is the one to scan")
    args = parser.parse_args()
    args.into.mkdir(parents=True, exist_ok=True)

    both = not (args.grid or args.scattered)
    written = []
    if both or args.scattered:
        centres = scattered_centres()
        written.append((
            write_sheet(args.into, "markers_scattered", centres, "scattered markers",
                        "Scattered: every neighbourhood has its own distances."),
            centres, "scattered"))
    if both or args.grid:
        centres = grid_centres()
        written.append((
            write_sheet(args.into, "markers_grid", centres, "regular grid",
                        "A grid repeats itself — for comparison, not for scanning."),
            centres, "grid"))

    for path, centres, kind in written:
        print(f"{path}")
        print(f"   {len(centres)} markers, closest pair {closest_pair(centres):.1f} mm apart")
        print(f"   neighbourhoods that are unique to within 1.5 mm: "
              f"{how_unique(centres):.0%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
