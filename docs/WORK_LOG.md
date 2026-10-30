# Work log

What has been built, what was measured, and what is still open. Newest
first. Every number here was measured on this machine — a Revopoint POP 4
(serial D26545575B6S00E1B) on CachyOS with an RTX 2060 — not estimated.

The findings ledger for the scanner's protocol is
[`docs/protocol/RE_NOTES.md`](protocol/RE_NOTES.md); this is the record of
the application around it.

---

## 2026-10-30 — Beads, and depth that was rounded to whole millimetres

**The beads had two causes in the renderer, both mine.** Eye-dome lighting
read the sub-millimetre depth steps between neighbouring splats as contours
and drew a dark ring round every splat; it now ignores steps under 1.5 mm,
three times the per-frame noise. And splats were hard-edged discs meeting
at edges; they are drawn in two passes now, depth then soft-rimmed colour,
blended where they overlap. Close up, a cloud 0.75 mm apart drawn 15 px
wide is still visibly a mosaic — at that zoom the view of a scan is its
mesh, which Live mesh is for.

**Depth was truncated to whole millimetres before the cloud was built.**
Found by holding the scanner still: 68% of pixels returned the identical
integer for 32 frames. Fixed by carrying the float depth through and
rounding the whole-mm copy. Measured: every point now sub-millimetre; the
systematic ~0.5 mm shrink toward the camera gone (per-point error −0.50 to
+0.50, mean +0.005); per-frame thickness 0.44–0.48 → **0.400 mm**.

**Tried and reverted:** re-seeding a lost frame from the centroids. A/B
showed no gain, and the test that keeps a frame half a metre away from
being merged went red — sliding middles together lets any similar shape
fit anywhere.

## 2026-10-30 — The 3D view, and settings for it

**The renderer was already on the GPU; shading was what was missing.**
Measured first: 350,000 points draw at 60 fps, which is the display's limit,
so speed was never the problem. Both shaders were a single lamp at the eye,
`0.3 + 0.7 * |N.z|`, which gives everything facing the viewer the same
brightness.

Now three lights — a key off to one side, a dim fill, a rim — the same for
splats and surfaces, so a scan does not change character when it is meshed.
The highlight is tinted towards the surface's own colour rather than left
white: the cloud's colour says whether a scan is running, and a white
highlight washes that out.

**Eye-dome lighting** draws a cloud offscreen and then shades it by its own
depth, so contours and holes show without meshing. 60.7 fps on a real
99,444-point scan.

| | |
| --- | --- |
| Anti-aliasing asked for | 4 samples |
| Granted | `GL_SAMPLES = 4`, `GL_SAMPLE_BUFFERS = 1` |
| Context on this machine | OpenGL ES 3.2 via EGL (Wayland) |

**Settings** sits in the menu bar beside About: smoothing, depth shading,
point size and how many points are drawn at once, behind Performance /
Quality / Custom presets, plus which window system to start on. Apply hands
the settings back without closing, so a change can be judged against the scan
behind it.

**Two bugs the tests caught, not the reasoning.** Giving a background
neighbour an invented distance turned every point with nothing behind it
black — most of a sparse cloud. And when Qt rebuilds the context the
offscreen target is remade, so the remembered size has to be cleared with it,
or the textures are never attached and everything draws into nothing.

## 2026-10-30 — Meshing a cloud with no depth views

Meshing only ever worked on a scan taken here, because it melts that scan's
own depth views together. Every scan from every other scanner had a dead
Mesh button.

Screened Poisson reconstruction fills the gap
(`third_party/PoissonRecon`, wrapped by `mesh_cpp/CloudSurface`). The
depth-view path is kept and still preferred where it applies: the views
record where the scanner looked and saw nothing, so a hole stays a hole.

**The fit closes over everything, including where the scan never reached**,
so each vertex carries how much data stood behind it and the thinnest are cut
away:

| | spans |
| --- | --- |
| the points | 270 × 152 × 131 mm |
| untrimmed | 297 × 233 × 214 |
| **trimmed 10%** (the default) | **276 × 156 × 129** |
| trimmed 25% | 254 × 143 × 117 — eating real surface |

End to end: 350,435 points opened from PLY → isolated to 99,444 → 100,425
triangles in 1.1 s.

**Licence.** Upstream's `LICENSE` says MIT; 94 of its 95 source files carry
BSD-3-Clause headers, and every vendored file is one of them. See
[`third_party/PoissonRecon/NOTICE.md`](../third_party/PoissonRecon/NOTICE.md).

## 2026-10-29 — Isolation, and the flashing lights

**Isolation** removes the flat surface the object stands on, then everything
not joined to the largest remaining piece — in that order, because the object
is joined to the room *through* the surface it stands on. On a real scan:
350,435 points spanning 471 × 492 × 311 mm → 99,444 spanning
270 × 152 × 131, in 0.05 s. Guarded both ways: a scan *of* something flat is
kept whole, and a step that would leave almost nothing is not taken.

**The flashing lights were a slow pulse at 500 ms**, chosen on the reasoning
that faster would crowd the captures. That was never measured, and is not
true:

| interval | 500 | 250 | 160 | 120 | 80 | 50 | 500 again |
| --- | --- | --- | --- | --- | --- | --- | --- |
| frames/s | 7.48 | 7.48 | 7.47 | 7.47 | 7.51 | 7.51 | 7.44 |

Against 6.56 with no flashing at all. Now 120 ms; below about 130 ms nothing
more is gained, since the light is only switched between captures.

## 2026-10-29 — Live preview, and reading other scanners' files

**The cameras only woke up once a scan started.** Connecting gave a status
bar saying "connected" and two panels saying "No signal", so there was no way
to aim the scanner. They are now live whenever a scanner is connected, at
10 fps, and nothing a preview frame carries is counted or kept.

**Interchange.** The app could read nothing and could save only STEP
primitives — a scan taken here could not even be kept. It now reads and
writes PLY (text, binary, either byte order), OBJ, STL, PCD, and plain
columns (`.asc` `.xyz` `.pts` `.txt` `.p3`). Checked against Blender 5.2 in
both directions, with four of its files committed as fixtures.

Two things files leave out are worked out: normals (0.994 median agreement
with the sensor's own, on 95% of points) and metres-vs-millimetres.

**Every known scanner on the bus is now named**, whether or not it can be
driven; one that cannot says so and points at File → Open.

## 2026-10-28 — Real scanning

**The mock was removed from the app.** The POP 4 is the scanner; the stand-in
survives behind `--mock` for development.

**A scanner held still kept almost no views to mesh from** — 3 views from a
142-frame scan, a 42-triangle surface. Now one is kept every few merged
frames: 47–80 views, 8,142–30,532 triangles.

**A narrower depth search finds more surface:**

| range (mm) | 250–500 | 250–420 | 270–400 | 290–370 | 250–500 again |
| --- | --- | --- | --- | --- | --- |
| points/frame | 4,161 | 4,724 | 5,172 | **5,336** | 4,106 |

Auto now narrows onto the object after setting exposure, but only when the
band is really narrower — whatever falls outside it is lost.

---

## Still open

- **Coverage is 2.5–3%** of the depth camera's pixels. This is the dominant
  limit on scan quality and nothing done so far touches it.
- **The fused model is 2.5× furrier than its own frames** (1.18 mm against
  0.47 mm), which is about 1 mm of pose error per frame. The obvious fix —
  pulling points onto their neighbours' plane — was built and **rejected**:
  measured against independent frames it moved the model *away* from them
  (0.359 → 0.629 mm) and halved real curvature.
- **Overlap, Smooth and Simplify** are still dead buttons. Smooth needs the
  quadric version and a measured check before it ships, given the above.
- **The flatness/bow question is unresolved.** Four frames of one unchanged
  scene gave bow estimates of −3.8, −25, +56 and +107 mm per 100 mm: the
  patches available span too little radius to fit a curve to. Settling it
  needs a flat surface filling the field of view.
- **Live capture is POP 4 only.** The Einstar Rockit protocol contributed in
  [issue #2](https://github.com/BR1ZB3AR/All3DScan/issues/2) is unverified
  here and nothing from it is wired up.
- **Format readers are checked against Blender only**, not against Revo Scan,
  Creality or Einstar exports — which is where such readers usually break.
