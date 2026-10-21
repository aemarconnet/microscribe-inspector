# Roadmap

© 2026 Marconnet Technologies — licensed under [CC BY 4.0](LICENSE-docs).

What is planned, in the order it will be worked on. The list is shaped by what a manual hard-probe
arm can do well: positions are captured with the pedals (there is no touch-trigger probe), the
stylus has a fixed ball or point tip, and accuracy is in the few-tenths-of-a-millimetre class over a
reach of about 1.27 m. A pass/fail decision is only trustworthy when the tolerance is roughly four
times the arm's measuring error, so the focus is on dependable basics rather than fine GD&T.

Effort: **S** small · **M** medium · **L** large.

## Phase 1 — Trustworthy numbers

1. [x] **Ball compensation for curves, freeform surfaces, lines and single points** (S–M) —
   offset along the surface normal (freeform surfaces), perpendicular to the curve in the stylus
   direction or in the curve's plane (curves), or along the stylus (points, lines).
2. [x] **Movement check and hands-free capture** (S) — reject a capture when the stylus moved during
   the averaging window; optional dwell capture when the stylus is held still.
3. [x] **Probe qualification on a reference sphere** (S) — measure a ball of known diameter to find a
   tip's effective radius and its error; kept per tip with the date.
4. [x] **Arm check** (M) — single-point articulation test in a cone seat and a length test on a known
   gauge, in the spirit of ASME B89.4.22, with a history and pass/fail against your own limit.
5. [x] **Measurement capability in reports** (S) — the latest arm-check result is shown with the
   results; tolerances tighter than 4× of it are flagged once tolerances exist (item 6).

## Phase 2 — Inspection against nominal values

6. [x] **Nominal values and tolerances** on measurements, pass/fail colours and an
   out-of-tolerance summary (M).
7. [x] **Basic GD&T** (M–L) — flatness, straightness, roundness, cylindricity, parallelism,
   perpendicularity, angularity, position, coaxiality; the part coordinate system as the datum
   reference frame; optional minimum-zone evaluation next to least squares.
8. [x] **Nominal model import** (L) — STL: display, point-pair and best-fit alignment to the model,
   live stylus-to-model deviation, points coloured by deviation, profile of a surface or of
   comparison points. (STEP import of the nominal model is a later addition.)
9. [x] **Report with pictures** (M) — HTML (print to PDF from the browser) with a 3D snapshot,
   tables, pass/fail, logo and part, serial and operator fields; CSV results log per part.
10. [x] **Inspection plans: learn and replay** (L) — a measured project is the plan: features in
    order with their notes as prompts, nominals and tolerances; the next part is measured step by
    step with markers where to probe; results per part feed simple SPC (trend, Cp/Cpk).

## Phase 3 — Capture and digitizing to CAD

11. [x] **More hard-probe scan modes** (M) — fixed time, section planes (a point each time the stylus
    crosses parallel planes at a set spacing) and free point-cloud scanning with filtering.
12. [x] **2D sketch from a probed profile** (M) — project onto a plane, split into lines and arcs,
    export DXF and SolidWorks curve files.
13. [x] **More feature types** (M) — round-ended slot, rectangle (square slot, pocket, tab) and
    ellipse, with ball compensation. Fillets along straight edges measure as (partial) cylinders;
    sheet-metal vector points are comparison points against the nominal model (item 8); width and
    keyways come with the mid-plane construction (item 14).
14. [x] **More constructions** (S–M) — offset plane, mid-plane, projection onto a plane, plane
    through a line parallel/perpendicular to a datum, circle or plane through hole centres (bolt
    circle), mirror.
15. [x] **Constrained fits** (M) — fixed diameter or cone angle, forced parallel/perpendicular to a
    datum.
16. [x] **More export formats** (M) — IGES (exact B-spline surfaces and curves), STL mesh of the
    surfaces, DXF (2D sketches, item 12).

## Phase 4 — Larger parts and conditions

17. [x] **Leapfrog** (M) — move the arm using three or more cone seats or spheres, best-fit the
    transform and report the added error.
18. [x] **Temperature correction** (S) — part temperature and material entered by hand, results
    scaled to 20 °C.
19. [ ] **Faster sampling** (S) — 115200 baud for denser scans. A "Find fastest…" test (Arm & tip →
    Connection) measures the readings per second at each baud rate and keeps the best; still to be
    confirmed on a real arm.

## Ongoing

- [ ] Results as QIF / DMIS for quality-management systems (needs a consumer to validate against)

- [ ] Confirm STEP import in SolidWorks, Fusion, FreeCAD and Onshape; publish example files
- [ ] Test reports from other serial MicroScribe models (DH1.0 parameter format, 6-axis stylus)
- [ ] SpaceMouse: per-axis tuning presets
- [ ] Localisation (German, French, Spanish…)

## Not planned

Things this kind of arm cannot do well: touch-trigger hit detection, automated (CNC) measuring
programs, laser-scanner workflows, probe changers and star styli, tight GD&T well under ±1 mm,
threads, surface roughness and features smaller than the probe ball.
