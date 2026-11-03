# Four TAGs: two CS pairs, and a reflector next to another board

**Question.** With two more TAGs on the bench (N004, N005), two CS pairs ran at once and both
read 10–20 m over 2–2.5 m. Is it the pairs interfering, battery power, or placement?

## Setup

- **Nodes:** N001 ranges with N002, N005 with N004; all four report to the sink (N003) over
  their own sink links, the reflectors at 1 s. Firmware `0.1.0+0f39a7d3` on N004 and N005,
  `796d5839-dirty` on N001 and N002.
- **Placement:** the two initiators together, the two reflectors about 20 cm apart, about
  2–2.5 m between the groups. Later moves as the rows say.
- **Measure:** `navgrid monitor` for 60 s per row, the ranges' median, 10th and 90th percentiles
  and quality, from `stats.py` over the monitor's text.
- **Stopping a pair:** re-provisioning its initiator without a peer, through the probe. (A
  `probe-rs reset --halt` does nothing: the option doesn't exist.)
- **Reference:** earlier today, N001–N002 alone, both DK-powered, 2 m apart: median 1.57 m,
  p10 1.49, p90 2.18, sigma 0.68 m, quality ~254.

## Results

| # | Arrangement | Pair 1 (N001–N002) | Pair 2 (N005–N004) |
|---|---|---|---|
| 1 | N002 moved onto battery, N004 enrolled (sink link only, no CS) and placed 20 cm from it | median 2.30 m, p90 31.7 m | – |
| 2 | N005 enrolled: both pairs running | median 10.9 m, p10 6.3, p90 52 m, quality 142 | median 9.6 m, p10 6.0, p90 49 m, quality 148 |
| 3 | Pair 1 stopped | – | median 19.8 m, p10 8.3, p90 75 m, quality 123 |
| 4 | N001 restarted on a battery; N004 moved into DK …0413, away from N002 | median 2.49 m, p10 1.97, p90 2.98, quality 243 | median 2.67 m, p10 2.52, p90 15.2, quality 209 |
| 5 | N002 put 20 cm from the DK with N004 | median 8.0 m, p10 6.7, p90 14.4, quality 177; 90 procedures lost in the minute | median 3.0 m, p90 19.3, quality 196 |
| 6 | N004 unplugged, dead, still there | median 8.7 m, p10 6.7, p90 20.6, quality 197 | – |
| 7 | N004 back in the DK; N002 moved to 50 cm from the DK | median 9.45 m, p10 8.2, p90 10.8, sigma 2.5, quality 206: a tight wrong answer | median 2.53 m, p10 2.42, p90 2.83, sigma 0.73, quality 237 |
| 8 | A metal jar near the DK removed; the laptop 50 cm from the DK stays | median 9.2 m, p10 5.5, p90 10.2, quality 220 | median 2.66 m, p10 2.51, p90 2.87, quality 250 |
| 9 | N002 moved to 1 m from the DK, the laptop and the hub | median 4.0 m, p10 2.6, p90 4.9, sigma 2.1, quality 233 (true distance to N001 not yet measured) | median 2.73 m, p10 2.51, p90 2.98, quality 245 |
| 10 | N001 moved about 1 m (which of the two battery TAGs was N001 was found this way: pair 1 changed, pair 2 didn't) | median 3.48 m, p10 3.23, p90 3.69, sigma 0.88, quality 247; **N001–N002 measured about 1.0 m** | median 2.65 m, p10 2.48, p90 2.85, quality 247; **N005–N004 measured about 0.8 m** |

In the bad rows N001's estimator reported coherence 0.54 and a fit residual of 1.37 rad over
70 pairs, against 0.998 and ~0.3 rad in the reference: the phase data itself is corrupt, not
biased.

Battery power was ruled out by row 4: both of pair 1 on batteries, and its best numbers of the
day. CS interference between the pairs was ruled out by rows 3 and 6: a pair alone, and a dead
neighbour, read the same.

| 11 | All four on batteries on the floor, pairs 1.5 m apart (an 8.4 × 6.1 m rectangle was meant; the pairs were on its 6.1 m sides, see row 12) | median 7.3 m, p10 6.9, p90 7.9, quality 221 | median 9.6 m, p10 8.7, p90 10.3, quality 210 |
| 12 | All four on the tops of office half-walls, 1.7 m up, pairs 6.1 m apart with partitions between | median 20 m, p10 14, p90 66, quality 129; 162 empty procedures in the minute | median 12.5 m, p10 10, p90 55, quality 156 |
| 13 | N001 and N002 cabled to the DKs on the desk, 40 cm apart (the cables' reach, as this morning) | median 2.05 m, p10 1.75, p90 2.26, quality 242; a later minute 1.88 m | (6.1 m, as row 12) median 13.3 m |

Row 11's distance was reported as "about 1.5 m" at the time and corrected to 6.1 m afterwards
with the layout of row 12; which it was is not certain, so row 11 stands as reported.

## The capture at 40 cm

Both boards flashed with `NAVGRID_LOG_STEPS`, 60 s of raw subevent results from each (660 and
654), joined by `navgrid cs`: 149 procedures with a phase-slope estimate from the raw tones.

| Estimator | Median | p10 | p90 |
|---|---|---|---|
| On the device (`PhaseSlope`, 4 paths) | 1.88 m | 1.73 | 1.99 |
| Host, from the raw tones | 1.96 m | 1.84 | 2.09 |

For a true 0.40 m. The tones themselves carry the 1.5 m: it is the radio's delay, the
calibration offset (DEF-6), not the estimator.

## Also seen

- N001 provisioned with `--no-diversity` produced no ranges at all in 90 s (its log wasn't
  captured). Single-path ranging may be broken; to check with a probe on an initiator.
- Both pairs at once cost each 11.0 → 10.6 procedures/s.
- Fresh CR2032s read 2.80–2.95 V under this load after an hour or two; the monitor's day
  estimate then says "0 days to low", which is the trend fit, not the cells (DEF-52).
- The monitor named N005 by its ID all day: it was started before N005 was enrolled.

## Conclusion

- **No bug at short range.** Every clean reading sat 1.5–2.5 m above the true distance, the
  morning's included (those TAGs were also 40 cm apart, the cables' reach). The raw tones show
  the same offset, so it is the uncalibrated radio delay, which `navgrid calibrate` (DEF-6)
  is for. Nordic's SDC has a "board distance offset" for the same thing.
- **Placement corrupts the phase data,** and that is the day's real finding:
  - a board or an object within about 0.5 m of a TAG (another TAG, dead or alive; a DK; a metal
    jar nearby) gave scattered readings with low quality or a tight wrong one (rows 5–8);
  - at 1 m clear, clean (rows 9–10);
  - flat on the floor, tight readings of 7–10 m (row 11): the floor reflection at grazing
    incidence cancels the direct path, and the bounces off the room are what's left;
  - 6.1 m through office partitions: scattered, 2–3× long, a quarter of the procedures empty.
- **Not the batteries** (row 4 and row 13), **not the other pair's procedures** (rows 3 and 6).
- **What it means for the design:** nodes need clearance from other boards and metal, and the
  phase-slope estimator has no multipath rejection; keys on the floor, the use case, need an
  estimator that finds the first path (an IFFT or super-resolution one, as NCS's `cs_de`), and
  captures from the floor to develop it on.

## Evening, at home: captures on a wooden desk, and a bisect

N001 and N002 cabled to the DKs on an IKEA desk, firmware with `NAVGRID_LOG_STEPS`, 60 s raw
captures (`docs/engineering/captures/2026-11-03-home/`), both host estimators on each. Lying flat
on the desk, with 2 × 2 antennas and then one antenna each (`--no-diversity` on both):

| True | Antennas | Phase slope | First path |
|---|---|---|---|
| 1.25 m | 2 × 2 | 4.34 m; per antenna path 9.5 / 4.2 / 8.6 / 3.2 | paths 1 and 3: 3.4 and 3.1 |
| 0.80 m | 2 × 2 | 4.50 m; per path 4.9 / 3.6 / 7.8 / 5.9 | 2.56 m, 22 % of the strongest |
| 0.80 m | 1 × 1 | 4.79 m, p10 4.48, p90 5.22 | 2.33 m, p10 1.95, p90 2.76, 13 % |
| 0.40 m | 1 × 1 | 2.43 m, p10 2.24, p90 2.85 | 2.80 m |

The strongest component sat at 3–4.5 m whatever the distance; the first-path estimator's earliest
component tracked the true distance plus 1.7–1.9 m at 0.8 and 1.25 m, under a component 9 dB
stronger. One antenna each made the phase slope tight but no truer, so the antenna switch is not
the cause (that was a suspect: paths 0 and 2 read 8–9 m at 1.25 m while 1 and 3 read 3–4 m).

**The bisect.** Were the day's firmware changes the cause? Yesterday's last commit (`78f97aa`),
built with the raw-step logging and provisioned with its own tool, against today's, at one
placement, N002 on the desk edge and N001 on a chair 30 cm lower, 60 cm apart, one antenna each:

| Firmware | Phase slope | p10–p90 | First path |
|---|---|---|---|
| `78f97aa`, 2026-11-02 | 2.88 m | 2.83–2.96 | 2.60 m |
| `0f39a7d3`, today | 2.84 m | 2.79–2.94 | 2.60 m |

The same to 4 cm. Nothing today changed what the radios measure. The 2.0–2.3 m over the true
0.6 m is yesterday's "reads about 2.4 m long" (DEF-6). And with both TAGs off the desk surface
the readings were tight again, where flat on the desk at 0.8 m they read 4.8 m: the desk, at
grazing incidence, is this evening's reflector, as the floor was in the afternoon.

## Night: a calibration series that isn't one

N005 ranging with N004, both on batteries, flat on a box and a chair 0.35 m above the floor, in
a clear room (4 m ceiling, metal HVAC at 3.4 m, nothing else within 2 m), the device's own
ranges through the sink, 60 s a point (`captures/2026-11-03-home/cal/`):

| True | 1.0 | 1.5 | 2.0 | 3.0 | 4.0 | 5.0 | 2.0 again | 6.0 | 6.0, N005 flipped |
|---|---|---|---|---|---|---|---|---|---|
| Median | 5.92 | 3.56 | 3.33 | 7.52 | 7.58 | 7.17 | 3.58 | 8.25 | 9.77 |
| Quality | 246 | 230 | 245 | 238 | 237 | 237 | 240 | 242 | 243 |

- **Not an offset.** Only the 2 m point sat near true + 1.3 m, twice. From 3 to 5 m the reading
  was pinned near 7.5 m, and 1 m read 5.9 m.
- **Every point was tight and the device called it good:** quality 230–246 throughout, the fit
  residual of a clean single path. The quality figure can't tell a reflection from the direct
  path (DEF-53).
- **Flipping one puck moved the 6 m reading by 1.5 m** (8.25 → 9.77). The pucks had opposite
  sides up for the whole series until then.
- **Meanwhile N001–N002,** cabled at 0.6 m on the desk edge and a chair, read 2.69, 2.60 and
  2.61 m over the three thirds of the hour: a pair that sits still reads the same all evening.

What's consistent with all of it: at 0.35 m above the floor with the pucks flat, the direct
path between them is weak, and a bounce off the ceiling or the ducts is what the phase slope
reads; its distance barely depends on the pair's separation, which is what the pinned 7.5 m
looks like. The deciding test wasn't run: the same pair 1.5 m up, same side up, at 3 m.

## Late night: a round-trip-time referee, and the chairs

N005 and N004 on wooden chair seats, 0.45 m up, flat, same side up. The first two points
(2.0 m → 4.31 m, 3.0 m → 6.27 m, both tight) looked like twice the distance, so the phase path
was checked for a factor of two:

- The estimator divides the round-trip slope by 4π, and the firmware runs with inline PCT
  transfer, so the initiator's reported tone is already the round trip (Core 6.3 enhancement
  bit 0, `INLINE_PCT`). Consistent.
- **The host's "raw tones" estimate was never independent.** `navgrid cs` without `--ipt`
  multiplies the initiator's tones by the reflector's, and on the kept captures that changes
  nothing (0.6 m: 2.84 m either way, coherence 0.997 either way): under inline transfer the
  reflector's reported tone is flat, and the initiator's carries everything. So "the tones
  themselves carry the 1.5 m" in the 40 cm section above was the device's own data read back,
  not a second measurement. `scripts/cs-capture.sh` now passes `--ipt`. (The first-path
  numbers in the evening section used the same product and so a wrong amplitude weighting;
  with `--ipt` the 0.6 m capture's first path reads 0.87 m and the 0.8 m one 1.35 m, against
  2.60 and 2.33 before. Not examined further tonight.)
- **A referee that doesn't use phase:** mode-1 steps (sub-mode 1, Nordic's default too) were
  added to the procedure, and the initiator reports its mean ToA−ToD over the step's packets as
  a second `Range` with method `Rtt`, next to the phase slope. The reflector's turnaround isn't
  transferred, so the value carries a bias that is constant for a pair but differs between
  pairs; steps the controller marks unreceived (`0x8000`, which swamped the first build's means)
  are left out; the value is reported 100 m up so negative means survive the unsigned field.
  Access-address-only timing: about ±6 m per procedure of ~17 steps, ±0.3 m a minute. Three
  builds went to the four TAGs through the sink, over the air, about six minutes a round, with
  the pucks where they were.

| True (m) | 1.0 | 2.0 | 3.0 | 4.0 |
|---|---|---|---|---|
| Phase slope, median | 3.04 | 4.42 | 6.35 | 6.67 |
| RTT, mean ± s.e. | 12.34 ± 0.31 | 13.04 ± 0.29 | 15.96 ± 0.21 | 15.67 ± 0.26 |
| RTT − phase slope | 9.3 | 8.6 | 9.6 | 9.0 |

Meanwhile N001–N002, on batteries on the desk a hand apart, read 1.67–1.77 m by phase and
−0.4 to −0.9 m by RTT all along.

- **No factor of two.** The RTT follows the phase slope within half a metre across the row,
  with a 9 m bias for this pair (−2 m for the desk pair, so the bias is per pair: the
  reflector's untransferred turnaround, or the pair's frequency offset in the timing
  estimator). The phase slope reports a real path length.
- **It isn't the direct path.** 3 → 4 m moved neither method; 2 → 3 m moved the RTT 2.9 m
  and the phase 1.9 m. Two flat pucks at the same height look at each other edge-on, and what
  both methods see is a bounce whose length hardly depends on the chairs' separation.
- The product geometry, a node high and flat over a tag flat on the floor, is the opposite case
  and was the next test.

## Late night: a node high, a tag on the floor

The same pucks: N004 flat on top of a wardrobe, 2.30 m up; N005 flat on the floor, first directly
below it, then slid sideways along the floor, then flipped. True distance is the diagonal. Data in
`captures/2026-11-03-home/night2/`.

| Sideways (m) | 0 | 1 | 2 | 3 | 3, flipped |
|---|---|---|---|---|---|
| True (m) | 2.30 | 2.51 | 3.05 | 3.78 | 3.78 |
| Phase slope, median | 3.83 | 4.05 | 5.03 | 5.60 | 5.76 |
| Over true | +1.53 | +1.54 | +1.98 | +1.82 | +1.98 |
| p10–p90 | 3.73–4.0, with a tail to 43 | 3.98–4.38 | 4.86–5.13 | 5.49–5.66 | 5.58–5.85 |
| Quality | 218 | 246 | 247 | 243 | 243 |
| RTT, mean (biased) | 12.80 | 13.82 | 14.43 | 14.62 | 15.28 |

- **It ranges.** Every point reads the true distance plus the usual 1.5–2.0 m (DEF-6), tight,
  and the RTT rises with it. The same two pucks were pinned at 6.3–6.7 m on the chairs an hour
  earlier.
- **Flipping the floor puck** moved the reading by 0.16 m, against 1.5 m on the chairs at 6 m
  the night before: with the node above, the tag's side up hardly matters.
- **What it means.** Two flat pucks at one height, edge-on, are the worst geometry and not the
  product's. A node mounted high and flat over the floor, and a tag flat on the floor wherever
  it lands, is the product's, and it ranged at 2.3–3.8 m with the known offset. Untested tonight:
  a tag under furniture or against a wall, the offset's constancy across pairs and antenna
  paths (the calibration, DEF-6), and more than one node.

**Correction, the same night.** The "edge-on" explanation above is wrong: the chip antenna's
pattern (Johanson 2450AT18D0100, evaluation-board plots) is within a few dB in all three planes.
What fits is two-ray ground reflection. With both pucks 0.45 m up, the floor bounce is nearly the
direct path's length (13 cm longer at 3 m, about one wavelength, changing by 0.2 rad across the
74 MHz band), and the reflection inverts it, so at some distances the direct path sits in a deep
fade across the whole band and the longest-lived bounce (ceiling, wall, furniture) sets the
reading, whose length barely follows the separation; at others the two reinforce and the reading
is true plus the offset. That is the on-and-off pattern of both nights' low rows, and flipping a
puck (its polarization against the floor) moving a point by 1.5 m. With one end at 2.3 m the floor
bounce at the tag merges with the direct path and nothing else is as short. To check: raise one of
two low pucks to about 1 m, which should un-pin the chair row.

## After midnight: the chair sweep, 0.50 to 4.00 m in 25 cm steps

Same chairs, same pucks, same side up, 0.45 m up, 60 s a point, N005–N004. Phase slope median,
RTT mean (per-pair bias included), data in `captures/2026-11-03-home/night2/sweep-*`:

| True (m) | 0.50 | 0.75 | 1.00 | 1.25 | 1.50 | 1.75 | 2.00 | 2.25 | 2.50 | 2.75 | 3.00 | 3.25 | 3.50 | 3.75 | 4.00 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Phase slope | 2.21 | 1.95 | 2.39 | 2.95 | 3.17 | 3.79 | 4.05 | 4.20 | 4.17 | 4.70 | 6.62 | 7.42 | 7.86 | 6.98 | 7.22 |
| Over true | 1.7 | 1.2 | 1.4 | 1.7 | 1.7 | 2.0 | 2.0 | 2.0 | 1.7 | 2.0 | 3.6 | 4.2 | 4.4 | 3.2 | 3.2 |
| RTT | 11.7 | 11.7 | 12.4 | 12.3 | 13.0 | 13.7 | 13.7 | 13.7 | 13.7 | 13.9 | 17.4 | 16.1 | 17.3 | 16.8 | 16.6 |
| Procedures with a phase estimate | 415 | 414 | 436 | 450 | 428 | 445 | 447 | 442 | 431 | 433 | 450 | 329 | 98 | 425 | 428 |

- **Clean to 2.75 m:** true + 1.2–2.0 m by phase, and the RTT rose 2.2 m over 2.25 m of
  true distance. Every point tight.
- **A step at 3.00 m** of +1.9 m by phase and +3.5 m by RTT for a 25 cm move, and from there
  to 4.00 m both sit 3.2–4.4 m over true. The same 3 m edge as the two earlier rows tonight and
  last night's (0.35 m up: pinned 3–5 m, back to true + 2.2 at 6 m).
- **At 3.50 m only 98 of about 430 procedures produced a phase estimate**, the rest having too
  few usable tones; the RTT, which needs only the packets, kept reporting. The deepest point of
  the fade thins the tones out, a sign the device could use.
- **The two-ray reading.** At 0.45 m the floor bounce is 2h²/d longer than the direct path:
  one wavelength at 3.3 m, where the inverted reflection cancels the direct path, and the
  ratio then falls slowly (0.8 λ at 4 m, 0.65 at 5, 0.55 at 6), so the fade is wide and
  recovers toward 6 m; the second-order fade near 1.65 m is narrow and shallow because at that
  range the direct path outweighs any room bounce by far more. That is the shape of the row:
  clean to 2.75 m, a wide fade from 3 m, and (last night) recovery by 6 m. The floor's reflection
  phase, polarization and what the pucks sit on move the edges by tens of centimetres.

## After midnight: the same chairs with the pucks on edge

The TAG's two chip antennas (Johanson 2450AT18D0100, ceramic monopoles, E-field along the
chip) sit 120° apart on the round board, so on edge each is about 60° from the horizon: three
quarters vertical by power, for both antennas and all four paths. Same chairs, same height, same
pair, 60 s a point, a coarse sweep with the fade filled in (`night2/vertical-*`):

| True (m) | 0.50 | 1.00 | 2.00 | 2.50 | 3.00 | 3.50 | 4.00 |
|---|---|---|---|---|---|---|---|
| On edge: phase slope | 1.79 | 2.35 | 3.58 | 4.37 | 5.41 | 6.71 | 6.61 |
| On edge: over true | 1.3 | 1.35 | 1.6 | 1.9 | 2.4 | 3.2 | 2.6 |
| On edge: RTT | 11.7 | 12.1 | 12.8 | 13.5 | 14.9 | 14.8 | 14.7 |
| Flat: phase slope | 2.21 | 2.39 | 4.05 | 4.17 | 6.62 | 7.86 | 7.22 |
| Flat: over true | 1.7 | 1.4 | 2.0 | 1.7 | 3.6 | 4.4 | 3.2 |
| Flat: RTT | 11.7 | 12.4 | 13.7 | 13.7 | 17.4 | 17.3 | 16.6 |

- **Shallower, not gone.** On edge the fade's worst point is 3.2 m over true (flat: 4.4), the
  RTT's excess over 2 → 4 m is 1.9 m for 2 m of true distance (flat: 2.9), and no procedures
  lost their tones at 3.5 m (flat: three in four). A quarter of the power is still horizontally
  polarized and the floor still reflects the vertical part at these grazing angles, so the
  direct path still dips at 3–4 m, just less.
- **Below 2.5 m both sweeps read true + 1.3–2.0 m,** the edge-on one a little lower.
- Polarization is a real lever, worth a node variant with a properly vertical antenna, but it
  doesn't replace placement: a metre below the ceiling and unequal node heights do more.

## After midnight: raw tones over the air, and the four antenna paths

To look inside the fade without cables, the initiator now sends one antenna path's raw tones a
second through the sink (`Message::Tones`, three chunks of 33 channels; commit 46cabaf), and
`navgrid tones` runs the host estimators on a monitor log: the phase slope, the first-path
estimator and the delay profile's peaks (what an IFFT shows), per procedure and antenna path.
Four 2-minute captures on the chairs, N005–N004 (`night2/tones-*`):

| Placement | Device range | Host phase slope | Strongest peak | Per path, coherent medians | Shortest coherent path |
|---|---|---|---|---|---|
| 3.0 m on edge | 5.14 | 5.05 | 5.0 | 5.07 / 5.94 / 4.95 / 4.76 | 4.79 |
| 3.5 m on edge | 6.76 | 6.88 | 5.4 (p10 3.45) | 7.04 / 6.85 / 6.89 / 6.62 | 6.63 |
| 3.5 m flat | 6.47 (p90 19) | 6.81 | 7.05 (p10 5.05) | 6.83 / **5.10** / 6.88 / 5.75 | **5.13** |
| 3.0 m flat | 4.45 | 4.51 | 4.4 | 4.49 / 3.88 / 4.77 / 4.23 | 4.21 |

(True + the usual offset: 4.5 at 3.0 m, 5.0 at 3.5 m.)

- **The host reads the device's tones the same way the device does** (phase slopes within
  0.1–0.3 m), so the pipeline is sound and the captures are the real thing.
- **At 3.5 m flat one antenna path still saw the direct path.** Path 1 read 5.10 m with
  coherence 0.98 while the other three read 6.8–9.2 m, and the residual-weighted blend gave
  6.47. The four paths do not fade together: the two chips sit 120° apart and a few centimetres
  from each other, enough to change the floor reflection's phase. A bounce is never shorter
  than the direct path, so **the shortest coherent path** is the better combiner: 5.13 here,
  4.79 at 3.0 m on edge, no worse elsewhere. It is now the device's and the host tools' default
  (`Combine::ShortestCoherent`, coherence ≥ 0.95, commit 770ae73).
- **No hidden direct path where all four paths read long.** At 3.5 m on edge the profile is one
  merged peak per path with nothing earlier above a tenth; the first-path estimator's
  earlier "components" there have strengths of 0.01, noise. Super-resolution has nothing to
  separate when the direct pair itself has faded; it would matter where a weak direct path
  survives in front of a stronger bounce, which these captures don't show.
- **A ladder at 3.5 m on edge.** On one path the profile shows peaks at 3.65, 7.15 and 10.5 m,
  equally spaced, strongest first: the first rung at the true distance with no offset. Two
  real paths 3.5 m apart would give 3.65, 5.4 (their cross term in the squared round-trip
  response) and 7.15, not 10.5; a harmonic ladder suggests something periodic in the phase
  (quantization, or the inline-PCT application) and is unexplained. Worth a look with the
  PCT values themselves, which the captures hold.
- **3.0 m flat was clean this time** (4.45 against 6.62 in the sweep an hour earlier): the fade's
  edge moves by tens of centimetres with the exact spot and the pucks' pose.
- The telemetry budget is tight with tones on: the outbox keeps 4 slots free for them and
  drops ranges beyond 12 queued, and the RTT range is queued before the phase one, so phase
  ranges are the ones dropped first (N001 kept 32 of ~500 one minute). To rebalance.

## Morning after: the estimators on the raw tones

**Question.** With raw tones in hand, does a super-resolution estimator (an IFFT or CIR view,
MUSIC, Prony-type) see a direct path where the phase slope reads long, and would it make a
better range for the proof of concept?

**Method.** `scripts/tones_estimators.py` (numpy, offline) rebuilds each (node, procedure,
antenna path) response from the `Tones` records of the four 2-minute captures
`night2/tones-{edge,flat}-{3.0,3.5}` (N005 initiating with N004 on the chairs; N001–N002 on the
desk a hand apart as the reference) and runs: the device's phase slope; the delay profile
(matched filter on a 5 cm grid, Hann window: what an IFFT shows); the matrix pencil (Hua and
Sarkar) and root-MUSIC with forward–backward smoothing, at model orders 2–5 and at an order taken
from the singular values (those above a tenth of the largest, 2 to 5). About ten of the 72
channels in every procedure carry a mode-1 RTT step instead of a tone, at random positions, so the
75-slot vector is completed first by Cadzow iterations (rank 4; ranks 2–10 give the same
components). A pencil or MUSIC component has a least-squares amplitude and a pole radius; it
counts as a path when its amplitude is at least a quarter of the strongest (a quarter to 0.4 makes
no difference), its pole radius is within 0.1 of 1, and it lies beyond 1.2 m (nothing can read
below the device offset). "Shortest" is the shortest such component, "strongest" the strongest;
medians over the ~27 procedures per antenna path. A synthetic self-test (paths at 5 and 9 m,
amplitudes 1 and 0.5, ten channels missing) returns 5.00 and 9.00 from both estimators where the
Hann profile shows one peak at 5.1. `scripts/tones_components.py` prints the amplitude-weighted
histogram of pencil components per path: a super-resolved profile over the two minutes.

**Results.** Expected = true + the pair's offset (1.45 m from the clean 3.0 m flat point): 4.5 at
3.0 m, 5.0 at 3.5 m. Medians per antenna path 0 / 1 / 2 / 3:

| Capture | Expected | Phase slope | Profile, strongest | Pencil (auto order), shortest | Pencil, strongest | MUSIC (order 2), shortest |
|---|---|---|---|---|---|---|
| 3.0 m on edge | 4.5 | 5.07 / 5.94 / 4.97 / 4.77 | 4.95 / 6.10 / 5.10 / 4.85 | 4.52 / 5.55 / 4.13 / 4.35 | 4.60 / 7.23 / 4.15 / 4.35 | 4.94 / 5.26 / 4.88 / 4.66 |
| 3.5 m on edge | 5.0 | 7.03 / 6.82 / 6.89 / 6.58 | 3.50 / 5.90 / 3.80 / 6.60 | 4.38 / 4.47 / 4.20 / 3.85 | 4.38 / 4.47 / 4.30 / 3.97 | 3.97 / 5.61 / 4.33 / 4.23 |
| 3.0 m flat | 4.5 | 4.50 / 3.91 / 4.77 / 4.23 | 4.75 / 3.35 / 4.60 / 4.00 | 5.22 / 3.20 / 4.58 / 3.96 | 5.37 / 5.77 / 9.58 / 7.06 | 4.46 / 3.66 / 4.27 / 4.08 |
| 3.5 m flat | 5.0 | 6.85 / 5.12 / 6.87 / 5.81 | 7.00 / 5.10 / 7.15 / 9.20 | 4.95 / 4.72 / 6.73 / 6.20 | 6.93 / 7.69 / 6.76 / 6.20 | 6.96 / 4.95 / 6.96 / 5.97 |
| Desk pair (N001), in the first capture | offset | 1.78 / 1.60 / 1.40 / 1.33 | 1.90 / 1.65 / 1.35 / 1.30 | 1.75 / 1.62 / 1.38 / 1.34 | same | 1.76 / 1.62 / 1.37 / 1.32 |

Shortest over the four paths (what the device's combiner would pick): slope 4.77 / 6.58 / 3.91 /
5.12 against the pencil's 4.13 / 3.85 / 3.20 / 4.72, expected 4.5 / 5.0 / 4.5 / 5.0.

Singular values of the completed Hankel matrix, medians per path after the largest: 3.0 m on edge
0.11–0.18 then ≤ 0.10 (one component and a weak second); 3.5 m on edge 0.57 / 0.35 / 0.17 on
path 0, 0.49 / 0.25 / 0.13 on path 2, 0.42 / 0.25 / 0.14 on path 3, then a floor at 0.03–0.05
(four components); 3.5 m flat 0.68 / 0.25 / 0.20 on path 0 and 0.69 / 0.15 / 0.12 on path 1
(three or four); the desk pair 0.09 then 0.02 (one).

What the super-resolved profiles show:

- **3.5 m on edge, every path:** components at 3.9–4.6 m (the strongest on paths 0 and 2),
  6.4–7.0 m, 9.9–12 m, and a weak one at 0.8–1.0 m on paths 0 and 2 (a fifth of the strongest,
  present at every completion rank). The profile's "ladder" at 3.45 / 7.3 / 10.6 on path 0 is
  these three components, 2.5–3 m apart, interfering inside a Hann main lobe 4 m wide: not
  harmonic, not periodic, nothing in the PCT values. (The middle one sits at the mean of the
  outer two, where the squared round-trip response puts their cross term, but the amplitudes do
  not fit a pure square.)
- **3.5 m flat:** paths 0 and 1 hold the direct path at 4.75–5.0 m at a third to a half of the
  strongest component (6.9 and 7.6 m); paths 2 and 3 have nothing before 6.0–6.5 m. The extra
  2.0–2.6 m fits the ceiling: with the pucks 0.45 m up under a 2.7 m ceiling the ceiling bounce
  at 3.5 m is 2.2 m longer than the direct path (2.4 m at 3.0 m).
- **The floor bounce is 13 cm longer than the direct path** and no estimator at 75 MHz of
  bandwidth separates it. "Direct" above means the direct-plus-floor pair; in the fade its two
  near-equal phasors sum to a phase that is no longer linear in frequency, so the pair's apparent
  delay moves. That is why the pencil's component sits at 4.0–4.6 m rather than 5.0 on edge, and
  why the slope on path 1 at 3.0 m flat reads 3.9 (components at 3.9 and 4.6).
- **Errors.** In the fades the pencil's shortest path is 0.3–1.2 m short where the slope is
  1.6–2.0 m long (3.5 m on edge), and reads 4.95 / 4.72 on the two 3.5 m flat paths that keep a
  direct component (slope 6.85 / 5.12) but 6.2–6.7 on the other two. Outside the fade (3.0 m
  flat) it is no better than the slope and far noisier: p10–p90 of 1–2 m per path against
  0.3 m. Taking the shortest over the four paths halves the worst error (3.85 against 6.58 at
  3.5 m on edge) but is short everywhere: the minimum of noisy values picks the noise.
- **Root-MUSIC** matches the pencil at order 2 and is noisier at every higher order (wider
  spreads, more outliers); it finds nothing the pencil does not.
- **The offset is a delay, not a slope artefact.** The desk pair reads 1.3–1.9 m by every
  estimator, each path the same across estimators; the spread across paths (and between the two
  captures 40 min apart, pucks untouched) is itself 0.5 m and belongs to the calibration (DEF-6).

**Conclusion.** Super-resolution sees inside the fade: at 3–3.5 m on the chairs the response is
three or four components (a weakened direct-plus-floor pair, a ceiling bounce about 2.2 m longer,
something at 10–12 m), which the slope averages and the windowed profile smears, and the matrix
pencil at the singular-value order recovers the direct component on all paths at 3.5 m on edge
and on two of four at 3.5 m flat, with 0.3–1.2 m of error against the slope's 1.6–2 m. It is not
a better range for the proof of concept: it needs a model order per procedure, an amplitude
threshold and a pole-radius filter, it is 1–2 m noisy procedure to procedure where the slope is
0.3 m, and the fade's distortion of the direct-plus-floor pair is below the resolution of any
estimator at this bandwidth. The shortest-coherent slope over antenna paths plus geometry (a node
high, unequal heights) stays the plan; the pencil is worth keeping on the host as a diagnostic of
what is in the channel, not on the device. The harmonic ladder is explained. The scripts are
offline analysis in Python next to the captures; a Rust port into `navgrid tones` is a todo if the
diagnostic earns its keep.

