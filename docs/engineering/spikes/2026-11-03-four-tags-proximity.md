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
