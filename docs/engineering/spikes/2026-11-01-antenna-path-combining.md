# Spike: ranging quality metric and antenna-path combining

**Question.** How do we tell a bad CS range from a good one? With 2–4 antenna paths per link, how
should the per-path estimates be combined?

**Answer.**

- **Quality:** use the RMS residual of the phase-slope fit. Adjacent-channel coherence cannot see
  multipath.
- **Combining:** diversity helps a lot, but no single combiner wins everywhere. Default to
  residual-weighted; decide on real captures in P1.

## Method

- **Channel model** (`navgrid_ranging::sim`): a direct path plus an optional reflector, over the
  72 CS channels. Each side gets a random LO phase and Gaussian per-tone phase noise (default
  0.05 rad), and the product cancels the LO phases.
- **Antenna paths:** each path sees the reflector with its own extra length (uniform within one
  wavelength) and strength (0.18–0.6).
- **Estimator:** phase slope, then the chosen combiner, then the placeholder σ model
  `0.05 + 2·residual`.
- **Geometry:** 2D fix (`--height fixed`) against 6 nodes in a 10 × 8 m room at mixed heights,
  with random targets.

## Reproduce

```sh
cargo run --release -p navgrid -- sim --multipath 3                       # single run, per-node table
cargo run --release -p navgrid -- sim --trials 500 --height fixed --multipath 3 --antenna-paths 4
# vary --multipath 1 / 3 / 6 and --antenna-paths 1 / 2 / 4
```

## Results

**Quality metric.** Reflection 3 m longer than the direct path at amplitude 0.6, against a clean
10 m link (unit test `multipath_shows_in_residual_not_coherence`):

| | Distance | Coherence | Fit residual |
|---|---|---|---|
| Clean | 10.00 m | 1.0000 | < 0.01 rad |
| Reflection | 11.79 m (+1.8 m) | 0.9998 | > 0.1 rad (sim run: 0.65–0.79 rad) |

**Combiners.** 2D fix p90 over 500 trials; "1 path" is the first path only:

| Excess path | 1 path | 4 paths, mean | 4 paths, lowest residual | 4 paths, residual-weighted |
|---|---|---|---|---|
| none | — | 0.006 m | 0.012 m | 0.006 m |
| 1 m | 0.36 m | **0.20 m** | 0.33 m | 0.20 m |
| 3 m | 0.88 m | **0.51 m** | 0.84 m | 0.60 m |
| 6 m | 1.96 m | 1.36 m | **0.59 m** | 0.82 m |

With 2 paths at 3 m excess: first 0.92, lowest-residual 0.92, mean 0.71, residual-weighted 0.76 m.

**Equal biases.** When all six ranges carry the same +1.8 m bias, a free 3D solve hides it in z
and puts the target 2.9 m below the floor. The residual flags every one of those ranges.

## Conclusion

- **Diversity pays:** ~40 % lower p90 at 1–3 m excess path.
- **Mean wins for short reflections.** Their bias flips sign with phase, so it averages out.
- **Selection wins for long reflections.** Their ripple is visible in the residual, so the clean
  path stands out.
- **Default:** residual-weighted is never the worst, so it stays the default (`Combine::default()`).
- **Caveats:** a single reflector, random phase per path, and a guessed σ model. Real indoor
  channels have many reflections and sometimes no direct path, so treat these as directional only.
- **Follow-ups:**
  - first-path IFFT / super-resolution estimator;
  - calibrating σ from real captures;
  - choosing the combiner on P1 captures.
