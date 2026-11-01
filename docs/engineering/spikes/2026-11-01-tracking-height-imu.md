# Spike: 3D height sources, single-range tracking and IMU gating

**Questions.**

1. Is 3D viable without a magnetometer or a node layout designed for height?
2. Can a tag track with one range per update?
3. What does IMU motion gating buy?

**Answers.**

1. 3D is viable with differential barometry; ranges alone are fragile.
2. Single-range tracking works at about 2 ranges/s while walking.
3. Gating saves about a third of the procedures at the same accuracy, provided the filter handles
   stop/start correctly.

## Method

- **Fixes:** `navgrid sim` with `--height free | baro | fixed`. `baro` is a height observation at
  the true z plus N(0, 0.5 m).
- **Tracking:** `navgrid track` simulates a 70 s walk at 1 m/s. That is two laps of a 6 × 4 m loop
  around a 30 s stop, at a 1.2 m carry height.
  - Ranges go round-robin over 6 nodes, one per update; the barometer updates once a second.
  - With IMU gating: zero-velocity updates while still, a 6-range settling burst after a stop,
    0.1 ranges/s while still, and motion onset re-opens velocity uncertainty.
  - Without IMU: constant rate, and the filter never learns the target stopped.
  - Errors are sampled every 0.5 s and pooled over 20 seeds.

## Reproduce

```sh
cargo run --release -p navgrid -- sim --trials 500 --multipath 3 --antenna-paths 4 --height free   # or baro, fixed
cargo run --release -p navgrid -- track
cargo run --release -p navgrid -- track --multipath 3 --antenna-paths 4
cargo run --release -p navgrid -- track --multipath 3
```

## Results

**Height modes.** 500 trials, 4 paths, residual-weighted, fix p90:

| Height | 3 m reflection | Clean |
|---|---|---|
| free (3D, ranges only) | 2.40 m | 0.020 m |
| baro (3D + barometer σ 0.5 m) | 0.98 m | 0.38 m |
| fixed (2D, horizontal) | 0.60 m | 0.006 m |

The clean barometric case is worse than free because the placeholder σ model overstates clean
range errors (~0.18 m against a real ~0.005 m), so the barometer wins the weighting. See DEF-7.

**Tracking.** 20 runs; horizontal p90 while moving / while still:

| Ranges/s moving | IMU | Procedures | Clean, 1 path | 3 m reflection, 4 paths | 3 m reflection, 1 path |
|---|---|---|---|---|---|
| 0.5 | no | 41 | 5.03 / 17.98 m | 15.29 / 18.80 m | 13.08 / 14.59 m |
| 0.5 | yes | 33 | 14.16 / 0.37 m | 15.18 / 1.28 m | 14.22 / 1.27 m |
| 1 | no | 76 | 1.29 / 0.43 m | 3.58 / 1.95 m | 7.65 / 8.87 m |
| 1 | yes | 54 | 1.46 / 0.46 m | 4.23 / 1.82 m | 10.07 / 3.30 m |
| 2 | no | 146 | 0.33 / 0.16 m | 1.32 / 1.01 m | 2.06 / 1.92 m |
| 2 | yes | **94** | 0.40 / 0.16 m | 1.37 / 0.88 m | 1.95 / 1.16 m |
| 4 | no | 286 | 0.21 / 0.07 m | 1.05 / 0.72 m | 1.34 / 1.05 m |
| 4 | yes | 174 | 0.23 / 0.23 m | 1.05 / 0.76 m | 1.32 / 1.09 m |

Vertical p90 is 0.3–0.6 m throughout; the barometer dominates it.

**How the first IMU result went wrong.** The first run made IMU gating look *worse* than no IMU:
2 Hz overall horizontal p90 0.82 m vs 0.27 m.

1. **First suspect: filter overconfidence after a stop.** Zero-velocity updates collapse the
   velocity covariance, so after walking resumes the 3σ gate could reject every range. I added
   `Tracker::motion_started` (re-open velocity uncertainty at motion onset) and recovery after 3
   consecutive rejections. Both are sensible robustness features, but **the numbers barely moved**
   (p90 0.82 m before and after), so this was not the cause.
2. **Actual cause: the policy and the comparison.** While still, the no-IMU run kept ranging at
   full rate and averaged down to centimetres. The IMU run dropped to the keep-alive rate and kept
   the walking-phase error (overall p50 0.31 m vs 0.07 m). Fixed with a 6-range settling burst at
   the moving rate right after a stop. Errors are now reported per phase (moving / still) so the
   comparison is fair.

## Conclusion

- **Barometry makes 3D viable** under multipath. Pure-range 3D needs varied node heights and
  clean ranges.
- **Single-range tracking** with a constant-velocity model needs **~2 ranges/s while walking**.
  At 1/s, p90 is ~1.3 m; at 0.5/s the filter loses track. Lower rates need dead reckoning
  (gyro + steps).
- **IMU gating cuts procedures by 36 % here** at the same accuracy. This walk is still 43 % of the
  time; asset tags are still nearly all the time.
- **Multipath dominates.** A better ranging estimator beats filter tuning.
- **Caveats:**
  - round-robin node choice (no GDOP selection);
  - perfect still/moving detection;
  - single-reflector channel;
  - placeholder σ model;
  - no side-by-side with full-fix tracking yet.
