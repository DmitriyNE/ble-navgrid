# CS antenna diversity (2×2) on the TAG pair

**Question.** Do two antennas per TAG work for CS? Does combining the four antenna paths help?

## Setup

- **Boards:** the TAG pair, about 90 cm apart on a desk, each on its own DK. The fork is pinned
  at `6381257` (SDC v3.4.1, RRAM standby), with inline PCT transfer on.
- **Tone mapping:** `navgrid_ranging::steps` maps tones to antenna paths with the Core 6.0
  permutation tables (Vol 6, Part H, Tables 4.13–4.15, as in Zephyr's
  `bt_le_cs_get_antenna_path`).
- **Antenna config:** the initiator picks the tone antenna configuration from both antenna
  counts: index 7 (A2:B2, 4 paths) between two TAGs. The preferred peer antenna mask is `0b11`.
- **Estimate:** on-device phase slope per path, then the residual-weighted combination. The host
  `navgrid cs --ipt --json` gives the per-path values.

## Results

Two captures, 206 and 167 procedures. Every subevent reported 4 paths; each procedure took 2
subevents.

| Path | Capture 1: median (p10–p90) | Residual | Capture 2: median (p10–p90) | Residual |
|---|---|---|---|---|
| 0 | 3.76 m (3.65–3.84) | 0.34 rad | 3.75 m (3.65–3.85) | 0.33 rad |
| 1 | 3.84 m (3.73–3.97) | 0.40 rad | 3.83 m (3.73–3.98) | 0.40 rad |
| 2 | 3.38 m (3.27–3.50) | 0.40 rad | 3.30 m (3.21–3.38) | 0.39 rad |
| 3 | 5.42 m (4.25–7.21) | 1.36 rad | 7.05 m (4.71–7.35) | 1.32 rad |
| **Combined** (residual-weighted) | **3.72 m (3.63–3.87)** | | **3.72 m (3.62–3.87)** | |

Other combiners on capture 1:

| Combiner | Median (p10–p90) |
|---|---|
| First path only | 3.76 m (3.65–3.85) |
| Lowest residual | 3.75 m (3.43–3.84) |
| Mean | 4.12 m (3.79–4.59) |

The single-path IPT run at the same placement gave 3.68 m (3.63–3.76), residual 0.18 rad.

## Conclusion

- **Diversity works end to end:** the antenna switch callback, the permutation mapping, and
  per-path estimates on-device and on the host.
- **Paths 0–2 agree within about 0.5 m.** Path 3 is consistently bad here, with a residual of about
  1.3 rad and readings of 5–7 m. That looks like strong multipath on that antenna pairing at this
  placement, not a mapping error: the other paths stay clean (DEF-25).
- **No gain over one path yet.** The residual-weighted combination stays sound because path 3
  gets little weight. Which combiner is most accurate can't be told without ground truth, and one
  desk placement says little about multipath diversity.
