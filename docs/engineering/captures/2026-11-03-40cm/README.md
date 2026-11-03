# Raw CS capture, N001–N002 at 40 cm, 2026-11-03

Both TAGs cabled to DKs on the desk, 40 cm apart by ruler, firmware `0.1.0+0f39a7d3` built with
`NAVGRID_LOG_STEPS`. 60 s of each board's RTT log (gzipped): 660 and 654 subevent results.

```sh
gunzip -k n001.log.gz n002.log.gz
cargo run -p navgrid -- cs n001.log n002.log --json
```

gives 149 procedures with a phase-slope estimate: median 1.96 m, p10 1.84, p90 2.09, for a true
0.40 m. The device reported 1.88 m over the same minute. The first-path estimator (`first_path_m`)
gives median 1.81 m, p10 1.25, p90 1.91, finding 3–4 round-trip components per antenna path with
the direct one the strongest. `monitor-stats.py` summarises a
`navgrid monitor` text capture per node (median, p10, p90, sigma, quality), as used in the
spike `2026-11-03-four-tags-proximity.md`.
