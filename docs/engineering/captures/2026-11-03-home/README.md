# Raw CS captures at home, N001–N002 cabled on a desk, 2026-11-03 evening

Firmware `0.1.0+0f39a7d3` built with `NAVGRID_LOG_STEPS`; 60 s of each board's RTT log per run
(gzipped), taken with `scripts/cs-capture.sh`. `-1ant` runs had both TAGs provisioned
`--no-diversity` (one antenna each, one antenna path); the others ran 2 × 2 antennas (four
paths). Distances are between the TAGs by tape; the sink's reports ran alongside.

| Run | True | Phase slope (device's method, host) | First path (`navgrid cs`) |
|---|---|---|---|
| 125cm | 1.25 m | 4.34 m; per antenna path 9.5 / 4.2 / 8.6 / 3.2 | 4.35 m (median over paths); paths 1 and 3: 3.4 and 3.1 |
| 80cm | 0.80 m | 4.50 m; per path 4.9 / 3.6 / 7.8 / 5.9 | 2.56 m, strength 0.22 |
| 80cm-1ant | 0.80 m | 4.79 m, p10 4.48, p90 5.22 | 2.33 m, p10 1.95, p90 2.76, strength 0.13 |
| 40cm-1ant | 0.40 m | 2.43 m, p10 2.24, p90 2.85 | 2.80 m, p10 1.91, p90 3.10, strength 0.51 |
| old-1ant | 0.60 m, N002 on the desk edge, N001 on a chair 30 cm lower; **yesterday's firmware `78f97aa`** | 2.88 m, p10 2.83, p90 2.96 | 2.60 m |
| new-1ant | the same, today's firmware `0f39a7d3` | 2.84 m, p10 2.79, p90 2.94 | 2.60 m |

```sh
gunzip -k 80cm-1ant-*.log.gz
cargo run -p navgrid -- cs 80cm-1ant-initiator.log 80cm-1ant-reflector.log --json
```
