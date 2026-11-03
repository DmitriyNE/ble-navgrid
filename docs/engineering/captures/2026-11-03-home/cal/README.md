# N005–N004 on batteries, flat pucks 0.35 m above the floor, 2026-11-03 night

`navgrid monitor --json`, 60 s per point, the on-device range (2 × 2 antennas, 4 paths, 70
steps) as N005 reported it through the sink. Pucks on a box and a chair in a room with a 4 m
ceiling and metal HVAC at 3.4 m, nothing else within 2 m. The pucks had opposite sides up until
the last point, when N005 was flipped to match N004.

| File | True | Median | p10–p90 | Quality |
|---|---|---|---|---|
| n005-1.0 | 1.0 m | 5.92 | 5.43–6.32 | 246 |
| n005-1.5 | 1.5 m | 3.56 | 3.20–4.23 | 230 |
| n005-2.0 | 2.0 m | 3.33 | 3.26–3.42 | 245 |
| n005-3.0 | 3.0 m | 7.52 | 7.14–8.33 | 238 |
| n005-4.0 | 4.0 m | 7.58 | 6.47–8.29 | 237 |
| n005-5.0 | 5.0 m | 7.17 | 7.01–7.50 | 237 |
| n005-2.0b | 2.0 m again | 3.58 | 3.42–3.89 | 240 |
| n005-6.0 | 6.0 m | 8.25 | 8.10–8.56 | 242 |
| n005-6.0-flipped | 6.0 m, same side up | 9.77 | 9.53–10.58 | 243 |
