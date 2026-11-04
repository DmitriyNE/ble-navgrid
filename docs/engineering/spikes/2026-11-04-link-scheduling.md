# Link scheduling: several links per node, in the simulator

**Question.** Mesh stage 3 gives every node several links with Channel Sounding across them. Which
neighbours, who connects to whom, what duty each link runs at, how bursts are scheduled, and what
ranging rate that leaves a moving tag and a surveying anchor, before any of it is measured on
hardware (roadmap S1: the measurements come in S5, so the quantities are settings).

**Method.** The policy is the `navgrid-link` crate (no_std, shared with the firmware): neighbours
from beacons, the lower anchor connects, admission by the beacon's free links, one burst at a time
per node, surveys on a period and on motion, a moving tag rotating bursts over its links.
`navgrid link-sim` runs it over a radio timeline: anchors on a jittered grid, tags that walk for
2 min and rest for 3, beacons heard in scan windows with a log-distance path loss (exponent 3,
3 dB of noise), links at the slow interval that burst to the fast one, and CS procedures as
subevents right after the link's connection events with the timing of spike 2026-11-02 (four
paths: two 22 ms subevents in consecutive events, a procedure every three; one path: one 11 ms
subevent every two). A procedure is **empty** when another link's connection event (the SDC's
reserved 2.5 ms, counted on the central end at every event and on a peripheral with latency `l`
on one event in `l + 1`) or another procedure overlaps a subevent on either end. The model has no
loss, no connection drops, and instant parameter updates apart from the `--switch-ms` delay.

Reproduce: `cargo run --release -p navgrid -- link-sim --anchors 8 --tags 2 --hours 1 [--json]`
with the flags of the tables.

**Results.** 8 anchors 6 m apart, 2 tags, 1 h; anchors at a 1 s slow interval with latency 4,
degree 6, 10-procedure surveys every 15 min; tags' links at the fast interval times their link
count, events spaced by the fast interval, 4-procedure bursts; moving 40 % of the time.

Moving tags (ranges per second while moving, distinct anchors ranged per 5 s, empty procedures,
seconds from setting off to the first range, p50 / p90):

| Tag links × interval, paths | Ranges/s | Anchors per 5 s | Empty | First range |
|---|---|---|---|---|
| 1 × 30 ms, 4 paths | 6.9 | 0.9 | 31 % | 1.0 / long tail |
| 2 × 60 ms, 4 paths | 4.0 | 1.7 | 30 % | 1.0 / 1.2 |
| **3 × 90 ms, 4 paths** | **1.8** | **1.8** | **54 %** | **1.0 / 2.0** |
| 3 × 90 ms, 4 paths, events not spaced | 1.6 | 1.5 | 60 % | 1.1 / 3.0 |
| 3 × 90 ms, 4 paths, anchors' slow interval 2 s | 2.6 | 2.3 | 33 % | 0.9 / 1.2 |
| 2 × 30 ms, 1 path | 11.0 | 1.8 | 17 % | 1.0 / 1.2 |
| 3 × 45 ms, 1 path | 8.2 | 2.4 | 27 % | 0.9 / 1.1 |
| 20 anchors, 5 tags: 3 × 90 ms, 4 paths | 2.4 | 2.2 | 39 % | 0.9 / 2.1 |
| 20 anchors, 5 tags: 3 × 45 ms, 1 path | 8.0 | 2.4 | 27 % | 0.9 / 1.3 |
| 20 anchors, 10 tags: 3 × 90 ms, 4 paths | 2.1 | 2.0 | 45 % | 0.9 / 1.9 |

Earlier runs of the same model settled two design points: tags on slow links that switch to the
fast interval per burst by a connection parameter update (six slow intervals to the instant)
got 0.26 ranges/s and their first range 7 s after setting off (13.5 s p90), against 3.75/s with a
200 ms switch; and three 30 ms links on one tag, 4 paths, came back 100 % empty however they were
spaced, since two 24.5 ms subevent slots don't fit in 30 ms. Spacing three 90 ms links greedily
from the previous one (DEF-26's way) left the third 22.5 ms from both neighbours, two in three
procedures empty; spacing them by the fast interval, planned for the count, fits.

Anchor surveys (8 anchors, no tags, 1 h):

| Slow interval / latency / degree | Links | Bursts/h | Empty | Events/s |
|---|---|---|---|---|
| 0.5 s / 4 / 6 | 5.7 | 11.5 | 51 % | 12.1 |
| **1 s / 4 / 6** | **5.7** | **11.5** | **38 %** | **7.1** |
| 2 s / 4 / 6 | 5.7 | 11.5 | 32 % | 5.5 |
| 1 s / 0 / 6 | 5.7 | 11.5 | 56 % | 7.1 |
| 1 s / 4 / 4 | 3.7 | 7.5 | 29 % | 4.6 |
| 1 s / 4 / 6, events blocking 1 ms instead of 2.5 | 5.7 | 11.5 | 37 % | 7.1 |

With 2–10 moving tags around, the anchors' surveys lose 50–73 % of their procedures, the tags'
fast links adding events at the anchor end.

**Conclusion.**

- **Tags connect at the fast interval and never switch.** A parameter update per burst is out for
  a moving tag; anchors, whose links burst every 15 min, switch by parameter update (six seconds
  each way, a few percent of the time fast). Subrating can replace the update later (D-10).
- **A tag's links share the radio:** interval = fast interval × links, each link's events one
  fast interval after the previous, planned for the full count. The firmware passes the k-th
  link's spacing to the SDC's central event spacing when it connects.
- **Three links at 90 ms with four paths give a moving tag about two ranges a second over two
  anchors**, with half the procedures empty in this model; one path at 45 ms gives eight, but
  loses the antenna diversity the chairs showed we need (the shortest-coherent path). Four paths
  stays the default; the one-path mode is a setting for a dedicated link.
- **The anchors' slow-link events dominate the empties on both kinds of link.** A 2 s slow
  interval takes a third off; latency matters as much. The model counts every reserved event
  on both ends and is pessimistic: the bench saw 2–3 % empties from one 1 s link (DEF-50) where
  the model gives 4.9 %. Survey bursts of 12 procedures leave enough either way.
- **The dedicated sink link goes:** telemetry rides whichever mesh link reaches a sink.
- **For the bench (S5):** the empties per link count (how the SDC arbitrates connection events
  against CS subevents), CS across a parameter update, the SDC's central spacing applied as the
  k-th link, admission by the beacon's free links, and the tag's first-range time.
