# CS ranging rate on the TAG pair

**Question.** How fast can the initiator range with inline PCT transfer, and what does the rate
cost in spread? The starting point was a 50 ms connection with a procedure every 10 events:
2 estimates/s.

## Setup

- **Boards:** the TAG pair, about 90 cm apart, each on its own DK. Firmware `f739cf0` plus timing
  changes, with inline PCT transfer.
- **Estimate:** on-device phase slope, combined over the antenna paths by residual.
- **Logging:** raw step logs off (they outrun 100 kHz SWD at these rates); rates from the
  on-device `distance` log lines.
- **One path:** a temporary build with diversity off on both TAGs.

## Results

| Connection | Paths | Procedure: interval / max length (events) / subevent | Rate | Median | p10–p90 | Empty |
|---|---|---|---|---|---|---|
| 50 ms | 4 | 10 / 9 / 16 ms (before) | 2.0 Hz | 3.72 m | 3.62–3.87 m | 0 |
| 15 ms | 4 | 6 / 5 / 11 ms | 5.6 Hz | 3.69 m | 3.59–3.93 m | every other procedure |
| 15 ms | 4 | 8 / 7 / 11 ms | 8.3 Hz | 3.91 m | 3.71–4.30 m | 0 |
| 15 ms | 1 | 2 / 1 / 11 ms | 33.3 Hz | 3.33 m | 3.24–3.38 m | 0 |
| **30 ms** | **4** | **3 / 2 / 22 ms** | **11.1 Hz** | **3.60 m** | **3.51–3.66 m** | 0 |
| 30 ms | 1 | 1 (granted 2) / 1 / 11 ms | 16.7 Hz | 3.59 m | 3.51–3.65 m | 0 |

- **The SDC always placed CS events two connection events apart** ("event interval 2"), so a
  procedure takes at least two events.
- **Empty procedures:** with four paths and 11 ms subevents, a procedure needed four subevents.
  The 6-event interval then collided with the next procedure, which came back empty.
- **Subevent count:** more subevents per procedure widened the spread (8 / 7 / 11 ms). Two 22 ms
  subevents were as tight as one path.

## Conclusion

- **Adopted:** a 30 ms connection, with timing by path count. One path: every 2 events, 11 ms
  subevents. Two paths: every 2 events, 22 ms. Four paths: every 3 events, at most 2, 22 ms
  subevents. The CS event length is 22 ms.
- **Result:** 11.1 estimates/s with four paths and 16.7 with one, both with a p10–p90 spread of
  about 0.15 m.
- **One-path rate:** one path could reach 33 Hz on a 15 ms connection. That needs a connection
  update after the capability exchange, since the path count is known only then.
- **The median moved with the configuration** (3.33–3.91 m at one placement), so a calibration has
  to be keyed by timing configuration as well as by mode.

## One path, after the 2.5 ms ACL events

Firmware `1c72aa1` (2.5 ms ACL events, scheduled uplink) with diversity off. The initiator asked
for a procedure every connection event. The SDC granted every 2 ("event interval 2"): 16.7/s, none
empty, with the uplink up. Shorter ACL events don't lift the two-event minimum.

33/s needs a 15 ms connection. There, a one-path CS event fills about 14 ms of every other 15 ms
frame, and the free frames' parity is not under our control. That leaves no dependable room for a
second link on the node: an uplink or another ranging link. A 15 ms connection suits a dedicated
one-to-one link only.
