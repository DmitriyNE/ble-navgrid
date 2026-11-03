# 2026-11-03, late night: the chairs and the high node

`navgrid monitor --json` range records (`Range` only) from N005 (initiator, with N004) and N001
(initiator, with N002, on the desk a hand apart all along, the reference), one file per placement,
60 s or more each. Spike: `spikes/2026-11-03-four-tags-proximity.md`, last two sections.

- `chairs-<d>.jsonl.gz`: N005 and N004 flat on wooden chair seats, 0.45 m up, same side up,
  `d` metres apart. The two `-phase-only` files are from the firmware before the RTT referee.
- `high-<x>.jsonl.gz`: N004 flat on top of a wardrobe at 2.30 m, N005 flat on the floor `x` metres
  sideways from directly below it; `high-3-flipped` is the 3 m point with N005 the other side up.

Ranges with method `Rtt` are the initiator's mean mode-1 ToA−ToD over the procedure, reported
100 m up (DEF-55): subtract 100 m.
