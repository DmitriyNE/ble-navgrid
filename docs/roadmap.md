# navgrid roadmap

Status: 2026-11-04, after the chair and estimator sessions. The phases are the agreed P0–P3
([requirements §3](requirements.md)); this is the order of work inside them and the test that
closes each milestone, given what the bench has shown. The work items themselves live in
[the todo](engineering/todo.md); the evidence in [the spikes](engineering/spikes/).

## Where we stand

- **The stack is Rust end to end and ranges:** CS mode 2 with 2×2 antenna diversity and inline
  PCT transfer, a mode-1 round-trip-time referee, raw tones to the host, over-the-air updates,
  provisioning and the mesh-state crate, diagnostics through a sink. Four TAG pucks and one DK
  sink on the bench; 20 TAGs in the box.
- **The physics is understood well enough to design around:**
  - each antenna path carries a fixed offset of 1.3–1.9 m, a real delay seen by every estimator,
    so calibration is per path (DEF-6);
  - two boards at one low height fall into the floor reflection's fade from about 3 m and read a
    bounce, by phase and by RTT alike; no estimator at 75 MHz of bandwidth separates a bounce
    13 cm longer than the direct path, so the fix is geometry, not signal processing;
  - a node 2.3 m up over a tag on the floor reads true plus the offset from 2.3 to 3.8 m, face up
    or down. Nodes go high, about a metre below the ceiling, at unequal heights; tags lie wherever
    they land (C-5);
  - super-resolution (matrix pencil) shows what is in the channel and stays a host diagnostic.
- **Not yet measured:** energy per activity on a CR2032, concurrent-link ranging rates, the
  per-path offsets on a rig, and anything with more than one node.

## Milestones

### M0 — a range you can trust (closes P0)

Goal: a calibrated range in the product geometry, with its error known, from a tag anywhere a key
lands.

1. **Calibration rig and per-path offsets.** A node 2.3 m up, a tag on the floor, tape points from
   1 to 6 m; `navgrid calibrate` keyed by antenna path; the offset applied on the device. Exit:
   line-of-sight p90 ≤ 0.5 m from 1 to 6 m (NFR-1 at short range).
2. **Where keys land.** Under a table, against a wall, in a bag and a pocket, on a shelf, 2–5 m
   sideways from the node; then two nodes at once. Exit: a table of error by situation, and a
   decision on which the mesh covers and which the final approach must cover.
3. **Several links.** One tag ranging to two or three nodes with CS scheduled across the links
   (mesh stage 3), rates measured per link count. Exit: at least two ranges a second to a moving
   tag over three links (NFR-2).
4. **Energy.** A power profiler on a TAG: advertising, an idle link, CS per antenna-path count,
   wake-on-motion; CR2032 sag during a procedure (R-9). Exit: a budget for the asset tag and the
   person tag against NFR-6 (a month as the goal, a week as the floor).
5. **Housekeeping the above needs:** the outbox balance with tones on, the tones rate and the
   combiner as provisioning settings, the sub-mode 1 abort rate, the kept captures re-run with
   `--ipt`, the `cs_de` cross-check that P0's exit names.

Not in M0: self-survey, the upstream PRs (a parallel track).

### M1 — find my keys at home (P1, first cut)

Goal: four nodes hung in two rooms, and the host pointer walks you to the keys.

- **Hardware:** pendants — TAGs on edge in a holder, hung about a metre below the ceiling at
  unequal depths (0.7–1.3 m), tape-surveyed and north-aligned (pins, FR-A3a). One asset tag, one
  person tag, the DK sink.
- **Firmware:** the asset tag's motion gating (ADXL367 wake-on-motion, a settling burst, publish
  on settle: FR-T5, FR-T6); the person tag at its moving rate; tags choose nodes by beacon and
  hand over between them (FR-M4); link records with range, quality and antenna path.
- **Host:** the central solver over link records publishes position records (mesh stage 5, host
  first): robust multilateration with the per-path offsets, bounce rejection by residual, 3D with
  a carry-height prior; the pointer view as the HUD stand-in (bearing and distance from the
  person to the asset), in the monitor or the thin web UI (D-23). The final approach: direct
  person↔asset ranging (FR-F4), distance shown, bearing from how it changes as the person moves,
  the asset's LED blinking.
- **Exit, on a test course at home:** asset position p90 ≤ 1 m horizontal; pointer bearing
  ≤ 20° p90 beyond 3 m; final-approach distance ≤ 0.3 m p90 (re-baselined if the per-path offsets
  do not hold unit to unit); diversity on against off and gating on against off, measured.

### M2 — nodes that survey themselves (P2)

Goal: the product's node exists and the mesh solves its own positions.

- **Hardware:** a node variant with a vertical antenna (or the TAG on edge in a housing), a
  ceiling mount about a metre down, CR2032 per decision Q1 (nodes range rarely). Node-to-node
  ranging under a ceiling at two depths is tested before the mount is fixed, since the ceiling
  bounce seen on the chairs (2.2 m past the direct path) sits above the pendants too.
- **Mesh:** node-to-node ranging on low-duty links (D-10); the central solve (FR-M6) and then the
  distributed one (FR-M2); commissioning presets (FR-A3); re-survey of a moved node (FR-M5);
  sync over the links (stage 4: sessions, controller commands, the monitor's mesh view).
- **Operations:** the field tool and install mode, identify, registry operations; update resume
  and retry, canary rollouts.
- **Exit:** node p90 ≤ 0.5 m against tape; the distributed solution within 0.2 m of the central
  one; frame yaw ≤ 5°; a moved node re-surveyed without an operator.

### M3 — a building (P3)

Many tags with admission control and an airtime budget (NFR-4), handover across rooms, routing to
several sinks, delta sync, the fleet key at rest with APPROTECT, updates across hops, the HUD's
heading in place of the host view, and a multi-room deployment with capacity measured.

### Parallel tracks

Upstream PRs (bt-hci, nrf-sdc, TrouBLE, embassy-nrf), CI with host tests and firmware clippy,
the probe-rs report, simulator scenarios (pointer-error statistics, GDOP-aware node selection), and
the matrix pencil ported into `navgrid tones` as a diagnostic.

## Risks that set the schedule

- **Energy (R-7, R-9).** If CS on a CR2032 cannot give a person tag a week, the person tag goes
  rechargeable (revisiting Q1). Measured in M0, before the firmware is tuned around it.
- **Offsets per path and per unit (DEF-6).** If offsets drift with temperature or battery, or
  differ unit to unit by more than 0.3 m, every unit is calibrated at enrolment; the rig is built
  so that is cheap.
- **Link capacity (R-2).** The controller's concurrent CS links and the per-link spacing (DEF-26)
  cap ranges per second; if three links cannot give two a second, the tracker leans on the
  accelerometer (FR-T7) and lower rates.
- **Multipath beyond the floor (R-4).** The hedge is geometry: unequal node heights and the
  shortest coherent antenna path. Tested under the ceiling in M1 before anything is mounted.
- **Sub-mode 1 and the RTT referee.** Optional; dropped if it costs ranges.

## Pace

At the bench's current rate (bring-up to raw tones over the air in four days) M0 is about a week of
bench time, M1 two to three weeks (the solver, gating and handover are the bulk), M2 is set by the
node hardware's lead time rather than by firmware, and M3 is open.
