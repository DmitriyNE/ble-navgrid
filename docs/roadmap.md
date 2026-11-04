# navgrid roadmap

Status: 2026-11-04. The phases are the agreed P0–P3 ([requirements §3](requirements.md)); this is
the order of work across them and the test that closes each stage. The work items themselves live
in [the todo](engineering/todo.md); the evidence in [the spikes](engineering/spikes/).

**The ordering principle: code is cheap, bench time and hardware are not.** The software is
finished end to end first (mesh, self-survey, distributed solving, command and control, the web
interface, operations at scale), validated on the simulator and on a desk mesh of spare TAGs where
the ranges are rough but the protocol is real. The precise calibration, the measurement campaign
and the custom hardware come once, at the end, with the software ready to ingest and show their
results. Wherever a design choice would wait on a measurement, the software takes the quantity as
a setting with today's best guess, so the campaign tunes numbers rather than changing structure.

## Where we stand

- **The stack is Rust end to end and ranges:** CS mode 2 with 2×2 antenna diversity and inline
  PCT transfer, a mode-1 round-trip-time referee, raw tones to the host, over-the-air updates,
  provisioning and the mesh-state crate with its simulator, diagnostics through a sink. Four TAG
  pucks and one DK sink on the bench; 20 TAGs in the box.
- **The physics is understood well enough to design around:** each antenna path carries a fixed
  offset of 1.3–1.9 m (a real delay, calibrated per path later; the device holds a per-path offset
  table, zero until then); two boards at one low height fall into the floor reflection's fade from
  about 3 m, which no estimator at this bandwidth undoes, so nodes go high at unequal heights and
  the solver rejects bounces by residual; a node 2.3 m up over a floor tag ranges correctly.
  Super-resolution is a host diagnostic, not a range.
- **Settings standing in for measurements until the campaign:** ranging rate per link count and
  the per-link CS spacing (DEF-26), every duty period (NFR-6 demands they are configurable
  anyway), the per-path offsets, and the range σ model (DEF-7).

## Stages

### S1 — the mesh talks (mesh stages 3 and 4)

Several links per node with CS scheduled across them, low-duty links that burst to range (D-10);
sync sessions over the links with a byte budget; the sink mirrors its replica to the host and
takes desired records back; neighbour discovery and hop counts from beacons; the controller key
and a MAC on desired state; the first commands (blink, sensor periods, diversity) and the
monitor's mesh-state view.

Bench: about 8 TAGs on the desk as nodes, 2–3 as tags, the DK sink. Exit: commands and records
cross partitions and several hops on the desk mesh; latency and bytes per contact match the
simulator.

### S2 — the mesh knows where everything is (mesh stage 5, FR-M, FR-T, FR-A)

The host's central solver over link records publishes position records (FR-M6): robust
multilateration with the per-path offsets, bounce rejection by residual, 3D with the carry-height
prior and barometric height. Then distributed on-node solving (FR-M2) validated against the host,
link down-weighting and re-survey of moved nodes (FR-M5), absolute referencing from commissioning
presets (FR-A3, `fit_pins` / `fit_bearings`, the frame version on every position). Tags: the
on-device tracker, motion gating with a settling burst (FR-T6), publish on settle (FR-T5), node
choice by beacon with handover (FR-M4).

Exit: in the simulator and on the desk mesh, the distributed solution within 0.2 m of the central
one; a moved node re-surveyed without an operator; a tag's position published within a minute of
settling; frame handedness and yaw right against the presets.

### S3 — command and control, and the web interface (FR-C, FR-F, FR-S6, FR-H)

The host becomes a service: the global replica, the asset registry (FR-F1), desired-state commands
with reported state, the registry operations (place, replace, retire), identify and button events.
The thin web UI (D-23): floor plan with nodes and tags, mesh links and health, sideband, pin and
preset placement, the asset registry, and the find-my-keys flow as the HUD stand-in: pointer and
distance from the person to the asset, the final approach on direct mobile↔asset ranging (FR-F4)
with the bearing refined from motion and the asset's LED blinking.

Exit: the whole find-my-keys flow runs end to end on the desk mesh (a pendant hung from the
ceiling if it helps the demo), from "where are my keys" to the blink, through the service and the
web page only.

### S4 — operations at scale (FR-O, NFR-4)

Admission control and the airtime budget, handover across rooms, several sinks, delta sync
(DEF-28), compact record layout (DEF-29), updates across hops with resume, retry and canary
rollouts, the field tool and install mode, the fleet key at rest with APPROTECT (DEF-33), retiring
nodes. Energy accounting per activity in the firmware as counters, so the campaign reads them off.

Exit: 500 nodes and 100 tags in the simulator inside NFR-4's per-node bounds; a rollout and a
retire across the desk mesh without touching a device.

### S5 — the measurement campaign (closes P0 and P1 properly)

One campaign, with everything above in place to ingest and display it:

1. **Calibration** per antenna path and per unit on a rig: a node 2.3 m up, a tag on the floor,
   tape points 1–6 m, `navgrid calibrate` through the service; offsets written to the devices.
   Exit: line-of-sight p90 ≤ 0.5 m from 1 to 6 m; the unit-to-unit spread known (if over 0.3 m,
   calibration becomes an enrolment step).
2. **Energy and sag** with a power profiler against the firmware's own counters; CR2032 sag during
   a procedure (R-9). Exit: the asset and mobile tag budgets against NFR-6; if a mobile tag cannot
   reach a week, it goes rechargeable (Q1 revisited).
3. **Link capacity:** ranges per second per link count, measured (R-2, DEF-26); the scheduler's
   settings tuned.
4. **Where keys land** and **nodes under a ceiling at two depths** (the ceiling bounce seen on the
   chairs sits above pendants too): error by situation; what the mesh covers and what the final
   approach must.
5. **The P1 test course** at home with four pendants: position p90 ≤ 1 m, pointer ≤ 20° beyond
   3 m, final-approach distance ≤ 0.3 m p90, diversity and gating each on and off. The NFRs are
   re-baselined from these numbers (Q3).

### S6 — custom hardware and the building (P2 hardware, P3)

The node variant (vertical antenna, ceiling mount about a metre down, power per Q1), the
single-antenna asset module, the mobile tag; node-to-node under the ceiling checked before the
mount is fixed; then a multi-room deployment with capacity measured and the HUD's heading in place
of the host view.

Exit: P3's, with node p90 ≤ 0.5 m against tape and NFR-4 met in the building.

### Parallel tracks

Upstream PRs (bt-hci, nrf-sdc, TrouBLE, embassy-nrf), CI with host tests, firmware clippy and
replay of recorded captures (NFR-9), the probe-rs report, simulator scenarios (pointer-error
statistics, GDOP-aware node selection, link loss and reboots), the matrix pencil in `navgrid
tones` as a diagnostic, and the housekeeping the bench left (outbox balance with tones on, the
tones rate and combiner as settings, sub-mode 1 aborts, the `cs_de` cross-check).

## Risks the ordering carries

- **Building on guessed numbers.** Rates, periods, offsets and σ are settings; the campaign may
  move them a lot, but not the structure, as long as nothing hard-codes them (the simulator runs
  the same code with the same settings).
- **The desk mesh validates protocol, not accuracy.** Positions on the desk are only consistent,
  not right; the solver's accuracy claims wait for S5.
- **Energy could still force a design change** (a rechargeable mobile tag, or coarser gating); the
  firmware keeps every activity switchable so that is a settings change too.
