# navgrid — architecture

Status: **draft v0.4** (2026-11-01). Companion to [requirements.md](requirements.md). Evidence
behind the numbers here lives in [engineering/spikes](engineering/spikes/).

## 1. Overview

```
            pinned                              pinned
           ┌──────┐   mesh link (CS + data)    ┌──────┐
           │ N1   │━━━━━━━━━━━━━━━━━━━━━━━━━━━━│ N2   │
           └──┬───┘╲                          ╱└──┬───┘
              ┃     ╲        ┌──────┐        ╱    ┃
              ┃      ╲━━━━━━━│ N3   │━━━━━━━╱     ┃         mesh: rare, event-driven refresh,
              ┃              └──┬───┘             ┃         nodes solve own positions
           ┌──┴───┐             ┃              ┌──┴───┐
           │ N4   │━━━━━━━━━━━━━╋━━━━━━━━━━━━━━│ N5   │
           └──┬───┘             ┃              └──────┘
              ┃                 ┃      ·  ·  ·  tag links (CS): while moving, ~2 Hz
              ┃              ┌──┴───┐ ·        ┌─────┐
         USB  ┃              │  P   │ · · · · ·│  A  │  direct link for the final approach
      ┌───────┴──┐           └──────┘          └─────┘
      │ sink/host│           person tag         asset tag (keys)
      └──────────┘           (HUD wearer)       still most of the time
```

- **Mesh links** connect static nodes. Ranges change slowly, so they refresh rarely. The same
  connections carry queued telemetry toward sinks when they wake to range (D-6).
- **Tag links** are short-lived links between a moving tag and nearby nodes, one range at a time.
- **Advertisements** carry each node's position, uncertainty, capacity, battery and pressure
  (FR-M4), so a tag needs no directory service.
- **The person↔asset direct link** serves the final approach of a find (§10).

## 2. Roles on a link

| Link | BLE roles | CS roles | Who computes distance |
|---|---|---|---|
| node ↔ node | lower `NodeId` is central *(proposed)* | central initiates | initiator; it sends the result back over the link because both ends need it |
| tag → node | tag is central (D-3) | tag initiates | tag |
| person → asset | person tag is central | person tag initiates | person tag |

A device must be central on some links and peripheral on others at the same time, so the
controller runs **multirole**. Nordic's SoftDevice Controller (SDC) has enabled initiator and
reflector roles per central/peripheral since NCS v3.4.0.

## 3. Link lifecycle

```
connect ─▶ encrypt ─▶ CS caps exchange ─▶ CS create config ─▶ CS security start
   │       (D-7)      (cache per peer:      (mode, channels,      (needs encrypted
   │                   write_cached_*)      steps, role)           ACL)
   ▼
set procedure params ─▶ procedure enable ─▶ ┌──────────── repeat ───────────┐
                                            │ subevent results (local)       │
                                            │ + reflector step data (D-4)    │
                                            │ ─▶ estimator ─▶ link filter    │
                                            └────────────────────────────────┘
```

Capabilities and the FAE table are cached per peer to cut reconnect time. `bt-hci` has
`LeCsWriteCachedRemoteSupportedCapabilities` / `...FaeTable` for this.

### Battery-driven link policy (D-10)

Every device runs on a CR2032 (budget in §5): no radio stays busy without a reason.

- **Mesh links stay connected at low duty.** They use a long connection interval with peripheral
  latency, and speed up only for a ranging burst (connection-parameter update or LE subrating),
  then slow down again. Re-discovering a static neighbor every time would cost a scan, which is
  much more expensive. It is not yet verified that CS works across subrating changes; P0 compares
  that against parameter updates and against reconnect-per-survey.
- **Survey cadence is event-driven.** After commissioning, nodes re-range rarely (minutes to
  hours), plus whenever the ADXL367 reports the node was moved (FR-M5).
- **Tags wake on motion** (§8). Devices advertise at `NodeConfig::adv_interval_ms` (default 1 s,
  ≈ 3 % of a one-month budget). Asset tags stay connectable for the final approach (FR-F5).
- **Telemetry is delay-tolerant** (D-6): queued data rides the next ranging contact toward a sink.
  There is no always-on routing tree.

## 4. Ranging pipeline (`navgrid-ranging`)

1. **Collect.** Per mode-2 step, take the phase correction term (PCT) for each antenna path, on
   both sides, plus tone quality indicators. Per mode-1 step, take the RTT timestamps.
2. **Combine.** `H[k] = PCT_init[k] · PCT_refl[k]`. Multiplying cancels both local-oscillator
   phases and leaves the round-trip phase `−4π f d / c`.
3. **Estimate.**
   - **Phase slope** (implemented): angle of `Σ H[k+1]·conj(H[k])` over adjacent 1 MHz channel
     pairs, which needs no unwrapping and is unambiguous over 0–150 m. A weighted least-squares
     fit over the full band then refines it. **Quality = RMS residual of that fit.** Adjacent-pair
     coherence tracks noise but is blind to multipath: a reflection 3 m longer at 0.6 amplitude
     biases the range by +1.8 m while coherence stays at 0.9998. The fit residual catches it.
   - **RTT** (implemented, units to be verified on hardware): coarse, multipath-robust, and
     relay-attack-resistant.
   - **IFFT / super-resolution** (P0/P1): resolves the first path under multipath.
4. **Combine antenna paths** (`combine_paths`). Each antenna sees the multipath differently.
   Diversity cuts fix p90 by ~40 % under moderate multipath, but the best combiner depends on the
   delay spread. Residual-weighted is never the worst, so it is the default until real captures
   decide ([spike](engineering/spikes/2026-11-01-antenna-path-combining.md)).
5. **Fuse and filter.** Combine estimates per procedure into distance + σ. A per-link filter
   rejects outliers before the solver.

**Antenna paths per link.** CS sounds local antennas × remote antennas paths, at most 4: TAG ↔ TAG
gives 4, TAG ↔ single-antenna module gives 2. A device with diversity off configures its controller
with 1 antenna and 1 path, so every link with it uses one path (FR-R7). More paths make each step
longer, which costs airtime and energy; that is the reason to allow the opt-out.

All of this is `no_std` and runs bit-identically on host captures (FR-R5). P0 validates it against
Nordic's `cs_de` (C, CMSIS-DSP) on the same captures.

## 5. Scaling, airtime and energy

Each node has an airtime budget:

```
U_node = Σ_links  rate_link × T_procedure   ≤  U_max   (e.g. 50 %)
```

- **Mesh links** are cheap. Degree 6 × one procedure per 30 s × ~30 ms ≈ 0.6 %.
- **Tag links** dominate. One tag at 2 Hz ≈ 6 % of a node, so about 8 moving tags per node before
  saturation.

Both figures are back-of-envelope. CS procedure time depends on step count, channel repetitions
and antenna paths, and P0 must measure it.

| Constraint | Consequence |
|---|---|
| CS is scheduled relative to connection events. The SDC default central ACL event spacing is 90 ms with CS on. | A tag's per-link rate is limited. Measure how many links × Hz one central sustains. |
| A node serving many tags is peripheral to many centrals whose anchor points collide. | Expect dropped CS events under load. Measure; adapt rates; advertise capacity (FR-M4). |
| Connections cost RAM and SDC configuration (`cs_count`, `central_count`, `peripheral_count`). | Per-node budget of about 6 mesh + 8 tag links *(to validate)*. |

Crowds are the hard case: many tags near the same few nodes. Mitigations: tag-side node selection
weighted by advertised capacity and battery, adaptive per-tag rates, and fewer steps per procedure
when busy.

### CR2032 energy budget (estimate)

A CR2032 holds ≈ 225 mAh. **One month ≈ 310 µA average** (the goal); **one week ≈ 1.3 mA**
(acceptable). `E/day = Σ activities  count/day × energy each`.

The per-activity costs are **placeholders** until P0 measures them with a power profiler:

| Activity | Assumed cost |
|---|---|
| Idle floor (System ON, RTC, RAM retention, sensors asleep) | a few µA |
| One single-path CS procedure (~25 ms radio + CPU at ~5 mA), on **both** ends | ~125 µC |
| Advertising event (3 channels) | ~10 µC → 1 s interval ≈ 10 µA |
| Persistent mesh link, connection event every 4 s | ~2–3 µC → ≈ 0.6 µA per link |
| BMI270 gyro running | assume ~0.5 mA (to be measured) |
| BME688 T/H/P sample, once a minute | negligible; the gas heater is not |

Per role, against the one-month budget:

| Role | Estimate |
|---|---|
| **Asset tag** | Still: idle + advertising ≈ 15 µA, which is months. Carried: tracking only while moving. |
| **Person tag** | Moving: 2 single-range updates/s ≈ 250 µA, plus gyro ≈ 0.5 mA, so ≈ 0.75 mA. That allows ~8 h/day of motion for a month. |
| **Node** | Idle, advertising and ~6 mesh links ≈ 20 µA, plus ≈ 40 µA while a person walks nearby (round-robin over 6 nodes). Busy corridors drain first, so tags weigh advertised battery (FR-T1). |
| **Find session** | Direct person↔asset ranging at 4 Hz for a minute ≈ 30 mC on each side. Negligible. |

Energy is no longer the binding constraint at a one-month goal, but the no-waste rules (NFR-6)
stay. Motion gating is what keeps asset tags near zero.

## 6. Rust stack — verified status (2026-11-01)

Investigation and sources: [spike](engineering/spikes/2026-11-01-rust-cs-stack.md).

| Layer | Crate / revision | Channel Sounding status |
|---|---|---|
| Controller | Nordic SDC (nrfxlib from NCS v3.4.0, Core 6.3) via `nrf-sdc` git | `Builder::support_channel_sounding_*` enables the roles. All 16 `sdc_hci_cmd_le_cs_*` C functions are in the bindings. **They are not yet wired to `bt-hci` commands**, and there is no builder setter for `SDC_CFG_TYPE_CS_COUNT` / `CS_CFG`. |
| HCI types | `bt-hci` 0.11.0 (2026-10-08) | All LE CS commands (opcodes 0x89–0x96) and events, including a step-data parser. `LeCsCreateConfigParams::reserved` sits where Core 6.3 has `cs_enhancements`, which is how to switch on inline PCT transfer. |
| Host | `TrouBLE` git | `Stack::command()` sends any HCI command. **CS LE events are logged as "Unknown LE event" and dropped.** No Ranging Service (RAS). |
| Estimation | — | Nothing in Rust; Nordic's `cs_de` is C. We write `navgrid-ranging`. |
| Antennas | SDC `sdc_cs_antenna_switch_callback_set` + `SDC_CFG_TYPE_CS_CFG` | Both reachable through `nrf_sdc::raw`. The node firmware sets the antenna count and path limit, and registers a GPIO callback for the TAG's SKY13348 switch. |
| HAL | `embassy-nrf` (pinned) | **No nRF54L option for DC/DC or crystal load capacitors.** The firmware writes the registers itself (`board.rs`, ported from Zephyr's nRF54L SoC init, using FICR trims). |

The crates.io releases (`trouble-host` 0.8, `nrf-sdc` 0.4) still depend on `bt-hci` 0.10, which
has no CS. That forces git dependencies (D-8).

Work needed:

- **P-1** `nrf-sdc`: bind the CS commands (one `sdc_cmd!` line each), add `cs_count()` /
  `cs_cfg()` setters and a safe antenna-switch callback setter. Mechanical; upstream PR.
- **P-2** `TrouBLE`: pass CS LE events to an `EventHandler` hook. Small; upstream PR.
- **P-3** navgrid: RAS client/server on TrouBLE GATT, and/or inline PCT transfer (D-4).
- **P-4** navgrid: distance estimation (`navgrid-ranging`).
- **P-5** `embassy-nrf`: nRF54L DC/DC and crystal load-capacitor config. Upstream PR; until then
  `board.rs` does it.

Until P-1 lands, firmware can call `nrf_sdc::raw::sdc_hci_cmd_le_cs_*` directly. The node skeleton
does this to read local CS capabilities at boot.

Reference fixtures (C, not shipped): NCS v3.4.1 `samples/bluetooth/channel_sounding/`
`{ras,ipt}_{initiator,reflector}` on the same boards.

## 7. Repository layout

```
crates/                     no_std, shared by firmware and host
  navgrid-proto/            wire messages (serde + postcard, COBS framing), NodeConfig
  navgrid-ranging/          CS channel response → distance; channel model for simulation
  navgrid-solver/           multilateration with height sources; tag tracker; mesh solver later
host/
  navgrid/                  CLI: sim, track, decode [--json]; later monitor, replay, central solver
firmware/                   separate cargo workspace, thumbv8m.main-none-eabihf
  node/                     single firmware image; role from config (D-2)
    src/board.rs            board-dk / board-tag: DC/DC, crystal caps, antenna switch (D-13)
docs/
  requirements.md, architecture.md
  engineering/              log, known defects, todo, spikes (D-21)
```

The host workspace (repository root) builds and tests the shared crates natively. `firmware/` is
its own workspace because of the embedded target and pinned git revisions; it uses the shared
crates through path dependencies.

## 8. Tag tracking (`navgrid-solver`)

**One-shot fix** (`multilaterate`) handles acquisition, settle fixes and reference solves. Its
`Height` option covers every height source:

- `Free`: ranges only. Needs nodes at varied heights.
- `Fixed(z)`: 2D.
- `Observed { z, σ }`: a barometric height or a loose carry-height prior, fused with the ranges.

**Tracker** (`tracker::Tracker`) is a constant-velocity Kalman filter over position and velocity
that takes **one measurement at a time**:

- a range to one node, with a 3σ outlier gate and recovery after 3 consecutive rejections;
- a height (differential barometry);
- a zero-velocity update (the IMU says the tag is still);
- motion onset (the IMU says the tag started moving), which re-opens velocity uncertainty.

One range per update means fewer simultaneous links and a quarter of the procedures that full fixes
need at the same update rate.

**Height.** Pure-range 3D is fragile under multipath (3D p90 2.4 m in simulation); barometry brings
it to ~1 m. Every TAG's BME688 measures pressure, so every node is a barometric reference. A tag
compares its own pressure with nearby nodes' (FR-M4 adverts), and weather drift cancels. Building
HVAC zones and sensor offsets are the risks (R-10): use nearby same-zone nodes only, and calibrate
per-node offsets against CS geometry.

**IMU.** The TAG has a BMI270 (accelerometer and gyro) and an ADXL367, but no magnetometer.

| Tier | Who | Sensors | What it does | Status |
|---|---|---|---|---|
| a | all tags | ADXL367 (+ BMI270 accel) | Wake-on-motion, still/moving state, motion gating with a settling burst, zero-velocity updates, motion-noise selection | In the tracker and simulator |
| b | person tags | + gyro, step detection | Heading for the HUD (§10); pedestrian dead reckoning between ranges | P1 |
| c | — | + gyro, full inertial | Inertial propagation with a heading state | Only if it pays off |

Simulation ([spike](engineering/spikes/2026-11-01-tracking-height-imu.md)) shows:

- motion gating saves ~36 % of procedures at the same accuracy, on a walk that is still 43 % of
  the time;
- the constant-velocity model needs about 2 ranges/s while walking;
- multipath dominates the error, so a better ranging estimator is worth more than filter tuning;
- a settling burst after a stop is required.

## 9. Monitoring and sideband (FR-S)

```
 TAG sensors ──▶ telemetry queue ──(rides ranging contacts, D-6)──▶ sink (USB-powered TAG/DK)
 (health, BME688,                                                        │ USB, COBS frames
  link stats, positions)                                                 ▼
                                   host `navgrid monitor` ──▶ terminal view + JSON-lines log
                                            └──▶ asset registry (§10) and central solver (FR-M6)
```

Monitoring is **our own small engineering tool** (D-16), not an external stack.

- **Messages** (`navgrid-proto`): `Health` (battery, die temperature, firmware, queue depth),
  `Environment` (BME688: temperature, humidity, pressure at 0.01 Pa, optional gas),
  `LinkStats` (procedures, failures, distance, quality, RSSI), plus `Range` and `Position`.
  All fit in 64-byte frames.
- **The sink** is the one device without a battery limit. It can scan continuously and accept
  connections whenever devices have queued data.
- **Timestamps without clock sync.** Records carry the origin's uptime, and each forwarding hop
  adds its residence time, so the sink computes the record's age and timestamps it on arrival.
  The envelope format is still to be designed.
- **One ingest serves three consumers:** the monitor view, the asset registry, and the central
  reference solver all read the same records.
- **Today:** `navgrid decode --json` turns a capture into JSON lines. `navgrid monitor` (live
  terminal view plus log) comes with the sink firmware.

## 10. Find my keys

```
asset tag (keys)                        person tag (HUD wearer)                  host
────────────────                        ───────────────────────                  ────
still: advertise 1 s, near-zero cost
moved: tracker (tier a)
settled: burst → fix → Position ──(contacts)──▶ sink ──────────────────────▶ asset registry
                                                                             (last fix, σ, age)
                                        pose: tracker + gyro heading (tier b)        │
                                        pointer = bearing(pose → asset) ◀───────────┘ (Q13)
                                        └─▶ HUD (host view until integrated)
within radio range:
  reflector ◀────── direct CS ─────── initiator: distance + synthetic-aperture bearing
  LED blink on request
```

- **Asset registry** (D-18). The host keeps the last settled fix of every asset tag (position, σ,
  age) from `Position` records. A mesh-local query is a later option (Q14).
- **Person heading** (D-19). The gyro integrates yaw; the absolute offset is corrected whenever
  the person walks straight and steadily, by comparing the tracker's velocity direction with the
  gyro yaw. At 1 m/s with ~0.1 m/s velocity error that is ~6° per alignment. Between alignments,
  gyro drift over minutes is small. A spike will test whether CS phase differences between the
  person tag's two antennas give bearing to nodes directly (R-12).
- **Pointer** (FR-F3): bearing and elevation from the person's pose to the asset, plus distance.
  Error ≈ heading error + (person σ ⊕ asset σ) / distance, which is why it degrades close in
  (R-13).
- **Final approach** (D-20). When the asset is within radio range, the person tag ranges to it
  directly (person initiates; the asset is connectable per FR-F5). CS gives distance to
  decimetres. Bearing comes from the direct ranges collected along the person's tracked path:
  `multilaterate` with the person's past positions as anchors (synthetic aperture). This needs
  ~1 m of movement. The HUD shows distance first and bearing once it converges.
- **HUD interface** is open (Q13): what device, whether it has its own IMU, where the person tag
  is worn, and how the pointer reaches it. Until then the host view renders the pointer.

## 11. Decision log

| ID | Decision | Status |
|---|---|---|
| D-1 | Rust everywhere: Embassy + `nrf-sdc` + `TrouBLE` + `bt-hci` on firmware; std Rust on host. | **Accepted** (user requirement; feasibility checked §6) |
| D-2 | One firmware image; role (node / asset / person / sink), pinned coordinates and periods from provisioned `NodeConfig`. | Proposed |
| D-3 | On tag links the tag is central + initiator and nodes are peripheral + reflector. This gives self-positioning, nodes stay unaware of tags, and per-tag compute lands on the tag. | Proposed |
| D-4 | Reflector → initiator data: **inline PCT transfer (IPT) first.** All devices are ours, and it is faster (~30 ms vs 100–200 ms per estimate), has no GATT hop, and costs less energy per range. Its limits are mode 2 only and weaker relay-attack resistance. **RAS** comes when phones do, or as the fallback if IPT fails the R-8 checks. | Proposed |
| D-5 | Wire format: postcard + COBS; types in `navgrid-proto`. | Proposed |
| D-6 | Telemetry is delay-tolerant: queued data rides ranging contacts toward sinks over L2CAP CoC. No always-on routing tree and no second radio protocol. | Proposed |
| D-7 | Every link uses LTKs derived from the fleet network key and starts encryption directly, with no pairing. Phones, if they come, use LE Secure Connections. | Proposed (verify against SDC/TrouBLE) |
| D-8 | Firmware pins the TrouBLE / `nrf-sdc` / Embassy git revisions from TrouBLE's nRF54 example, and moves to our forks once P-1/P-2 exist. | **Accepted** (forced by `bt-hci` 0.11) |
| D-9 | Solvers: tags use a Gauss-Newton fix to start, then the tracker (D-14). Mesh uses distributed weighted least squares with neighbor covariances, bootstrapped by a central solve on the host. | Proposed |
| D-10 | Battery link policy (§3): low-duty persistent mesh links that burst to range, event-driven re-survey, wake-on-motion tags, configurable advertising. | Proposed (P0 measures alternatives) |
| D-11 | Antenna diversity: all paths the link allows; per-device opt-out via `NodeConfig::diversity`; per-path estimates combined residual-weighted by default (§4). | **Accepted** (user requirement); combiner open until P1 |
| D-12 | **3D with height sources** (CS geometry, differential barometry, carry-height prior) via `Height::Observed`; 2D is `Height::Fixed`. | **Accepted** |
| D-13 | Board selected at build time (`board-dk` / `board-tag` features, `cargo tag` / `cargo dk`). Role and diversity come from runtime `NodeConfig`. | Proposed |
| D-14 | Tag tracker: constant-velocity Kalman filter with one-measurement-at-a-time updates (range, height, zero-velocity), outlier gate with recovery, motion-onset re-opening (§8). | Proposed |
| D-15 | IMU tiers (§8): accelerometer gating on all tags; gyro heading on person tags; full inertial only if it pays off. | **Accepted** (revised v0.4: people need heading) |
| D-16 | Monitoring is our own small tool: telemetry rides ranging contacts to a USB-powered sink; `navgrid monitor` shows mesh state and logs JSON lines (§9). | **Accepted** |
| D-17 | CR2032 no-waste rules (requirements NFR-6), against a 1-month goal / 1-week floor. | **Accepted** |
| D-18 | Asset tags publish a fix when they settle; the host keeps the asset registry (§10). | Proposed |
| D-19 | Person heading from gyro integration, aligned to the CS-tracked walking direction; a spike tests antenna-path phase bearing (§10). | Proposed |
| D-20 | Final approach on direct person↔asset CS: distance direct, bearing by synthetic aperture over the person's track (§10). | Proposed |
| D-21 | Engineering records live in `docs/engineering/`: log, known defects, todo, spikes. Design stays in requirements/architecture; evidence goes in spikes. | **Accepted** (repo rule) |
