# navgrid — architecture

Status: **draft v0.7** (2026-11-02). Companion to [requirements.md](requirements.md). Evidence
behind the numbers here lives in [engineering/spikes](engineering/spikes/).
v0.7: operations: enrolment, commissioning, identification, the field tool, LED and button
(§13). v0.6: the mesh keeps a convergent shared state that carries routing, positions, sideband
and commands, with the host as controller (§12).

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
  (FR-M4), so a tag needs no directory service. The layout is `navgrid_proto::Beacon`: 17 bytes
  of manufacturer-specific data, with the name in the scan response.
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

**Encryption (D-7).** The link key is AES-CMAC under the fleet key of a label and both device
addresses, the lower first, so both ends derive it. A central adds it to TrouBLE as a non-bonded
key before connecting, and a peripheral as soon as it accepts. TrouBLE then encrypts without
pairing and forgets the key when the link drops.
- **Outsiders:** a peer with another key fails on the first encrypted packet (MIC failure). A
  provisioned node also drops a peer that pairs instead.
- **Unprovisioned devices** pair Just Works, for bring-up.

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
   phases and leaves the round-trip phase `−4π f d / c`. With inline PCT transfer (D-4, on by
   default) the reflector applies its phase to the returned tone, so `H[k] = PCT_init[k]` and the
   initiator ranges alone.
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
6. **Calibrate** (host, `navgrid calibrate`). Devices report raw distances. The host subtracts one
   constant offset per (initiator, reflector, antenna paths, method), fitted from ranges at
   measured distances; recalibrating needs no reflash. The path count stands in for the CS timing
   configuration, which moves the offset as well (spike 2026-11-02-cs-ranging-rate).

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
| **Person tag** | Moving: 2 single-range updates/s ≈ 250 µA, which fits a month of near-continuous motion. Heading is the HUD's job. Optional gyro dead reckoning would add ≈ 0.5 mA, giving ~8 h/day of motion for a month. |
| **Node** | Idle, advertising and ~6 mesh links ≈ 20 µA, plus ≈ 40 µA while a person walks nearby (round-robin over 6 nodes). Busy corridors drain first, so tags weigh advertised battery (FR-T1). |
| **Find session** | Direct person↔asset ranging at 4 Hz for a minute ≈ 30 mC on each side. Negligible. |

Energy is no longer the binding constraint at a one-month goal, but the no-waste rules (NFR-6)
stay. Motion gating is what keeps asset tags near zero.

## 6. Rust stack — verified status (2026-11-01)

Investigation and sources: [spike](engineering/spikes/2026-11-01-rust-cs-stack.md).

| Layer | Crate / revision | Channel Sounding status |
|---|---|---|
| Controller | Nordic SDC (nrfxlib from NCS v3.4.0, Core 6.3) via `nrf-sdc`, fork `DmitriyNE/nrf-sdc` | Upstream lacked CS command bindings and CS config setters. **Fixed in the fork (P-1):** 13 CS commands bound, layouts checked at compile time; `cs_count`, `cs_cfg`, `cs_antenna_switch_callback`. |
| HCI types | `bt-hci` 0.11, fork `DmitriyNE/bt-hci` | All LE CS commands (opcodes 0x89–0x96), events and LE event-mask bits 43–50. `LeCsCreateConfigParams::reserved` sits where Core 6.3 has `cs_enhancements`, which is how to switch on inline PCT transfer. Upstream parsed CS step data column-major while the SDC sends it row-major; **fixed in the fork (P-6).** |
| Host | `TrouBLE`, fork `GreatKet/trouble` | `Stack::command()` and `Stack::async_command()` send any HCI command. Upstream enabled no CS events and dropped any that arrived. **Fixed in the fork (P-2):** a `channel-sounding` feature with CS event-mask bits, `EventHandler` hooks and the CS host feature bit. The fork also fixes the connection-parameter-update response identifier. No RAS. |
| Estimation | — | Nothing in Rust; Nordic's `cs_de` is C. We write `navgrid-ranging`. |
| Antennas | SDC `sdc_cs_antenna_switch_callback_set` + `SDC_CFG_TYPE_CS_CFG` | Both reachable through `nrf_sdc::raw`. The node firmware sets the antenna count and path limit, and registers a GPIO callback for the TAG's SKY13348 switch. |
| HAL | `embassy-nrf` (pinned) | **No nRF54L option for DC/DC or crystal load capacitors.** The firmware writes the registers itself (`board.rs`, ported from Zephyr's nRF54L SoC init, using FICR trims). |

The crates.io releases (`trouble-host` 0.8, `nrf-sdc` 0.4) still depend on `bt-hci` 0.10, which
has no CS. The firmware therefore uses our forks at pinned revisions (D-8); upstream PRs are still
to be opened.

Work needed:

- **P-1 `nrf-sdc`** (mechanical, roughly 40 lines; gate like the existing CS builder flags):
  - **Bind the CS commands** in the LE command module. The byte layouts were checked field by
    field against the C structs; the 3-byte subevent lengths match as `ExtDuration`.
    - Sync with a return value: `LeCsReadLocalSupportedCapabilities`,
      `LeCsWriteCachedRemoteSupportedCapabilities`, `LeCsSetDefaultSettings`,
      `LeCsWriteCachedRemoteFaeTable`, `LeCsSetProcedureParams`.
    - Sync without one: `LeCsSetChannelClassification`.
    - Async (completion arrives as an LE event): `LeCsReadRemoteSupportedCapabilities`,
      `LeCsSecurityEnableCommand`, `LeCsReadRemoteFaeTable`, `LeCsCreateConfig`,
      `LeCsRemoveConfig`, `LeCsProcedureEnable`, `LeCsTestEnd`.
    - `LeCsTest` has a variable-length tail and needs a hand-written impl like `LeSetExtAdvData`.
      It is optional, but CS test mode is handy for RF bench checks without a connection.
  - **Builder setters:** `cs_count(n)` (`SDC_CFG_TYPE_CS_COUNT`) and
    `cs_cfg(max_antenna_paths, num_antennas)` (`SDC_CFG_TYPE_CS_CFG`).
  - **A safe antenna-switch callback setter** wrapping `sdc_cs_antenna_switch_callback_set`.
  - **No event work:** the SDC hands raw HCI events up and bt-hci parses them generically.
- **P-2 `TrouBLE`** (small):
  - **Event mask:** a `channel-sounding` cargo feature that adds the LE event-mask bits for the CS
    events (43–50).
  - **Event hook:** `EventHandler` methods for those events, in the style of the existing
    feature-gated CIS hooks: the remote-capabilities, FAE-table, security, config and
    procedure-enable completions, plus subevent result, subevent result continue and test end.
  - **Host feature bit:** set *Channel Sounding (Host Support)* (LE feature bit 47) through *LE Set
    Host Feature*, as Zephyr does when CS is enabled. This was found while writing the patch.
  - **Nothing else:** commands, async commands and encryption already exist. The hook runs in
    the runner task, so the app copies step data out and returns.
- **P-3** navgrid: inline PCT transfer first, RAS as fallback (D-4).
- **P-4** navgrid: distance estimation (`navgrid-ranging`).
- **P-5** `embassy-nrf`: nRF54L crystal load-capacitor config, and a time driver that clears
  stale GRTC interrupt state at init. Upstream PR; until then `board.rs` does both. (DC/DC needs
  nothing: `init` already enables it.)
- **P-6 `bt-hci`:**
  - **Fix the step parser:** parse CS step data row-major (mode, channel, length, data per step)
    for subevent result and subevent result continue, and replace the test data with the real
    layout.
  - **Rename `reserved` → `cs_enhancements`** in create-config and test params (Core 6.3).
    Cosmetic.
  - Confirm on hardware in P0 by dumping one raw subevent.

**How the patches ship:** fork all three and point the firmware at the forks. `bt-hci` goes in
through `[patch.crates-io]`, because TrouBLE and nrf-sdc both take it from crates.io. Then open
upstream PRs.

Until P-1 lands, firmware can call `nrf_sdc::raw::sdc_hci_cmd_le_cs_*` directly. The node skeleton
does this to read local CS capabilities at boot.

Reference fixtures (C, not shipped): NCS v3.4.1 `samples/bluetooth/channel_sounding/`
`{ras,ipt}_{initiator,reflector}` on the same boards.

## 7. Repository layout

```
crates/                     no_std, shared by firmware and host
  navgrid-proto/            wire messages (serde + postcard, COBS framing), NodeConfig
  navgrid-ranging/          CS channel response → distance; channel model for simulation
  navgrid-solver/           multilateration with height sources; tag tracker; frame + pointer;
                            mesh solver later
  navgrid-mesh/             mesh state: records, merge, classes, sync, outbox, routes (§12)
host/
  navgrid/                  CLI: sim, track, decode [--json], monitor, calibrate, cs, mesh-sim;
                            later replay, central solver, controller
firmware/                   separate cargo workspace, thumbv8m.main-none-eabihf
  node/                     single firmware image; role from provisioning (D-2)
    src/board.rs            board-dk / board-tag: DC/DC, crystal caps, antenna switch (D-13)
    src/store.rs            provisioning page and boot epoch log in the top 8 KiB of RRAM
    src/keys.rs             link keys from the fleet key (D-7)
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

**IMU.** The TAG has a BMI270 (accelerometer and gyro) and an ADXL367. Tags do not produce
heading: the HUD provides it (§10).

| Tier | Who | Sensors | What it does | Status |
|---|---|---|---|---|
| a | all tags | ADXL367 (+ BMI270 accel) | Wake-on-motion, still/moving state, motion gating with a settling burst, zero-velocity updates, motion-noise selection | In the tracker and simulator |
| b | optional | + gyro, step detection | Pedestrian dead reckoning between ranges, to lower the ranging rate while walking | Only if it saves more than the gyro costs |
| c | — | + gyro, full inertial | Inertial propagation | Only if it pays off |

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
  These messages become the Health, Environment and Link records of the shared state (§12),
  which also carries the age.
- **One ingest serves three consumers:** the monitor view, the asset registry, and the central
  reference solver all read the same records.
- **Today:** a DK sink forwards the frames it receives over L2CAP (SPSM `0x0081`) to its
  VCOM1, and `navgrid monitor` prints them as text or JSON lines; only the initiator's `Range`
  stream feeds it so far. `navgrid decode --json` does the same for a capture file. The live
  mesh-state view is still to come.
- **Later: a thin web UI** (FR-S6, D-23), served by the same host process from the same state.
  It shows the mesh on a floor plan, health and sideband, and the asset registry. Pin placement
  for referencing (§11) happens there too. It is a single static page polling a small JSON
  endpoint; no frontend build chain.

## 10. Find my keys

```
asset tag (keys)                        person tag (HUD wearer)                  host
────────────────                        ───────────────────────                  ────
still: advertise 1 s, near-zero cost
moved: tracker (tier a)
settled: burst → fix → Position ──(contacts)──▶ sink ──────────────────────▶ asset registry
                                                                             (last fix, σ, age)
                                        position: tracker → world frame (§11)        │
                                        heading: HUD's own sensor (world frame)      │
                                        pointer = bearing(person → asset) ◀─────────┘
                                        └─▶ HUD (host view stands in for now)
within radio range:
  reflector ◀────── direct CS ─────── initiator: distance + synthetic-aperture bearing
  LED blink on request
```

- **Asset registry** (D-18). The host keeps the last settled fix of every asset tag (position, σ,
  age) from `Position` records. Host only for now; a mesh-local query may come later (Q14).
- **Heading** (D-19) comes from another sensor, typically the HUD's magnetometer, already in the
  world frame. Our side guarantees that positions are in the same frame (§11).
- **Pointer** (FR-F3, `frame::pointer`): bearing relative to the HUD's heading, elevation, and
  distance, all in the world frame. Error ≈ HUD heading error + mesh yaw error (§11) +
  (person σ ⊕ asset σ) / distance, which is why it degrades close in (R-13).
- **Final approach** (D-20). When the asset is within radio range, the person tag ranges to it
  directly (person initiates; the asset is connectable per FR-F5). CS gives distance to
  decimetres. Bearing comes from the direct ranges collected along the person's tracked path:
  `multilaterate` with the person's past positions as anchors (synthetic aperture). This needs
  ~1 m of movement. The HUD shows distance first and bearing once it converges.
- **HUD interface** is not relevant yet (Q13). The host view renders the pointer.

## 11. Absolute referencing (FR-A, `navgrid_solver::frame`)

CS ranges fix the mesh's shape and scale. A self-surveyed solution is still free to rotate,
mirror and translate. **Commissioning presets on a few nodes** fix the rest (Q15):

| Unknown | Fixed by | Needed for pointing? |
|---|---|---|
| Up direction, vertical mirror | Barometric heights (already in the solver) | yes |
| Yaw | ≥ 2 pins, **or** ≥ 1 preset bearing | **yes**: its error adds 1:1 to bearing |
| Horizontal mirror (handedness) | ≥ 3 non-collinear pins, **or** ≥ 2 preset bearings in different directions | **yes** |
| Origin (x, y, z) | 1 pin | no; maps and display only |

**`Frame`**: mesh → world. Mirror y, rotate by yaw, translate. Headings are counter-clockwise from
east; `compass_to_heading` converts clockwise-from-north bearings.

**Pins (`fit_pins`)** are nodes with known absolute coordinates. The fit is a 2D least-squares
rotation (Kabsch, no scale), tried plain and mirrored, keeping the better. It reports the RMS
misfit, so a mis-surveyed pin shows. In P1 the nodes themselves are hand-surveyed in a
north-aligned frame, so the transform is identity.

**Preset bearings (`fit_bearings`)** give the known world bearing from one node to another, e.g.
read off a floor plan. The mesh knows that pair's direction in its own frame, so each preset is
one heading observation. Yaw is the circular mean of the world − mesh differences, weighted by
the squared baseline. Handedness is whichever choice makes the differences agree; it only shows
with ≥ 2 directions. `Frame::anchored` adds an origin from one pin.

- **Accuracy:** with 0.1 m mesh position error and 5–9 m baselines, three presets give yaw
  within 3° (unit test).
- **Quality checks:** the fit reports `concentration` (agreement) and `handedness_margin`, so
  presets that contradict each other show.
- **Precision:** a pair's direction error is roughly the mesh's relative position error divided
  by the baseline, so presets on long baselines are the accurate ones (R-12).

**Presets live in a host site file** (pins, bearings, magnetic declination). The host fits the
frame from them.

**Later: learned from headings (`fit_headings`).** The same math takes HUD heading observations
on straight walks instead of presets, to reference a mesh without commissioning. It also needs
observations from many places, because indoor magnetic disturbances are local.

**Versioning (FR-A5).** The host owns the current transform and its version. Positions carry the
version they were solved in. Re-estimation happens when presets change or the mesh re-solves.

## 12. Mesh state, routing and control (FR-C)

```
 node ──contact──▶ node ──contact──▶ sink ──USB──▶ host: controller
        records flow up the hop-count gradient       global replica, solver, view
        ◀──── desired state flows down the reverse path ────  commands
```

FR-C asks for consensus about the mesh state, routing to keep it, positions and the link graph
built on it, sideband and commands carried in it, and a controller that observes and steers it.
On a coin-cell mesh that partitions, "consensus" means **convergence**. Every replica that has seen the same writes holds the same state, without voting
(D-24).

### State model

The state is a set of **records** (`navgrid-mesh`). Each record has exactly one writer, its
origin, and a key `(origin, topic)`. Records fall into **classes**: one subject, one scope. A
replica keeps a class whole or not at all.

| Kind | Writer | Content | Class: kept by |
|---|---|---|---|
| Node | the node | role, config hash, firmware, frame version seen, hop count to a sink, neighbour list | Near: 2-hop neighbourhood, sinks |
| Health | the node | battery, die temperature, uptime, queue depth (FR-S2) | Up: sinks |
| Environment | the node | BME688 sample (FR-S3); pressure also serves FR-T2 | Near |
| Link | the link's initiator | peer, distance, σ, quality, antenna paths, procedures and failures, aggregated over a ranging burst | Near |
| Position | the controller until P2, then the node | position, covariance, frame version | About (controller's): the node's neighbourhood, sinks; Near (node's own) |
| Desired | the controller | one setting's config, or a one-shot action | Desired: the target and its peers, sinks |
| Reported | the node | the desired version it acted on, per setting, and status | Up |
| Frame | the controller | the versioned mesh→world transform (FR-A5) | Global: everyone |

- **Versions.** Each origin numbers its writes with a monotonic sequence: an epoch bumped at boot
  (persisted), then a counter. A replica keeps the record with the highest sequence. With one
  writer per key nothing conflicts, so merging is commutative, associative and idempotent:
  replicas converge in any order, through any partition.
- **Derived state** is either a deterministic function of records (the link graph is the set of
  Link records) or a single-writer record of its own (positions, the frame).
- **Age and expiry.** Records carry their age, which every holder advances by the time it held
  them (FR-S4, NFR-5), so any replica knows a record's age without a clock. Records expire by
  kind: node records after 2 h, links after 24 h, unless rewritten. The controller retiring a
  node is still to come.

### Scope: who holds what (D-25)

- **Every node keeps:**
  - its own records;
  - its 2-hop neighbourhood's node, link, environment and own-position records, and the
    controller's positions of them;
  - the frame;
  - desired state for itself and its peers.

  The 2-hop neighbourhood comes from its peers' node records. That is what cooperative
  localization (FR-M2) and tag node selection (FR-T1) need.
- **A node also holds:**
  - an **outbox** of records in its custody, on their way to sinks or to a target;
  - **routes:** for each origin whose records it carried, the peer they came from.
- **Sinks and the host:** everything. Several sinks merge at the host; merging doesn't depend on
  order.
- **Size, measured** ([spike](engineering/spikes/2026-11-02-mesh-state-sim.md)):
  - **Wire:** a record is 20–40 B.
  - **RAM:** a record is 136 B as the crate holds it today (DEF-29).
  - **A node:** peaks at 111 records with 20 nodes and 143 with 500, set by degree rather than
    mesh size. That is 13–17 KiB at p50, and up to ~34 KiB at the busiest relays at 500 nodes
    with 4 sinks. Routes reach about 75 there.
  - **A sink:** 173 records (26 KiB) at 20 nodes, which fits a DK. At 500 nodes, each of 4 sinks
    holds 145–185 KiB, more than a DK can spare next to the SDC, so the host holds the replica.

### Sync on contacts

- **When:** a session runs over each mesh link's L2CAP channel (SPSM `0x0082`) when the link is
  awake (D-10), never on a dedicated wake-up (FR-S4). The simulator's starting point is a
  session every 60 s.
- **How** (`navgrid_mesh::wire`):
  1. Hello: hop count and sink flag.
  2. A **summary** from each side: an order-free hash (the sum of per-record hashes of key and
     version) for every class it keeps. A sink keeps everything, so it answers the peer's
     summary class for class.
  3. For the classes both list with different hashes, the **versions** each holds.
  4. The **records** the other lacks or holds older.
  5. Then the **pushes** (below).
- **Budget:** 2 KB per side per session caps records and pushes. It was reached in 0.01 % of
  sessions at 500 nodes with 4 sinks.
- **Cost:** summaries are about 80 % of the bytes, roughly 650 B per side per session.
  Sending only the classes that changed since the last complete session with that peer is the
  obvious saving (DEF-28).
- **Wire format:** postcard + COBS frames as today (D-5), one record per frame, at most 240 B.

### Routing (D-26)

- **Hop counts:** a sink's hop count is 0, and a node's is one more than its lowest peer's. It is
  carried in its node record, its session Hello, and later its advertisement (FR-M4).
- **Up, toward sinks:** a node writes its records into its outbox and **pushes** them to any
  peer with a lower hop count, which takes custody. The sender drops a record once the peer
  accepts it; a full peer refuses, and the sender keeps it. This is gradient routing,
  store-and-forward and delay-tolerant (D-6).
  - **Compaction:** a newer version replaces an older one in the outbox but keeps its place in
    the queue.
  - **Order:** commands and reports go first, the rest oldest first, so bulk can't starve
    anything.
- **Down, toward a target:** a node remembers, for each origin whose records it took from
  uphill, the peer they came from. The controller's records for a node (desired state,
  positions) are pushed along those routes until they reach a node that keeps their class. The
  target's neighbourhood takes them from there by sync. The host hands them to the sinks that
  have a route to the node.
- **No routing tree** to maintain and no second radio protocol; Q5 stays on BLE.
- **Sinks:** one sink cannot carry 500 nodes, because the relays next to it queue over a
  thousand records. About one sink per 125 nodes keeps every relay's outbox near 100.

### Commands are desired state (D-27)

- **What the controller writes:** `Desired { target, command }`, one record per setting of a
  node. It is either persistent config (BME688 period and gas, diversity, advertising interval)
  or a one-shot action (blink).
- **How the target applies it:** once per version. Config converges to the latest value, and a
  one-shot action runs once for each new version of its record. The target then writes
  `Reported { setting, applied, status }`, and the controller sees the acknowledgement when that
  record reaches a sink.
- **Retries:** the controller hands the same version to the sinks again. A command waits out a
  partition, and a node acts on each version once.
- **Measured:** a command's round trip at 60 s sessions is 1.6 min at p50 with 20 nodes (3 hops),
  and 4.8 min with 500 nodes and 4 sinks (11 hops).
- **Authentication** *(not yet, DEF-27)*: Desired records will carry a MAC under a controller key
  derived from the fleet key (NFR-7), so that a node obeys only the controller.

### Positions and the link graph (D-28)

- **Links:** the initiator writes a Link record after each ranging burst, aggregated (median
  distance, σ, quality), not one per procedure.
- **Until P2:** the host solves centrally over all Link records (FR-M6) and publishes Position
  records, with the frame version, as controller records. Nodes use them, e.g. in their
  advertisements.
- **From P2:** nodes write their own Position records (FR-M2), and the host solution stays as
  the reference that validates them.

### The controller

- **The host is the controller.** It holds the global replica merged from its sinks and shows it
  (`navgrid monitor`). It runs the central solver, and writes Desired records through a sink.
  Observing is reading the replica; controlling is writing desired state.
- **A sink is a mesh node at hop count 0 with a USB port.** It syncs with its neighbours like any
  node and mirrors its replica to the host over serial: records out, Desired in.

### Validation first

- **Shared crate:** `navgrid-mesh` (`no_std`, fixed-capacity tables on devices, growable on the
  host) holds the records, the merge, classes and scope, sync, the outbox and routes. Firmware
  and host share it.
- **Simulator:** `navgrid mesh-sim` runs the crate over synthetic deployments: nodes on a grid,
  sinks, contact and ranging periods, budgets, partitions. It reports staleness, delivery and
  command latency, bytes per session and memory per node.
- **Results so far** ([spike](engineering/spikes/2026-11-02-mesh-state-sim.md)):
  - **Steady state:** at 60 s sessions, neighbourhood replicas are current ~97 % of the time.
  - **Partitions:** after a one-hour partition, replicas are back to normal in 4–12 min.
  - **Cost per node** is set by degree, not mesh size.

## 13. Operations: enrolment, commissioning, identification (FR-O)

### Lifecycle (FR-O1, FR-O4)

| State | Reached by | Where | The device |
|---|---|---|---|
| Blank | — | — | runs nothing, or unprovisioned firmware (reflects, pairs Just Works) |
| Enrolled | `navgrid provision` through the probe: firmware, fleet key, fleet number, role and defaults (D-29) | bench | joins any fleet link; registry entry *enrolled*; label printed |
| Unplaced | powered on site; its records reach a sink | site | in the mesh; the monitor lists it as *unplaced*, with its neighbours |
| Commissioned | the operator names and places it: name, room, position or pin; role and settings as desired state | field tool, or host | normal operation |
| Retired | `retire`: a controller record drops the node's records everywhere, and a desired *decommission* makes the device wipe its provisioning page | either | silent; reusable after enrolment |

- **Replace** (`replace N042 N107`): the registry moves N042's placement (name, room, pin) to N107,
  writes N107's settings as desired state, and retires N042. IDs stay with the hardware; only
  placements move.
- **A node that goes silent** past its expiry is flagged *missing* in the monitor until the
  operator retires it or it returns.
- **Factory reset:** hold the button for 5 s while the device powers up (inserting the cell or a
  reset) until the LED turns red; it wipes the provisioning page. It can't happen by accident in
  operation, and a reset device can't join the fleet until it is enrolled again.
- **One shared fleet key** (D-30): losing a device means re-enrolling the fleet with a new key.
  That is fine without production deployments (DEF-33); provisioning gets revisited before them.

### Registry and labels (FR-O5)

- **The fleet directory** on the host holds:
  - `fleet.key`, the secret: never in git;
  - `devices.json`: fleet number, node ID, role, enrolment date, status;
  - placements per site: name, room, pin, notes; the site file of §11 holds the pins.
- **Fleet numbers** (D-31): `navgrid provision` assigns the next number (`N001`, `N002`, …),
  stores it in the provisioning page, and writes a 50 × 25 mm SVG label to `labels/`: the number
  in large type and a QR code of `NAVGRID:N042:<ID>`. The device reports its number in its node
  record and beacon, so the registry can be rebuilt from the devices.
- **The monitor** joins mesh state with the registry: `N042 kitchen-NE: battery 2.41 V
  (≈ 9 days), last heard 40 s ago, 3 hops`. Battery comes from Health records; the host keeps the
  trend and estimates days left, and flags low cells.

### Finding a node, either way

| Direction | How | Through the mesh (host only) | With the field tool |
|---|---|---|---|
| Host → device | **identify**: blink a distinctive pattern for 30 s | a Desired command: minutes in a deep mesh | a direct link: seconds |
| Device → host | **button press**: the node writes an event record and sets an *attention* flag in its beacon for 30 s, advertising fast | the event reaches the host in minutes | the tool sees the flag at once, with RSSI |
| Where is it | the surveyed position on the floor plan (P2), and the registry's room and notes | yes | yes, plus a hot/cold RSSI or CS-distance finder, which reuses the final approach of §10 |

### The field tool (FR-O6, D-32)

- **What it is:** a laptop running `navgrid field` with a DK that is enrolled like any node and
  runs a field-sink mode.
- **What it does:**
  - lists nearby nodes from their beacons: fleet number, role, hops, battery, attention, RSSI;
  - connects to one over a fleet link and runs an ordinary sync session at hop 0, so desired
    state (name, pin, identify, settings) lands at once and reports come straight back;
  - joins the mesh as a temporary sink while it is there; when it leaves, peers time out and
    routes heal (§12).
- **Without it,** every operation still works through the permanent sinks and the host, at mesh
  latency. The field tool is a shortcut, not a dependency.

### Firmware: versions and updates (FR-O2)

**Versions (D-33, built).**
- **Identity:** every build carries its version, the first 32 bits of its git commit, and whether
  tracked files had uncommitted changes (`navgrid_proto::Firmware`, shown as
  `0.1.0+1950d455-dirty`). `build.rs` stamps it, and the image embeds the same as text
  (`navgrid-firmware 0.1.0+…`).
- **Reporting:** the device logs it at boot and reports it in Health, in the sink's Hello and in
  its mesh node record.
- **Recording:** `navgrid flash ELF --probe A --probe B …` flashes several devices at once (about
  2 min whatever the count) and records each one's build and date in `devices.json`.
  `fleet list` shows them.
- **Checking:** the monitor marks a device whose reported build differs from the registry's, so
  one that missed an update stands out.

**The bootloader (D-34, built).**
- **What it is:** embassy-boot (`firmware/bootloader`, 5.4 KB), a Rust bootloader on `NorFlash`,
  which embassy-nrf's RRAM driver implements on the nRF54L15. It swaps a marked update into the
  active slot, verifies Ed25519 signatures, and reverts an image that doesn't confirm itself.
- **The firmware side** (`firmware.rs`): it takes over the bootloader's 5 s watchdog, and
  confirms a new image once a fleet link is up. It restarts unconfirmed after 5 min.
- **Measured** ([spike](engineering/spikes/2026-11-03-bootloader.md)): a swap takes about 25 s
  for the whole slot, and the confirm write 75 ms.

| Region | Start | Size |
|---|---|---|
| bootloader | 0x000000 | 32 KiB |
| bootloader state | 0x008000 | 12 KiB (embassy-boot needs four 16-byte progress lines per page) |
| active image | 0x00B000 | 720 KiB; today's image is 489 KiB |
| update slot | 0x0BF000 | 724 KiB (one page more than active, for the swap) |
| spare | 0x174000 | 20 KiB |
| kept update header | 0x179000 | 4 KiB: the header of the update this image came from |
| staged update header | 0x17A000 | 4 KiB |
| provisioning, boot epoch | 0x17B000 | 4 + 4 KiB, as before |

`navgrid_proto::layout` holds the same addresses. `navgrid flash --bootloader` writes the
bootloader once per device. Every flash blanks the state region and both update headers.

**RRAM writes after boot** go through MPSL timeslots (`nrf_mpsl::Flash`), as in NCS. Written
directly, a 4 KiB page keeps the RRAM busy for tens of milliseconds, and the radio's interrupts
wait for it: ranging lost 7 % of its procedures, and once the controller asserted
([spike](engineering/spikes/2026-11-03-single-hop-update.md)). Writes at boot, before MPSL
starts, stay direct.

**Signing (D-35, built).**
- **Keys:** an Ed25519 key pair in the fleet directory (`signing.key`), separate from the fleet
  key. `navgrid provision` writes the public key into the device's provisioning page.
- **Update files:** `navgrid sign ELF --out FILE` makes one: an 88-byte `UpdateHeader` (build,
  length, a signature over the image's SHA-512 digest, CRC), then the raw image for the active
  slot.
- **Verification:** the running firmware checks an update before handing it to the bootloader
  (embassy-boot's `verify_and_mark_updated`), at boot before the radio starts, in about 1.1 s.
  It takes nothing without a provisioned key, and a leaked fleet key still can't install firmware.
- **Staging:** an update waits for the check with its image in the update slot and its header in
  the staging page. `navgrid update FILE --probe P` stages one through the probe, and the BLE
  transfer writes the same two things.

**Single hop first (D-36), built.**
- **Transfer** (`update.rs`): a node that runs an installed update offers it on its fleet links.
  The header it was installed from is kept for that, at `0x179000`. A peer on another build asks
  for it, and the image streams in 224-byte chunks on a bulk L2CAP channel (SPSM `0x0083`). The
  receiver answers once it has stored it all, and only then does the sender close the channel.
  Today the initiator offers on its CS link.
- **Install:** the receiver writes the update slot and the staging page, then restarts into the
  check at boot, the swap and the confirm, as for a probe-staged update. The new image confirms
  itself only once it is back in the mesh (a fleet link encrypted within 5 min); otherwise the
  bootloader reverts it.
- **Declining:** a peer already on the offered build declines it. So does one whose image isn't
  confirmed yet: its update slot still holds the image a revert would put back.
- **Measured** ([spike](engineering/spikes/2026-11-03-single-hop-update.md)): about 7 KB/s, one
  chunk per 30 ms connection event, so a 490 KB image takes 70 s. Ranging on the same link holds
  11.1/s throughout. Ranging then stops for about 35 s while the receiver checks the image, swaps
  it and reconnects.
- **Boards:** an update's header names the board its image is for (header format 2). A device
  installs only its own board's images and declines others.
- **From the host, through the sink:** `navgrid update FILE --sink PORT` sends the update to the
  sink over VCOM1, the same messages as over the air with 4 KiB `Data` and a CRC-32 each, one at
  a time: the sink answers each chunk with where to go on from, and the host resends any that
  gets no answer within 1 s. The sink checks the signature, then installs an update for its own
  board, or keeps one for the TAGs in its update slot and offers it to the nodes on its links.
  The host's upload takes about 60 s. The sink updates itself the same way.
- **On the sink link:** packets stay short, so the uplink keeps its 11/s, except during an update,
  when the node switches the link to full-length packets: one full-length exchange fills the
  link's 2.5 ms event. The node returns credits four chunks at a time, which leaves telemetry
  about 6.6 frames/s of the link. A 500 KB image takes about 5.5 min at 1.5 KB/s
  ([spike](engineering/spikes/2026-11-03-updates-through-the-sink.md)). From there the node
  passes it on as above.
- **Not yet:** resuming after a dropout (`Want` carries the start offset for it).
- **Why first:** at 100 kHz SWD (DEF-19), reflashing 20 TAGs takes about 2 min per probe, and
  probes and DK adapters are scarce.

**Mesh-wide later (D-36), after stage 4.**
- **Announce:** the controller writes a *desired firmware* record (ID, size, SHA-256, signature,
  rollout group), which spreads to every node like the frame (Global class, §12).
- **Spread:** a node that lacks the image pulls missing pages from a neighbour whose node record
  says it holds it, over the bulk channel, outside the 2 KB sync budget, which would take hours
  per hop. Each finished node then serves its neighbours, and sinks get the image from the host.
- **Install:** at the controller's time, canary nodes first and the rest once the canaries report
  the new build healthy. Rollout shows in node records.
- **Compatibility:** while a rollout runs, versions N and N−1 must interoperate on the air: mesh
  records and sync frames change compatibly, or in two steps.
- **Cost:** about 400 KB received per node and a little more sent, a few µAh. Time is hops ×
  (wait for a contact + about 10 s per transfer).

### LED and button (FR-O7)

The TAG has one RGB LED (P2.08–P2.10, active low) and one button (P0.00). The second LED and
button footprints are empty. The LED draws a few mA, so a 50 ms blink every 10 s would cost about
4 % of NFR-6's budget: the LED only shows events.

| Trigger | LED | Also |
|---|---|---|
| Power-up | 1 s colour: green enrolled, blue not enrolled, red provisioning page corrupt | — |
| Short press | about 3 s of status, below | event record, beacon attention for 30 s |
| Long press (3 s) | toggles **install mode** for 10 min: a short pulse every 2 s coloured by link and route quality, for placing a node at the edge of coverage | fast advertising while on |
| Identify (command) | white fast blinks, 30 s | — |
| Asset blink (FR-F6) | as identify, for the final approach | — |
| Hold 5 s at power-up | red, then wipes provisioning | factory reset |

**Status on a short press.** A white 50 ms flash acknowledges the press, then two phases with a
0.7 s dark gap between them. The pattern carries the meaning on its own and the colour repeats
it, so neither colour vision nor a miscount loses it.

| Phase | Shows | Pattern |
|---|---|---|
| Connectivity: can this node get data out? | route to a sink | green, one short blink (100 ms) per hop, 1–4; one long glow for 5 or more; solid at a sink |
| | mesh peers, no route to a sink | yellow, one long glow |
| | beacons heard, but links fail to encrypt: another fleet key | magenta, slow double blink |
| | no peers | red, fast flicker |
| | not enrolled | blue, one long glow |
| Battery: under radio load | good | 3 long green blinks (300 ms) |
| | fine | 2 long yellow blinks |
| | replace soon | 1 long red blink |

- **Critically low cell:** the whole display is one short red blink, since a longer LED load could
  brown the node out (R-9).
- **Thresholds:** they come from the CR2032 sag measurements (P0). A CR2032 holds about 3.0 V at
  rest almost to the end, so only a reading under load tells.
- **Before devices run the mesh state** (§12, stage 4), the connectivity phase can only say
  whether the node's links are up, not its hop count.
- **A second press** restarts the display.

The ADXL367 adds a third signal: a node that notices it was moved asks for a re-survey (FR-M5).
The DK sink shows host link and connected nodes on its four LEDs.

## 14. Decision log

| ID | Decision | Status |
|---|---|---|
| D-1 | Rust everywhere: Embassy + `nrf-sdc` + `TrouBLE` + `bt-hci` on firmware; std Rust on host. | **Accepted** (user requirement; feasibility checked §6) |
| D-2 | One firmware image; role (node / asset / person / sink), pinned coordinates and periods from provisioned `NodeConfig`. | **Accepted** (implemented 2026-11-02: `navgrid provision`) |
| D-3 | On tag links the tag is central + initiator and nodes are peripheral + reflector. This gives self-positioning, nodes stay unaware of tags, and per-tag compute lands on the tag. | Proposed |
| D-4 | Reflector → initiator data: **inline PCT transfer (IPT) first.** All devices are ours, and it is faster (~30 ms vs 100–200 ms per estimate), has no GATT hop, and costs less energy per range. Its limits are mode 2 only and weaker relay-attack resistance. **RAS** comes when phones do, or as the fallback if IPT fails the R-8 checks. | Proposed |
| D-5 | Wire format: postcard + COBS; telemetry types in `navgrid-proto`, mesh state and sync in `navgrid-mesh`. | Proposed |
| D-6 | Telemetry is delay-tolerant: queued data rides ranging contacts toward sinks over L2CAP CoC. No always-on routing tree and no second radio protocol. | Proposed |
| D-7 | Every link uses LTKs derived from the fleet network key and starts encryption directly, with no pairing. Phones, if they come, use LE Secure Connections. | **Accepted** (works on the TAGs with SDC and TrouBLE, 2026-11-02; §3) |
| D-8 | Firmware uses pinned revisions of our forks: `DmitriyNE/bt-hci`, `DmitriyNE/nrf-sdc` and `GreatKet/trouble`, with Embassy at the revision TrouBLE's nRF54 example uses. Upstream PRs follow. | **Accepted** |
| D-9 | Solvers: tags use a Gauss-Newton fix to start, then the tracker (D-14). Mesh uses distributed weighted least squares with neighbor covariances, bootstrapped by a central solve on the host. | Proposed |
| D-10 | Battery link policy (§3): low-duty persistent mesh links that burst to range, event-driven re-survey, wake-on-motion tags, configurable advertising. | Proposed (P0 measures alternatives) |
| D-11 | Antenna diversity: all paths the link allows; per-device opt-out via `NodeConfig::diversity`; per-path estimates combined residual-weighted by default (§4). | **Accepted** (user requirement); combiner open until P1 |
| D-12 | **3D with height sources** (CS geometry, differential barometry, carry-height prior) via `Height::Observed`; 2D is `Height::Fixed`. | **Accepted** |
| D-13 | Board selected at build time (`board-dk` / `board-tag` features, `cargo tag` / `cargo dk`). Role and diversity come from runtime `NodeConfig`. | Proposed |
| D-14 | Tag tracker: constant-velocity Kalman filter with one-measurement-at-a-time updates (range, height, zero-velocity), outlier gate with recovery, motion-onset re-opening (§8). | Proposed |
| D-15 | IMU tiers (§8): accelerometer gating on all tags; gyro dead reckoning and full inertial only if they pay off. | **Accepted** (revised v0.5: tags do not produce heading) |
| D-16 | Monitoring is our own small tool: telemetry rides ranging contacts to a USB-powered sink; `navgrid monitor` shows mesh state and logs JSON lines (§9). | **Accepted** |
| D-17 | CR2032 no-waste rules (requirements NFR-6), against a 1-month goal / 1-week floor. | **Accepted** |
| D-18 | Asset tags publish a fix when they settle; the host keeps the asset registry (§10). | Proposed |
| D-19 | Heading comes from another sensor (the HUD's magnetometer), in the world frame; tags do not estimate it (§10). | **Accepted** (revised v0.5) |
| D-20 | Final approach on direct person↔asset CS: distance direct, bearing by synthetic aperture over the person's track (§10). | Proposed |
| D-21 | Engineering records live in `docs/engineering/`: log, known defects, todo, spikes. Design stays in requirements/architecture; evidence goes in spikes. | **Accepted** (repo rule) |
| D-22 | Absolute referencing from **commissioning presets**: barometry for up; pins and/or preset node-pair bearings for yaw and handedness; one pin for origin. The host owns a versioned mesh→world transform, fitted from a site file (§11). Learning from HUD headings is a later option. | **Accepted** (revised: presets, Q15) |
| D-23 | Monitoring gets a thin web UI later: one static page plus a JSON endpoint on the host tool, sharing `navgrid monitor`'s state (§9). | Proposed (later) |
| D-24 | Mesh state is **convergent**: single-writer records with per-origin sequence numbers, merged by keeping the highest. Derived values agree by determinism or by having a single writer. No voting (§12). | **Accepted** (2026-11-02) |
| D-25 | Replication scope: each node holds its own records, a 2-hop neighbourhood and routing state; sinks and the host hold the global state (§12). | **Accepted** (2026-11-02) |
| D-26 | Routing rides the sessions: custody pushes down a hop-count gradient toward sinks, and back along learned routes toward targets, store-and-forward (§12). | Proposed (simulated, 2026-11-02) |
| D-27 | Commands are **desired state**: controller-written, authenticated, sequenced per target; targets apply them idempotently and report what they applied (§12). | **Accepted** (2026-11-02) |
| D-28 | Positions: the host solves centrally and publishes them as records first; on-node distributed solving (FR-M2) follows and is validated against it (§12). | **Accepted** (2026-11-02) |
| D-29 | Enrolment at a bench through the probe (firmware, fleet key, fleet number); everything after it over the air (§13). | **Accepted** (2026-11-02) |
| D-30 | One shared fleet key; a lost device means re-enrolling the fleet. Revisit before production deployments (§13, DEF-33). | **Accepted** (2026-11-02) |
| D-31 | Devices are labelled with a sequential fleet number, stored on the device and in the host registry (§13). | **Accepted** (2026-11-02) |
| D-32 | Every operation works through the host and the mesh; a field tool (laptop + DK as a portable sink) does the same in seconds on site (§13). | **Accepted** (2026-11-02) |
| D-33 | Every build carries its version, commit and dirty flag, reported at boot, in Health and in node records; `navgrid flash` records what each device got (§13). | **Accepted** (built 2026-11-03) |
| D-34 | The bootloader is embassy-boot: active and update slots in RRAM, swap with rollback (§13). | **Accepted** (spike 2026-11-03) |
| D-35 | Firmware images are signed with an Ed25519 key separate from the fleet key, whose public part is provisioned; the running firmware verifies an update before the bootloader takes it (§13). | **Accepted** (built 2026-11-03) |
| D-36 | Over-the-air updates come single hop first (before stage 3), then spread mesh-wide over a bulk channel with controller-scheduled, canary-first rollout (after stage 4) (§13). | **Accepted** (2026-11-03) |
