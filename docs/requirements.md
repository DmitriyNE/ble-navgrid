# navgrid — requirements

Status: **draft v0.4** (2026-11-01). §8 records what is decided. Items marked *(proposed)* still
need sign-off.

## 1. Concept

navgrid is an **ambient positioning mesh** of nRF54L15 nodes.

1. Nodes measure distances to each other **peer-to-peer** with Bluetooth LE Channel Sounding (CS).
2. From those ranges the mesh **self-surveys**: every node solves its own position, cooperatively,
   on-device.
3. **Tags** on assets and on people range against nearby nodes and are **located against the
   mesh**.
4. Ranges and positions can optionally be **routed to a central node** (sink → host), but
   positioning must not depend on it.
5. **Mesh state and sideband data** (topology, link health, positions, battery, environmental
   readings) reach a small monitoring tool of our own.

Target scale is a **building with many tags**. Early phases are smaller, but no architectural
decision may preclude building scale.

### Driving use case: find my keys

A person wearing a HUD asks where their keys are. The HUD points at the keys and shows the
distance, from across the building down to the last metre.

1. **The asset tag** on the keys is still most of the time and costs almost nothing. When it moves
   (carried in a pocket) it tracks itself. When it settles, it fixes its position and publishes it.
2. **The person tag** tracks the wearer's 3D position and heading continuously while they move.
3. **Far away:** the pointer is the bearing from the person's pose to the asset's last published
   position.
4. **Close in** (within radio range): the person tag ranges **directly** to the asset tag. CS
   gives the distance between the two; the bearing is refined from how that distance changes as
   the person moves. The asset can blink its LED.

## 2. Glossary

| Term | Meaning |
|---|---|
| **Node** | nRF54L15 member of the mesh. Ranges with neighbors, maintains its own position estimate. |
| **Pinned node** | Node with surveyed coordinates. Pins fix the mesh to the building frame. |
| **Tag** / target | Device located against the mesh; does not contribute to node positions. Two kinds: |
| **Asset tag** | Tag on a thing (keys, equipment). Mostly still. May be a single-antenna module. |
| **Person tag** | Tag worn by a person with a HUD. Needs position and **heading**. |
| **Sink** | USB-powered gateway attached to the host. Collects telemetry; serves the asset registry. |
| **Link** | BLE ACL connection between two devices; CS procedures and data both run over it. |
| **Initiator / reflector** | CS roles on a link. The initiator computes the distance; it needs the reflector's step data. |
| **Procedure** | One CS ranging exchange (many steps over many channels) → one distance estimate. |
| **Frame** | Building coordinate system (metres, right-handed, z up). |

## 3. Phases (agreed)

| Phase | Goal | Exit criterion |
|---|---|---|
| **P0 — bring-up** | Rust firmware on nRF54L15 TAG boards runs CS between two boards, with antenna diversity; raw step data reaches the host. | Our estimator agrees with Nordic `cs_de` (NCS v3.4.1 `ipt_*` / `ras_*` samples) within 5 cm on the same captures. Ranging-rate and multi-link limits measured. Energy per activity measured. CR2032 sag and BME688 pressure noise measured. |
| **P1 — find my keys, first cut** | One asset tag, one person tag, ≥4 nodes at hand-surveyed positions. 3D tracker with barometric height and IMU gating. Host view stands in for the HUD: pointer and distance. Direct person↔asset ranging for the final approach. | Position p90 (horizontal and vertical), pointer bearing error, and final-approach distance error measured on a test course. Diversity on vs off, IMU gating on vs off. Antenna-path combiner chosen from real captures. |
| **P2 — self-survey** | Nodes range each other; positions solved, first centrally on host, then distributed on nodes. | Distributed solution within 0.2 m of central reference; node p90 vs ground truth measured. |
| **P3 — building** | Link scheduling, handover, many tags, routing to sinks, provisioning, OTA, HUD integration. | Capacity targets in NFR-4 met in a multi-room deployment. |

## 4. Functional requirements

### Ranging (R)

- **FR-R1** Any device can be CS initiator or reflector, and BLE central or peripheral, chosen per link at runtime.
- **FR-R2** Support CS mode 2 (phase-based ranging, PBR) and mode 1 (round-trip timing, RTT); mode 3 where the controller supports it.
- **FR-R3** Each procedure yields: distance, uncertainty, quality score, step count, channels used, antenna-path count.
- **FR-R4** The reflector's step data reaches the initiator each procedure (transport: decision D-4).
- **FR-R5** **Raw capture mode**: both sides' per-step data (PCT I/Q, RTT timestamps, quality indicators) streamed to the host. Every on-device estimator must run on the host against recorded captures and give identical results (same `no_std` crate).
- **FR-R6** **Antenna diversity.** Devices with several antennas (TAG: 2) sound every antenna path the link allows, up to 4 (2 × 2 between two TAGs, 2 × 1 against a single-antenna module), and combine the per-path estimates.
- **FR-R7** Diversity can be **disabled per device** by configuration. The device then holds antenna 0, and every link with it uses one antenna path, which means shorter procedures and less energy. Single-antenna devices always work.

### Mesh self-survey (M)

- **FR-M1** Nodes discover neighbors and keep ranging links to a bounded neighbor set (degree ~4–8, *proposed*).
- **FR-M2** Each node maintains its position + covariance, updated from neighbor ranges and neighbors' estimates (distributed cooperative localization, on-node).
- **FR-M3** ≥3 non-collinear pinned nodes define the building frame (≥4 non-coplanar for full 3D). With no pins the mesh solves in a relative frame. Unpinned node heights come from CS geometry plus differential barometry, or from provisioning.
- **FR-M4** Nodes advertise position estimate, uncertainty, spare ranging capacity, battery level and current barometric pressure. Tags can then choose nodes, and get a height reference, without a directory service.
- **FR-M5** Detect and down-weight bad links (NLOS bias, outliers). Detect moved nodes and re-survey them.
- **FR-M6** A central batch solver on the host, over the full range graph, serves as the reference for validating the distributed solver.

### Tag positioning (T)

- **FR-T1** A tag picks nodes from advertisements (default 4–6), optimising geometry (GDOP), link quality, advertised capacity and battery.
- **FR-T2** The tag solves its own **3D** position on-device, with uncertainty, using a filter that takes one measurement at a time. Height comes from three sources, weighted by their uncertainty:
  - CS geometry, where nodes sit at varied heights;
  - **differential barometry**, the tag's BME688 against nearby nodes' BME688;
  - a loose carry-height prior.

  2D (fixed height) remains available as a special case.
- **FR-T3** Handover: as the tag moves it swaps nodes without a gap in position output.
- **FR-T4** Nodes, asset tags, person tags and the sink run the same firmware, with the role from configuration. Tags are TAGs or single-antenna nRF54L modules.
- **FR-T5** An **asset tag publishes its position when it settles** (after the settling burst, FR-T6) toward the sink. Person tags may publish theirs too.
- **FR-T6** **Motion gating.** A tag ranges at its moving rate only while the IMU reports motion. On stopping it takes a short settling burst, then drops to a slow keep-alive rate. Wake-on-motion uses the ADXL367, so a still tag costs close to nothing.
- **FR-T7** **IMU fusion**, in tiers. The TAG has no magnetometer, so heading is never measured; it is integrated by the gyro and aligned to the building frame from motion.
  - (a) **All tags:** accelerometer still/moving detection drives FR-T6, zero-velocity updates and the filter's motion noise.
  - (b) **Person tags:** gyro-integrated heading, aligned to the building frame whenever the CS-tracked walking direction is reliable. Step detection (pedestrian dead reckoning) between ranges.
  - (c) Later, if it pays off: full inertial propagation with a heading state.

### Find (F)

- **FR-F1** **Asset registry:** the last published position of each asset (with uncertainty and age) is available to the finder. It starts at the host, fed through the sink (FR-S); a mesh-local query comes later if needed.
- **FR-F2** **Person pose:** the person tag provides 3D position and heading (yaw) in the building frame (FR-T7b). If the HUD has its own IMU, its frame is aligned to ours instead (Q13).
- **FR-F3** **Pointer:** bearing (azimuth and elevation) and distance from the person to the asset, in the person's frame, ready for a HUD to render. The host view renders it until a HUD is integrated.
- **FR-F4** **Final approach:** within radio range, the person tag ranges directly with the asset tag over CS. Distance comes straight from CS; bearing comes from direct ranges taken along the person's tracked path (synthetic aperture). This replaces mesh-based pointing close in, where position errors would swamp the bearing (R-13).
- **FR-F5** Asset tags stay connectable at a slow advertising interval, so a nearby person tag can start direct ranging without paging.
- **FR-F6** *(Optional)* An asset tag blinks its LED on request during the final approach.

### Data routing (D)

- **FR-D1** Telemetry (ranges, positions, health) can be routed from any device to one or more sinks; sinks forward to the host over USB/UART.
- **FR-D2** Routing is optional for positioning. Asset registry updates (FR-F1) need it.

### Monitoring and sideband (S)

- **FR-S1** Mesh state reaches the monitoring tool: nodes (role, config, firmware, position and covariance, last seen), links (distance, quality, antenna paths, success rate) and tags.
- **FR-S2** Every device reports health: battery voltage, die temperature, uptime, firmware version, and telemetry queue depth.
- **FR-S3** Environmental sideband from the BME688: temperature, humidity and pressure on a configurable period (default 60 s), and gas resistance only when enabled because its heater dominates the cost. Pressure doubles as the barometric reference for FR-T2.
- **FR-S4** Telemetry is delay-tolerant: it is queued and forwarded on ranging contacts, never on dedicated wake-ups. Records carry their age, so the sink can timestamp them without clock sync (NFR-5).
- **FR-S5** **Monitoring is our own small engineering tool**, not an external stack. `navgrid monitor` shows live mesh state in the terminal and appends every record to a JSON-lines log. `navgrid decode --json` covers offline captures.

### Host (H)

- **FR-H1** Ingest from sinks / directly attached devices; record to file; replay.
- **FR-H2** Central reference solver and live view: floor plan, nodes, links, tags, uncertainty, and the find pointer (FR-F3).
- **FR-H3** Provisioning (identity, role, pins, keys) and fleet health.

### Operations (O)

- **FR-O1** Provisioning of device identity, role, diversity on/off, pinned coordinates (or height only) and network key.
- **FR-O2** OTA firmware update over BLE (P3).
- **FR-O3** Observability: `defmt` over RTT in development; per-link counters (procedures, failures, quality) in the field.

## 5. Non-functional requirements

Numbers are *(proposed)* targets. Nordic publishes no CS accuracy figure; their staff quote "±0.5 m
at best", so P0 measurements will re-baseline these.

| ID | Requirement |
|---|---|
| **NFR-1** Accuracy | Ranging p90 ≤ 0.5 m line-of-sight, ≤ 20 m. Tag position p90 ≤ 1.0 m horizontal and ≤ 1.0 m vertical inside covered areas, node error included. Floor identification ≥ 99 %. Self-survey node p90 ≤ 0.5 m. |
| **NFR-2** Update rate | Moving person tag: pose output ≥ 2 Hz. In simulation, walking needs about 2 single-range updates per second without dead reckoning. A still tag may stop updating. Mesh links re-range rarely (minutes to hours) and whenever a node is moved. |
| **NFR-3** Latency | Tag position ≤ 500 ms after the last contributing range. Asset registry update ≤ 1 min after the asset settles, when a sink path exists. |
| **NFR-4** Scale | ≥ 500 nodes and ≥ 100 concurrent tags per building. Per-node connection count and airtime are bounded and enforced (admission control via FR-M4). |
| **NFR-5** No time sync | Ranging and positioning need no network-wide clock sync (CS is two-way). |
| **NFR-6** Power | **Every TAG runs on a CR2032** (~225 mAh). **Battery life: 1 month is the goal (≈ 310 µA average), 1 week is acceptable (≈ 1.3 mA).** Asset tags, still most of the time, should far exceed that. **Waste is still a defect:**<br>• No activity runs continuously without a reason.<br>• Every periodic activity has a configurable period.<br>• Expensive sensors (gyro, gas heater) run only when they pay off.<br>• Ranging is motion-gated (FR-T6).<br>• Telemetry rides existing contacts (FR-S4).<br>Each activity's energy is measured in P0. |
| **NFR-7** Security | All devices are ours (nRF54Lxx). All CS links are encrypted (the CS security-start procedure needs an encrypted ACL), with keys derived from a fleet network key, so no per-pair pairing. Phones may come later; they would add standard RAS and LE Secure Connections pairing for that class of device. |
| **NFR-8** Robustness | Losing nodes degrades accuracy gracefully; the mesh re-solves without operator action. |
| **NFR-9** Reproducibility | Estimators are deterministic. Recorded captures plus replay run in CI. |
| **NFR-10** Pointing | Pointer bearing error ≤ 20° p90 when the asset is ≥ 3 m away. Final-approach distance error ≤ 0.3 m p90. |

## 6. Constraints

- **C-1 Hardware:** Nordic **nRF54L15 TAG** boards for nodes and tags, **powered by CR2032 cells**. Features: 2 antennas through a SKY13348 switch on P1.09/P1.10; LFXO and HFXO on internal load caps (9 pF / 8 pF); DC/DC; ADXL367 low-power accelerometer; BMI270 IMU (no magnetometer); BME688; RGB LED. **Single-antenna nRF54L modules** as asset tags. nRF54L15 DK for bench work.
- **C-2 Language:** Rust everywhere. Firmware is `no_std` on Embassy; host is std Rust. Algorithm crates are `no_std` and shared by both.
- **C-3 BLE stack:** Nordic SoftDevice Controller via `nrf-sdc`, `TrouBLE` host, `bt-hci`. No NCS/Zephyr in product firmware. NCS C samples are allowed as reference fixtures.
- **C-4 CS is point-to-point per connection:** every range needs a live ACL between the pair, and the controller schedules CS relative to connection events. This drives the scaling design (see architecture §5).

## 7. Risks

| ID | Risk | Mitigation |
|---|---|---|
| R-1 | The Rust stack is missing CS glue (architecture §6). | Small upstream patches in pinned forks; NCS samples as cross-check. |
| R-2 | Controller limits for concurrent CS links and per-node ranging rate are undocumented. Nordic's defaults suggest ~100–200 ms per estimate with RAS, ~30 ms with inline PCT transfer, and 90 ms central ACL spacing with CS on. | Measure in P0 before committing NFR-2/NFR-4. |
| R-3 | Connection-based ranging capacity scales with links. Crowds of tags saturate nearby nodes. | Airtime budget, advertised capacity, adaptive rates. |
| R-4 | Indoor multipath/NLOS gives positive range bias; dominant error source. | Quality metrics, multi-antenna, robust solvers. |
| R-5 | Distributed self-survey can fail to converge (flip ambiguity, bad init, error growth away from pins). | Host central solve for bootstrap and validation (FR-M6). |
| R-6 | Depends on unreleased git revisions of TrouBLE / nrf-sdc / bt-hci 0.11. | Pin revisions; upstream patches early. |
| R-7 | Battery vs connection-based ranging: every range needs a live link, and links and discovery cost energy on both ends. | Low-duty persistent mesh links, burst only to range (D-10); wake-on-motion via ADXL367; P0 energy measurements. |
| R-8 | Inline PCT transfer (D-4) is new (Core 6.3): it is unverified whether it supports multiple antenna paths and whether `bt-hci` 0.11 parses its event data. | Check in P0; fall back to RAS (standard, slower) if either fails. |
| R-9 | CR2032 pulse load: a CS procedure keeps the radio on for tens of ms. Coin cells sag under pulses, more so when cold or aged, which risks brown-out mid-procedure. | Measure sag on TAGs in P0; cap procedure length; check bulk capacitance. |
| R-10 | Barometric height: building HVAC holds zones at different pressures (~8.5 cm of height per Pa), and sensors have offsets. BME688 pressure noise is not yet characterised for this use. | Reference only nearby nodes in the same zone; calibrate per-node offsets against CS geometry; measure noise in P0. |
| R-11 | The gyro costs far more than the accelerometer. The 1-month budget affords it on person tags while they move, not continuously. | Gyro only on person tags and only while moving; measure its draw in P0. |
| R-12 | **Heading without a magnetometer.** Aligning gyro heading to the walking direction assumes the tag faces where the person walks: true for a head-worn tag walking forward, false when sidestepping or looking around. Gyro yaw drifts while the person stands and turns. | Align only on straight, steady walking; gyro drift over minutes is small; spike: bearing from CS antenna-path phase differences on a 2-antenna person tag. |
| R-13 | **Close-in pointing.** Bearing error ≈ combined position error ÷ distance: 0.5 m at 1 m is ~27°. Mesh-based pointing fails exactly where the user needs it most. | Final approach on direct person↔asset ranging (FR-F4); show distance prominently close in. |

## 8. Decisions and open questions

Decided 2026-11-01:

| ID | Question | Decision |
|---|---|---|
| Q1 | Node power | **Battery**: CR2032 (NFR-6). |
| Q2 / Q9 | 2D or 3D; where heights come from | **3D**, with height from CS geometry, differential barometry and a loose carry-height prior; 2D is a special case (FR-T2). |
| Q4 | Tag trust | **Our own nRF54Lxx devices only**; phones maybe later (NFR-7). |
| Q6 | Hardware | **nRF54L15 TAG** boards with diversity, which can be disabled per device; single-antenna modules as asset tags (C-1, FR-R6/R7). |
| Q8 | Battery life | **1 month goal, 1 week acceptable** (NFR-6). No squeezing for a year. |
| Q10 | Monitoring | **Our own small engineering tool** (FR-S5). |
| Q11 | What tags are attached to | **Assets and people.** Driving use case: find my keys with a HUD pointer (§1, FR-F). |
| — | IMU | Accelerometer on all tags; gyro heading on person tags (FR-T7). No magnetometer. |
| — | Phases | P0–P3 as in §3. |

Open:

| ID | Question | Default until answered |
|---|---|---|
| Q3 | Are the NFR-1/2/4/10 numbers right for the product? | As proposed. |
| Q5 | Routing transport: opportunistic over ranging contacts, or 802.15.4/Thread? | Opportunistic over BLE ranging contacts (D-6). |
| Q7 | Who initiates on tag links? | The tag initiates (D-3); the person tag initiates on direct person↔asset links. |
| Q13 | The HUD: which device, does it have its own IMU (and compass), where is the person tag worn (on the HUD/head or on the body), and how does the HUD get the pointer (BLE from the person tag, or via phone/host)? | Person tag worn on the head; host view stands in for the HUD. |
| Q14 | Where does the asset registry live long-term: host only, or also queryable inside the mesh (for no-sink deployments)? | Host, fed through the sink (FR-F1). |
