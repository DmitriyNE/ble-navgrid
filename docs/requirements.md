# navgrid — requirements

Status: **draft v0.6** (2026-11-02). §8 records what is decided. Items marked *(proposed)* still
need sign-off.

## 1. Concept

navgrid is an **ambient positioning mesh** of nRF54L15 nodes.

1. Nodes measure distances to each other **peer-to-peer** with Bluetooth LE Channel Sounding (CS).
2. From those ranges the mesh **self-surveys**: every node solves its own position, cooperatively,
   on-device.
3. **Tags** on assets and on people range against nearby nodes and are **located against the
   mesh**.
4. The mesh frame is **referenced absolutely** (east, north, up), so its positions combine with
   headings from other sensors (a HUD's magnetometer) and with maps.
5. Ranges and positions can optionally be **routed to a central node** (sink → host), but
   positioning must not depend on it.
6. **Mesh state and sideband data** (topology, link health, positions, battery, environmental
   readings) reach a small monitoring tool of our own.
7. The mesh keeps one **shared state** that converges across partitions. Routing, the link graph
   and positions, sideband and commands all ride it, and a central controller observes and steers
   it (FR-C).

Target scale is a **building with many tags**. Early phases are smaller, but no architectural
decision may preclude building scale.

### Driving use case: find my keys

A person wearing a HUD asks where their keys are. The HUD points at the keys and shows the
distance, from across the building down to the last metre.

1. **The asset tag** on the keys is still most of the time and costs almost nothing. When it moves
   (carried in a pocket) it tracks itself. When it settles, it fixes its position and publishes it.
2. **The person tag** tracks the wearer's 3D position while they move. Heading comes from the
   HUD's own sensors (e.g. a magnetometer), in world coordinates.
3. **Far away:** the pointer is the bearing from the person to the asset's last published
   position, both in the world frame (FR-A), relative to the HUD's heading.
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
| **Person tag** | Tag worn by a person with a HUD. Provides position; the HUD provides heading. |
| **Mesh frame / world frame** | The frame the mesh solves in (its own) vs east-north-up (absolute). FR-A relates the two. |
| **Sink** | USB-powered gateway attached to the host. Collects telemetry; serves the asset registry. |
| **Link** | BLE ACL connection between two devices; CS procedures and data both run over it. |
| **Initiator / reflector** | CS roles on a link. The initiator computes the distance; it needs the reflector's step data. |
| **Procedure** | One CS ranging exchange (many steps over many channels) → one distance estimate. |
| **Frame** | Building coordinate system (metres, right-handed, z up). |
| **Record** | One unit of mesh state with a single writer, its origin, and a sequence number. The shared state is the set of records (FR-C). |
| **Controller** | The host, through a sink: it holds the global state, solves positions until nodes do, and writes commands as desired-state records. |

## 3. Phases (agreed)

| Phase | Goal | Exit criterion |
|---|---|---|
| **P0 — bring-up** | Rust firmware on nRF54L15 TAG boards runs CS between two boards, with antenna diversity; raw step data reaches the host. | Our estimator agrees with Nordic `cs_de` (NCS v3.4.1 `ipt_*` / `ras_*` samples) within 5 cm on the same captures. Ranging-rate and multi-link limits measured. Energy per activity measured. CR2032 sag and BME688 pressure noise measured. |
| **P1 — find my keys, first cut** | One asset tag, one person tag, ≥4 nodes at hand-surveyed, north-aligned positions (pins, FR-A3a). 3D tracker with barometric height and IMU gating. Host view stands in for the HUD: pointer and distance. Direct person↔asset ranging for the final approach. | Position p90 (horizontal and vertical), pointer bearing error, and final-approach distance error measured on a test course. Diversity on vs off, IMU gating on vs off. Antenna-path combiner chosen from real captures. |
| **P2 — self-survey** | Nodes range each other; positions solved, first centrally on host, then distributed on nodes. Mesh referenced from commissioning presets (FR-A3). | Distributed solution within 0.2 m of central reference; node p90 vs ground truth measured; frame yaw within NFR-11. |
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
- **FR-T7** **IMU fusion**, in tiers. Tags do not need to produce heading: the HUD provides it (FR-F2).
  - (a) **All tags:** accelerometer still/moving detection drives FR-T6, zero-velocity updates and the filter's motion noise.
  - (b) *Optional:* gyro and step detection (pedestrian dead reckoning) between ranges, if it lowers the ranging rate needed while walking by enough to pay for the gyro.
  - (c) Later, if it pays off: full inertial propagation.

### Find (F)

- **FR-F1** **Asset registry:** the last published position of each asset (with uncertainty and age) is available to the finder. It starts at the host, fed through the sink (FR-S); a mesh-local query comes later if needed.
- **FR-F2** **Person pose:** the person tag provides 3D position; heading comes from another sensor, typically the HUD's magnetometer, in the world frame. Positions are converted to the world frame (FR-A) before the pointer is computed, so heading and bearing share a reference.
- **FR-F3** **Pointer:** bearing (azimuth and elevation) and distance from the person to the asset, in the person's frame, ready for a HUD to render. The host view renders it until a HUD is integrated.
- **FR-F4** **Final approach:** within radio range, the person tag ranges directly with the asset tag over CS. Distance comes straight from CS; bearing comes from direct ranges taken along the person's tracked path (synthetic aperture). This replaces mesh-based pointing close in, where position errors would swamp the bearing (R-13).
- **FR-F5** Asset tags stay connectable at a slow advertising interval, so a nearby person tag can start direct ranging without paging.
- **FR-F6** *(Optional)* An asset tag blinks its LED on request during the final approach.

### Absolute referencing (A)

A self-surveyed mesh knows its shape, not its orientation, handedness or origin. Barometry fixes
"up"; the rest must come from outside.

- **FR-A1** **World frame:** x east, y north, z up, relative to true north. Magnetic declination is a per-site setting, applied to compass headings. A site may also carry a geodetic anchor (latitude, longitude, altitude) for maps.
- **FR-A2** **Vertical:** z is up and floors are separated, from barometric heights (FR-T2, FR-M3).
- **FR-A3** **Yaw and handedness** (all that pointing needs) come from **commissioning presets** on a few nodes, in either form or mixed:
  - (a) **Pins:** nodes with known absolute coordinates. ≥ 2 give yaw and offset; ≥ 3 non-collinear also give handedness. They are tape-measured, or read off a north-referenced floor plan.
  - (b) **Preset bearings:** the known world bearing from one node to another (e.g. "node 2 is at 75° from node 1"). One bearing gives yaw; ≥ 2 in different directions also give handedness. Longer baselines are more accurate.
  - (c) *Later, optional:* learned from heading observations. A device with its own absolute heading (a HUD) walks straight while the mesh tracks it, and many such segments in varied places give yaw and handedness without presets.
- **FR-A4** **Origin** (translation) comes from one pin or a floor-plan anchor. It is needed for maps and display, not for pointing.
- **FR-A5** **Versioned transform:** the mesh→world transform carries a version. Every published position states its frame version; the host converts, and devices that output world coordinates receive the current transform. The transform is re-estimated when pins change or heading observations drift.

### Data routing (D)

- **FR-D1** Telemetry (ranges, positions, health) can be routed from any device to one or more sinks; sinks forward to the host over USB/UART.
- **FR-D2** Routing is optional for positioning. Asset registry updates (FR-F1) need it.

### Mesh state and control (C)

- **FR-C1** **Shared state:** the mesh keeps one state (nodes, links, positions, health, sideband, commands) as records with a single writer each. Replicas that have seen the same records agree, however and in whatever order the records arrived, across partitions, with no voting.
- **FR-C2** **Routing:** records move hop by hop over ranging contacts toward sinks, and commands move back toward their target, delay-tolerant and without a dedicated wake-up (FR-S4). No routing tree to maintain.
- **FR-C3** **Link graph and positions** are part of the state: link records from each link's initiator, and position records that the host publishes until nodes solve their own (FR-M2, FR-M6). Each node holds its 2-hop neighbourhood; sinks and the host hold everything.
- **FR-C4** **Sideband** (health, environment, FR-S2/S3) is carried as records too, with their age (NFR-5).
- **FR-C5** **Commands** route to any node: persistent config ("enable the BME688", "diversity off") and one-shot actions ("blink the LED"). The controller writes the desired state; the target applies each command once and reports what it applied. Only the controller can command (NFR-7).
- **FR-C6** **Controller:** the host, through a sink, observes the whole state, sees which commands each node has applied, and gets the current positions and link graph. Several sinks may feed it.

### Monitoring and sideband (S)

- **FR-S1** Mesh state reaches the monitoring tool: nodes (role, config, firmware, position and covariance, last seen), links (distance, quality, antenna paths, success rate) and tags.
- **FR-S2** Every device reports health: battery voltage, die temperature, uptime, firmware version, and telemetry queue depth.
- **FR-S3** Environmental sideband from the BME688: temperature, humidity and pressure on a configurable period (default 60 s), and gas resistance only when enabled because its heater dominates the cost. Pressure doubles as the barometric reference for FR-T2.
- **FR-S4** Telemetry is delay-tolerant: it is queued and forwarded on ranging contacts, never on dedicated wake-ups. Records carry their age, so the sink can timestamp them without clock sync (NFR-5).
- **FR-S5** **Monitoring is our own small engineering tool**, not an external stack. `navgrid monitor` shows live mesh state in the terminal and appends every record to a JSON-lines log. `navgrid decode --json` covers offline captures.
- **FR-S6** *(Later)* A **thin web UI** served by the host tool: live mesh view on a floor plan, health and sideband, asset registry, and pin placement for absolute referencing (FR-A3a). It reads the same state as `navgrid monitor`.

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
| **NFR-10** Pointing | Pointer bearing error ≤ 20° p90 when the asset is ≥ 3 m away, excluding the HUD's own heading error. Final-approach distance error ≤ 0.3 m p90. |
| **NFR-11** Referencing | Mesh→world yaw error ≤ 5° p90; it adds 1:1 to pointer bearing error. Handedness correct. Origin within 0.5 m where pins exist. |

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
| R-11 | The gyro costs far more than the accelerometer. | Tags only need it for optional dead reckoning (FR-T7b); use it only if measurements show a net saving. |
| R-12 | **Referencing error goes straight into pointing.** A mis-surveyed pin or a wrong preset bearing biases the mesh yaw, and a short baseline magnifies mesh position errors into bearing error (0.1 m over 5 m ≈ 1°). | Presets on long baselines. Use more presets than the minimum and report their misfit and agreement, so a bad one shows. |
| R-13 | **Close-in pointing.** Bearing error ≈ combined position error ÷ distance: 0.5 m at 1 m is ~27°. Mesh-based pointing fails exactly where the user needs it most. | Final approach on direct person↔asset ranging (FR-F4); show distance prominently close in. |
| R-14 | **State sync costs** airtime, RAM and energy on coin cells, and the global state grows with the mesh. Records several hops from a sink may lag far behind. | Per-node scope bounded by degree, not by mesh size; a per-contact byte budget with priorities; size, convergence and latency measured in a simulator at 20 and 500 nodes before firmware (architecture §12). |

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
| — | IMU | Accelerometer on all tags (FR-T7). |
| — | Heading | Provided by another sensor (e.g. the HUD's magnetometer), not by tags. The mesh must be **absolutely referenced** so the two combine (FR-A). |
| Q13 | The HUD | Not relevant for now. The host view stands in. |
| Q14 | Asset registry location | **Host only** for now; may extend into the mesh later (FR-F1). |
| — | Monitoring UI | Terminal tool now; a **thin web UI later** (FR-S6). |
| Q15 | Referencing source | **Commissioning presets:** a few nodes with known absolute coordinates, or preset bearings between node pairs (FR-A3a/b). Learning from HUD headings is a later option. |
| — | Phases | P0–P3 as in §3. |

Decided 2026-11-02 (FR-C, architecture §12):

| ID | Question | Decision |
|---|---|---|
| — | What "consensus" means | **Convergence**: single-writer records, merged by sequence number; replicas that have seen the same writes agree. No voting or leader (D-24). |
| — | Who holds what | **Neighbourhood plus sinks:** each node holds its own records, its 2-hop neighbourhood and routing state; sinks and the host hold everything (D-25). |
| — | Where positions are solved | **Host first, nodes later:** the host solves centrally and publishes positions as records; distributed on-node solving (FR-M2) follows and is validated against it (D-28). |
| — | Command semantics | **Desired plus reported state:** the controller writes the desired state with a sequence; the target applies it once and reports what it applied (D-27). |

Open:

| ID | Question | Default until answered |
|---|---|---|
| Q3 | Are the NFR-1/2/4/10 numbers right for the product? | As proposed. |
| Q5 | Routing transport: opportunistic over ranging contacts, or 802.15.4/Thread? | Opportunistic over BLE ranging contacts (D-6). |
| Q7 | Who initiates on tag links? | The tag initiates (D-3); the person tag initiates on direct person↔asset links. |
