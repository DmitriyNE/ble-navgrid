# Todo

Highest priority first within each phase. Done items are deleted; the [log](log.md) records them.
The order of work and the exit test per milestone are in [the roadmap](../roadmap.md).

## P0 — bring-up

- [ ] **Upstream PRs** from the forks:
      - bt-hci: step parsing;
      - nrf-sdc: CS bindings, the nRF54L critical-section fix (DEF-23), RRAM standby;
      - TrouBLE: the CS feature, the `iso` import fix and the connection-parameter-update
        identifier fix (drop the blank-line commit first). Also an issue: the per-link ACL
        send lock goes to whichever sender asks first, so a busy one starves the rest (DEF-41,
        worked around).
- [ ] **nrf-sdc fork docs:** say on `cs_count` / `cs_cfg` that they must follow the `support_*`
      calls and `cs_antenna_switch_callback` (the SDC ignores or rejects them otherwise).
- [ ] **nrf-sdc optional follow-up:**
      `LeCsTest` binding (variable-length tail, hand-written like `LeSetExtAdvData`).
- [ ] **bt-hci follow-up (optional):** rename `reserved` → `cs_enhancements` in create-config and
      test params (API change, so a separate PR).
- [ ] **Report to probe-rs** (DEF-16): double-buffered flashing of an nRF54L15 through a J-Link
      OB replaces one word near each page start with its successor. Evidence in the 2026-11-02
      log; check whether it also happens on the DK's own chip.
- [ ] **Standalone J-Link Pro** doesn't connect to the TAG at all; the SWD adapter may be bad.
- [ ] **After first boot:** check the antenna-switch default outside CS (the CS callback works:
      2×2 ranges on four paths), and boot a DK (DEF-2, DEF-3).
- [ ] **Distance calibration** (DEF-6): record `monitor --json` at several tape-measured
      separations and run `navgrid calibrate`. Needs longer DK cables or battery TAGs on a tape,
      in a clear spot off any surface (spike 2026-11-03-four-tags-proximity). The clean points
      so far (10, 60, 90 cm) fit a constant of about 2.3 m with one antenna each; with 2 × 2
      antennas each antenna path has its own offset, so key the calibration by antenna path, not
      only by path count. Then put the offset on the device (Nordic's board distance offset, or
      ours in `cs.rs`).
- [ ] **Legacy connect** (DEF-17): retry `LE Create Connection` now that the cache is on.
- [ ] **Sub mode 1 and faster procedures:** with sub mode 1, 20 ms and a procedure every 5
      events, two in three procedures aborted (unspecified, or no CS_SYNC). Find out why before
      using RTT steps; RTT pairing gives nonsense today (DEF-21).
- [ ] **embassy-nrf** (DEF-22): make the GRTC time driver's compare channel configurable, or move
      it off CC[7]–[11], which MPSL owns on the nRF54L.
- [ ] **nrf-sdc fork:** bind `sdc_hci_cmd_vs_cs_params_set` (CS event length, T_PM, board distance
      offset) and drop the raw FFI in `cs.rs`.
- [ ] **bt-hci + TrouBLE** (DEF-20): parse the CS capabilities complete v2 event (`0x38`), and
      make TrouBLE log rather than panic on an LE event it can't parse. Also propose running the
      feature exchange on connect, which the SDC needs before CS (log 2026-11-02).
- [ ] **Diversity evaluation** (DEF-25): per-path ranges at several placements against measured
      distances. Find out why path 3 is bad, and pick the path combiner on accuracy.
- [ ] **One-path rate, opt-in:** a 15 ms connection gives one-path links 33/s instead of 16.7,
      but leaves no room for a second link (spike 2026-11-02-cs-ranging-rate). Decide with
      multi-link scheduling; candidate use: the dedicated final-approach link (D-20).
- [ ] **The first-path estimator on real multipath** (DEF-51): `navgrid_ranging::first_path`
      is built and reported by `navgrid cs` beside the phase slope. Captures from a bad
      placement next (`NAVGRID_LOG_STEPS`, both boards): two TAGs flat on the floor 1.5 m apart,
      and one 0.5 m from a DK; then judge it against the phase slope on those and on the 40 cm
      one (`docs/engineering/captures/2026-11-03-40cm/`), tune the threshold and the gain
      guard, and port it to the device (no allocation; the grid scan costs about 100 k complex
      MACs per antenna path per search).
- [ ] **Cross-check estimator:** compare against NCS v3.4.1 `ipt_*` / `ras_*` samples and `cs_de`
      on the same captures; settle sign and units (DEF-6).
- [ ] **Inline PCT transfer** (D-4) is described as on by default, but the firmware doesn't use
      it: a capture needs both boards' logs, and the reflector's tones go unused on the device.
      Either implement it or correct the design text.
- [ ] **Single antenna path:** N001 provisioned `--no-diversity` gave no ranges in 90 s; check
      with a probe on it.
- [ ] **Battery forecast** (DEF-52): ignore the first hour's sag, or fit from the settled part.
- [ ] **Energy per activity** with a power profiler on a TAG: advertising, idle link, CS per
      antenna-path count, BMI270 gyro, BME688. Also measure CR2032 sag during CS (R-9).
- [ ] **BME688 driver** and pressure-noise characterisation: two TAGs side by side, then known
      height steps (R-10).
- [ ] **ADXL367 wake-on-motion** driver.
- [ ] **Calibrate the range σ model** from captures (DEF-7).
- [ ] **P-5 embassy-nrf:** nRF54L instruction cache enabled at init (DEF-18), crystal load-cap
      config, and stale GRTC state cleared by the time driver at init, upstream.
- [ ] **CI:** host tests and clippy, plus firmware clippy and build for both boards.

## Mesh state and control (FR-C, architecture §12)

Staged; each stage ends with something measured. It runs alongside the rest of P0, and replaces
the old telemetry, configuration, mesh-link, routing and self-survey items. Bench deployment from
the 20 TAGs: about 8 anchors, a mobile tag, 2–3 asset tags, and the DK sink.

Stage 1, the `navgrid-mesh` crate and `navgrid mesh-sim`, is done
([spike](spikes/2026-11-02-mesh-state-sim.md)).

Stage 2, identity and configuration at runtime, is done too: the provisioning page and boot
epoch in RRAM, `navgrid provision`, the FR-M4 beacon, and fleet-key link keys (log 2026-11-02).

3. [ ] **Mesh links** (D-37): the policy crate, `link-sim` and the firmware are built (log
       2026-11-04; DEF-57). Next, the desk mesh: flash 8 anchors and 2 tags from the TAGs and the
       DK sink (`navgrid provision --role anchor|mobile`, the DK `--role anchor --sink`), watch
       links, hellos, bursts and empties per link count against the simulator, and fix what the
       bench shows. Then: the beacon's hop count from the policy, surveys on motion from the
       accelerometer (DEF-58), the tag's rest and settling (FR-T6), the CS-slot limit in the
       beacon (DEF-59), and measure ranging rate and energy per link count.
4. [ ] **Sync over the links** (partly done, log 2026-11-04): the `Driver` runs the session
       frame by frame on devices (DEF-62); every link runs a session at link-up (its hellos are
       the role exchange) and then every 60 s, started by the central, with a 2 KB budget; nodes
       write their node record (neighbours from the links, hops from the sessions), health every
       30 min and a Link record per burst; the sink mirrors what it keeps to the host as
       `Message::Record`, and the beacon's hop count comes from the replica. `navgrid control`
       signs a desired-state record with the controller key (from the fleet key, DEF-27) and
       sends it to a sink, which injects it; nodes check the MAC, act on it (blink; the other
       settings report Unsupported until settings change at runtime) and write Reported records.
       Left:
       - settings at runtime: environment period and gas, diversity, advertising interval
         (NodeConfig changes applied live, or a restart with the desired values);
       - `navgrid monitor` mesh-state view (FR-S5): nodes, links, health and positions as a
         table from the mirrored records, with ages;
       - the mesh `Environment` record once the BME688 is read;
       - the controller's node ID and key in provisioning rather than derived (DEF-27).
       - Exit: commands and records cross partitions and several hops on the desk mesh;
         latency and bytes per contact against the simulator.
5. [ ] **Positions** (partly done, log 2026-11-04): the central solver (`navgrid_solver::mesh`,
       `navgrid solve`) places every node from the mirrored Link records with the pins fixed,
       writes Position records back through the sink, and an unpinned anchor advertises the
       controller's position of it; a tag multilaterates from its bursts against the anchors'
       advertised positions and writes its own Position record every few seconds, its links
       syncing every 5 s; an unpinned anchor places itself from pinned or controller-placed
       neighbours and writes its own Position (FR-M2, first cut); `navgrid find` gives the
       pointer from the mirrored positions (FR-F3). Left: the self-fix validated against the
       host's solve on the desk mesh and, if it holds, anchors solving cooperatively from each
       other's fixes; the tracking filter on tags once the accelerometer gates motion (FR-T6,
       FR-T7); re-survey of moved nodes (FR-M5); referencing by `fit_pins` when pins come in
       another frame than the world's (FR-A3); a live pointer (the monitor or the web UI) in
       place of the log-based one.

Protocol follow-ups, in the simulator first:

- [ ] **Compact device layout** for stored records (DEF-29): 48-bit IDs, `u8` lengths.
- [ ] **Fleet key at rest** (DEF-33): APPROTECT on provisioned devices, and the key in the KMU.
- [ ] **Simulate link loss and mid-session drops,** and reboots (new epochs).

## Firmware updates (FR-O2, architecture §13)

Done (log 2026-11-03): version tracking, the bootloader
([spike](spikes/2026-11-03-bootloader.md)), signing, and single-hop updates between TAGs
([spike](spikes/2026-11-03-single-hop-update.md)), and updates from the host through the sink
([spike](spikes/2026-11-03-updates-through-the-sink.md)). Left before stage 3:

1. [ ] **The uplink during an update from the sink** (DEF-45): Health first in the outbox, or
       several frames per SDU.
2. [ ] **The sink's UART receive** (DEF-43): a buffered UARTE, which re-arms in its interrupt
       handler, instead of `read_until_idle`.
3. [ ] **Resume and retry** (DEF-39): pick up from the last page written after a dropout (`Want`
       carries the offset), and offer again to a peer that declined while unconfirmed.
4. [ ] **A shorter swap** (DEF-38): a slot closer to the image size, or buffered writes.

After stage 4: the desired-firmware record, neighbour-to-neighbour spreading, canary rollout and
version compatibility during rollouts.

## Operations (FR-O, architecture §13)

Each item works through the host and the mesh first; the field tool shortens it.

**Diagnostics over the air** (architecture §13) are done (log 2026-11-03): boot and crash
reports, logs at a configured level, a sink serving 4 nodes, and reflectors' own sink links.
Left from them: measure the TAG and DK 32 kHz crystals against a reference and declare their
real accuracy (DEF-2), and find what wedges the sink's uploads (DEF-46).

Items 1–3 are done (log 2026-11-02):
- fleet numbers, the registry and labels;
- the TAG's LED and button;
- battery readings.

Left over from them:
- check that a phone scans the label QR;
- try the factory reset on a TAG;
- bring in install mode on the long press (item 7).

4. [ ] **Identify and button events through the mesh:** the blink command (with stage 4's desired
       state), an event record for button presses, and `navgrid identify`.
5. [ ] **Registry operations:** `navgrid fleet place` and `retire` exist (log 2026-11-04;
       replace = retire, then provision the new device under the number). Left: the monitor
       and the service's page showing names, *unplaced* and *missing* (enrolled but not heard
       from).
6. [ ] **Field tool:**
       - a DK field-sink mode: beacon list with RSSI, direct fleet links, and a sync session at
         hop 0;
       - `navgrid field`: list, identify, name and place, plus install-mode readings.
7. [ ] **Install mode:** link and route quality on the LED, with fast advertising, for 10 min.
- [ ] **Floor tests with the node high** (spike 2026-11-03-four-tags-proximity, last section): a tag
      under a table, against a wall, in a bag, at 2–5 m sideways from a node 2.3 m up; then two
      nodes at once. The calibration (DEF-6) in that geometry, by pair and antenna path.
- [ ] **Node placement and antenna** (spike 2026-11-03-four-tags-proximity, last sections): a node
      sits about 1 m below the ceiling (the two-ray first fade is at 2h²/λ, the breakpoint at
      4h²/λ: 1.5 and 2.9 m for 0.3 m below, 16 and 33 m for 1 m below); nodes at unequal heights;
      a node variant with a vertical antenna (the flat TAG is horizontally polarized, and on edge
      the 3–4 m fade halved). Test node-to-node under a ceiling at two depths before fixing the
      mount.
- [ ] **Tones over the air, follow-ups:** rebalance the outbox (phase ranges are dropped before
      RTT ones when tones are on); make the tones rate and the combiner provisioning settings;
      port the matrix pencil (Cadzow completion, singular-value order, amplitude and pole-radius
      filters, as in `scripts/tones_estimators.py`) into `navgrid tones` as a host diagnostic if
      the raw tones stay; check the ceiling-bounce reading (a component 2.2 m past the direct
      path at 3.5 m on 0.45 m chairs) with the pucks at another height or under a lower ceiling.
- [ ] **RTT as a real range** (DEF-55): send the reflector's mode-1 ToD−ToA to the initiator over
      the CS link (what RAS does), subtract it, drop the 100 m report offset, and try the
      sounding-sequence RTT types for precision. Then check the per-pair bias against the pair's
      frequency offset (DEF-2).
- [ ] **Re-run the kept captures with `--ipt`** (DEF-54) and look at the first-path estimator's
      numbers again: 0.87 m at 0.6 m and 1.35 m at 0.8 m on a first look.

## P1 — find my keys, first cut

- [ ] **Tag firmware:** on-device tracker, motion gating with a settling burst, asset
      publish-on-settle.
- [ ] **Absolute referencing:** a host site file with commissioning presets (pins, node-pair
      bearings, declination). Fit with `fit_pins` / `fit_bearings` + `Frame::anchored`; the host
      owns the `Frame` and its version; positions carry the frame version (FR-A5).
- [ ] **Host asset registry and pointer** (FR-F1, FR-F3) in the host view.
- [ ] **Final approach:** direct mobile↔asset ranging and synthetic-aperture bearing (D-20).
      Optional: asset LED blink.
- [ ] **Simulator:** full-fix vs single-range tracking side by side; GDOP/battery-aware node
      selection; a find-my-keys scenario with pointer-error statistics.

## P2 / P3

- [ ] **Event-driven re-survey** of moved nodes (FR-M5).
- [ ] **OTA and capacity tests.**

## Later

- [ ] **Heading-learned referencing** (FR-A3c): extract straight, steady walking segments from
      tracks, pair them with the HUD's heading stream, and run `frame::fit_headings` with
      place/direction weighting. This references a mesh without presets.
- [ ] **Web UI, next** (FR-S6, D-23; `navgrid serve` has the page and the endpoint, log
      2026-11-04): the asset registry as its own list (last settled fix per asset tag, age); the
      floor plan with a background image and axes; sideband (environment) once the BME688 is
      read; commissioning presets for referencing (FR-A3) on the page; names for nodes beyond
      their fleet numbers.
