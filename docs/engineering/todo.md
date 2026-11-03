# Todo

Highest priority first within each phase. Done items are deleted; the [log](log.md) records them.

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
- [ ] **Distance calibration** (DEF-6), when accuracy matters: record `monitor --json` at several
      measured separations and run `navgrid calibrate`. Offsets are keyed by link, path count
      (which fixes the timing configuration) and method. Check whether one constant fits all
      separations, or a scale term is needed.
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
- [ ] **Move all advertising and scanning to the extended commands** once a node does both roles
      (the SDC rejects mixing the legacy and extended sets; DEF-17).
- [ ] **Diversity evaluation** (DEF-25): per-path ranges at several placements against measured
      distances. Find out why path 3 is bad, and pick the path combiner on accuracy.
- [ ] **One-path rate, opt-in:** a 15 ms connection gives one-path links 33/s instead of 16.7,
      but leaves no room for a second link (spike 2026-11-02-cs-ranging-rate). Decide with
      multi-link scheduling; candidate use: the dedicated final-approach link (D-20).
- [ ] **Cross-check estimator:** compare against NCS v3.4.1 `ipt_*` / `ras_*` samples and `cs_de`
      on the same captures; settle sign and units (DEF-6).
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
the 20 TAGs: about 8 nodes, a person tag, 2–3 asset tags, and the DK sink.

Stage 1, the `navgrid-mesh` crate and `navgrid mesh-sim`, is done
([spike](spikes/2026-11-02-mesh-state-sim.md)).

Stage 2, identity and configuration at runtime, is done too: the provisioning page and boot
epoch in RRAM, `navgrid provision`, the FR-M4 beacon, and fleet-key link keys (log 2026-11-02).

3. [ ] **Mesh links:** several links per node, with CS scheduled across them (generalising
       DEF-26's per-connection spacing), low-duty links that burst to range (D-10), and the
       one-path rate decision. Measure ranging rate and energy per link count.
4. [ ] **Sync over the links:**
       - `navgrid_mesh::wire` sessions on SPSM `0x0082` per mesh link, a session every 60 s with
         a 2 KB budget (replaces DEF-1's dedicated uplink; health and environment records carry
         their age, DEF-10). A session completes only when both sides have seen the other's
         `Done`;
       - the sink mirrors its replica to the host over VCOM1, and takes Desired records back
         (UART RX);
       - `navgrid monitor` mesh-state view (FR-S5);
       - a controller CLI with the first commands: LED blink, BME688 on and period, diversity;
       - the controller key from the fleet key, and a MAC on desired state (DEF-27);
       - the beacon's hop count from the mesh state, and neighbour discovery from beacons
         (FR-M1, DEF-11).
       - Exit: commands and records cross partitions and several hops on the bench mesh;
         latency and bytes per contact against the simulator.
5. [ ] **Positions:** the host solver over Link records (FR-M6) publishes Position records with
       the frame version; then distributed on-node solving (FR-M2), validated against the host.

Protocol follow-ups, in the simulator first:

- [ ] **Delta summaries** (DEF-28): per peer, send only the classes changed or newly in scope
      since the last complete session with it.
- [ ] **Compact device layout** for stored records (DEF-29): 48-bit IDs, `u8` lengths.
- [ ] **Hop counts** (DEF-30): poison the route when the sink path goes, instead of counting up to
      `MAX_HOPS`.
- [ ] **Commands to unrouted nodes** (DEF-31): fall back to pushing toward the target's last
      known neighbours.
- [ ] **Retiring a node:** a controller record that drops a node's records everywhere.
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

Items 1–3 are done (log 2026-11-02):
- fleet numbers, the registry and labels;
- the TAG's LED and button;
- battery readings.

Left over from them:
- the registry: learn what a device runs from its Hello and Health (DEF-44);
- check that a phone scans the label QR;
- try the factory reset on a TAG;
- bring in install mode on the long press (item 7).

4. [ ] **Identify and button events through the mesh:** the blink command (with stage 4's desired
       state), an event record for button presses, and `navgrid identify`.
5. [ ] **Registry operations:** `navgrid place`, `replace` and `retire` (with the mesh's retire
       record); the monitor shows fleet numbers, names, *unplaced* and *missing*.
6. [ ] **Field tool:**
       - a DK field-sink mode: beacon list with RSSI, direct fleet links, and a sync session at
         hop 0;
       - `navgrid field`: list, identify, name and place, plus install-mode readings.
7. [ ] **Install mode:** link and route quality on the LED, with fast advertising, for 10 min.

## P1 — find my keys, first cut

- [ ] **Tag firmware:** on-device tracker, motion gating with a settling burst, asset
      publish-on-settle.
- [ ] **Absolute referencing:** a host site file with commissioning presets (pins, node-pair
      bearings, declination). Fit with `fit_pins` / `fit_bearings` + `Frame::anchored`; the host
      owns the `Frame` and its version; positions carry the frame version (FR-A5).
- [ ] **Host asset registry and pointer** (FR-F1, FR-F3) in the host view.
- [ ] **Final approach:** direct person↔asset ranging and synthetic-aperture bearing (D-20).
      Optional: asset LED blink.
- [ ] **Spike:** first-path IFFT / super-resolution estimator vs phase slope.
- [ ] **Simulator:** full-fix vs single-range tracking side by side; GDOP/battery-aware node
      selection; a find-my-keys scenario with pointer-error statistics.

## P2 / P3

- [ ] **Event-driven re-survey** of moved nodes (FR-M5).
- [ ] **OTA and capacity tests.**

## Later

- [ ] **Heading-learned referencing** (FR-A3c): extract straight, steady walking segments from
      tracks, pair them with the HUD's heading stream, and run `frame::fit_headings` with
      place/direction weighting. This references a mesh without presets.
- [ ] **Thin web UI** (FR-S6, D-23): a static page plus a JSON endpoint on the host tool. It
      shows the floor plan, mesh, health and sideband, and the asset registry, and handles pin
      placement.
