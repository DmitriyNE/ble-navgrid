# Todo

Highest priority first within each phase. Done items are deleted; the [log](log.md) records them.

## P0 — bring-up

- [ ] **Upstream PRs** from the forks: bt-hci step parsing, nrf-sdc CS bindings, TrouBLE CS
      feature, `iso` import fix and connection-parameter-update identifier fix (drop the blank-line
      commit first).
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
- [ ] **After first boot:** check the antenna-switch default and callback, and boot a DK (DEF-2,
      DEF-3).
- [ ] **Nordic's samples on our TAGs** (DEF-18): build NCS v3.4.1 `ras_initiator` and
      `ras_reflector` for `nrf54l15tag/nrf54l15/cpuapp` (RTT console snippet) and run them on
      `ce36dfcc4c2d` / `df96e04a5900`. If they range, capture their HCI traffic and diff it with
      ours; if they abort too, take it to Nordic with their samples as the reproduction. Untried
      from the old list: sub mode 1, a DK pair.
- [ ] **embassy-nrf** (DEF-22): make the GRTC time driver's compare channel configurable, or move
      it off CC[7]–[11], which MPSL owns on the nRF54L.
- [ ] **nrf-sdc fork:** bind `sdc_hci_cmd_vs_cs_params_set` (CS event length, T_PM, board distance
      offset) and drop the raw FFI in `cs.rs`.
- [ ] **bt-hci + TrouBLE** (DEF-20): parse the CS capabilities complete v2 event (`0x38`), and
      make TrouBLE log rather than panic on an LE event it can't parse. Also propose running the
      feature exchange on connect, which the SDC needs before CS (log 2026-11-02).
- [ ] **Move all advertising and scanning to the extended commands** once a node does both roles
      (the SDC rejects mixing the legacy and extended sets; DEF-17).
- [ ] **Antenna permutation tables** (DEF-21): map multi-path tones to antenna paths in
      `navgrid-ranging::steps`, from the Core spec tables, so diversity captures work.
- [ ] **Inline PCT transfer** (`cs_enhancements`) so the initiator alone has both sides (R-8).
- [ ] **Cross-check estimator:** compare against NCS v3.4.1 `ipt_*` / `ras_*` samples and `cs_de`
      on the same captures; settle sign and units (DEF-6).
- [ ] **Energy per activity** with a power profiler on a TAG: advertising, idle link, CS per
      antenna-path count, BMI270 gyro, BME688. Also measure CR2032 sag during CS (R-9).
- [ ] **BME688 driver** and pressure-noise characterisation: two TAGs side by side, then known
      height steps (R-10).
- [ ] **ADXL367 wake-on-motion** driver.
- [ ] **Calibrate the range σ model** from captures (DEF-7).
- [ ] **Sink role:** frames out over USB CDC (COBS/postcard).
- [ ] **`navgrid monitor`:** terminal mesh-state view plus JSON-lines log (FR-S5).
- [ ] **P-5 embassy-nrf:** nRF54L crystal load-cap config, and stale GRTC state cleared by the
      time driver at init, upstream.
- [ ] **CI:** host tests and clippy, plus firmware clippy and build for both boards.

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
- [ ] **Configuration:** `NodeConfig` in flash, and provisioning over serial (DEF-5).
- [ ] **Telemetry envelope** with age and hop count (DEF-10).

## P2 / P3

- [ ] **Mesh links:** low-duty persistent connections, ranging bursts, event-driven re-survey
      (D-10).
- [ ] **Self-survey:** central solver on the host, then the distributed one on nodes.
- [ ] **Routing:** delay-tolerant forwarding toward sinks (D-6).
- [ ] **Fleet-key LTKs** without pairing (D-7).
- [ ] **OTA, provisioning tooling, capacity tests.**

## Later

- [ ] **Heading-learned referencing** (FR-A3c): extract straight, steady walking segments from
      tracks, pair them with the HUD's heading stream, and run `frame::fit_headings` with
      place/direction weighting. This references a mesh without presets.
- [ ] **Thin web UI** (FR-S6, D-23): a static page plus a JSON endpoint on the host tool. It
      shows the floor plan, mesh, health and sideband, and the asset registry, and handles pin
      placement.
