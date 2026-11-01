# Todo

Highest priority first within each phase. Done items are deleted; the [log](log.md) records them.

## P0 — bring-up

- [ ] **P-1 nrf-sdc:** done on branch `cs-hci-commands`, awaiting review. Optional follow-up:
      `LeCsTest` binding (variable-length tail, hand-written like `LeSetExtAdvData`).
- [ ] **P-2 TrouBLE:** done on branch `feat/channel-sounding`, awaiting review.
- [ ] **TrouBLE conn-param-update identifier fix:** done on branch
      `fix/conn-param-update-res-identifier`, awaiting review. The firmware needs it together with
      the CS branch: merge both into GreatKet `main` (or an integration branch) before switching.
- [ ] **bt-hci follow-up (optional):** rename `reserved` → `cs_enhancements` in create-config and
      test params (API change, so a separate PR).
- [ ] **Firmware:** switch to the forks; replace the raw-FFI stopgaps (capabilities read,
      `sdc_cfg_raw`, antenna callback) with the new APIs. Override `bt-hci-transport` together with
      `bt-hci` in `[patch.crates-io]`. A temporary build of this already passed for both boards.
- [ ] **First boot on a TAG:** check the boot log (SDC memory, CS capabilities); verify DC/DC,
      crystals and the antenna-switch default (DEF-2, DEF-3, DEF-4).
- [ ] **Two-TAG CS ranging:** inline PCT transfer first, RAS as fallback (R-8). Stream raw step
      data to the host over RTT/USB.
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
- [ ] **P-5 embassy-nrf:** nRF54L DC/DC and crystal load-cap config, upstream.
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
