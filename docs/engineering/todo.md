# Todo

Highest priority first within each phase. Done items are deleted; the [log](log.md) records them.

## P0 — bring-up

- [ ] **P-1 nrf-sdc:** bind the CS HCI commands (`sdc_cmd!` per command); add `cs_count()` /
      `cs_cfg()` setters and a safe antenna-switch callback setter. Upstream PR; fork until merged.
- [ ] **P-2 TrouBLE:** pass CS LE events to an `EventHandler` hook. Upstream PR; fork until merged.
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
- [ ] **Person heading:** BMI270 gyro, aligned to the CS-tracked walking direction (D-19).
- [ ] **Host asset registry and pointer** (FR-F1, FR-F3) in the host view.
- [ ] **Final approach:** direct person↔asset ranging and synthetic-aperture bearing (D-20).
      Optional: asset LED blink.
- [ ] **Spike:** bearing from CS antenna-path phase differences on a 2-antenna person tag (R-12).
- [ ] **Spike:** first-path IFFT / super-resolution estimator vs phase slope.
- [ ] **Simulator:** full-fix vs single-range tracking side by side; GDOP/battery-aware node
      selection; a find-my-keys scenario with pointer-error statistics.
- [ ] **Configuration:** `NodeConfig` in flash, and provisioning over serial (DEF-5).
- [ ] **Telemetry envelope** with age and hop count (DEF-10).
- [ ] **Answers needed:** Q13 (the HUD) and Q14 (where the registry lives).

## P2 / P3

- [ ] **Mesh links:** low-duty persistent connections, ranging bursts, event-driven re-survey
      (D-10).
- [ ] **Self-survey:** central solver on the host, then the distributed one on nodes.
- [ ] **Routing:** delay-tolerant forwarding toward sinks (D-6).
- [ ] **Fleet-key LTKs** without pairing (D-7).
- [ ] **OTA, provisioning tooling, capacity tests.**
