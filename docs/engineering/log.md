# Engineering log

Newest first. Conventions: [README](README.md).

## 2026-11-01

**Repo rules and initial commit.** Commit messages are terse, 1–2 sentences, with no trailers or
prefixes (see `CLAUDE.md`). Engineering records live here. The scaffold went in as the initial
commit on `main`.

**Decisions, requirements v0.4.**

- **Battery life:** 1 month is the goal, 1 week is acceptable. No year-long squeeze. At ≈ 310 µA
  average, the gyro on person tags is affordable while moving.
- **Monitoring:** our own small tool (`navgrid monitor`, terminal view plus JSON-lines log).
- **Tags** go on assets and people. Driving use case: **find my keys** with a HUD pointer. That
  makes person heading (no magnetometer: gyro aligned to the walking direction) and a direct
  person↔asset final approach first-class (requirements FR-F, architecture §10).
- **Roles:** `Role` is now Node / Asset / Person / Sink, and the default advertising interval is
  back to 1 s.

**Tracking and height**
([spike](spikes/2026-11-01-tracking-height-imu.md)).

- Added the one-measurement-at-a-time `Tracker` and `navgrid track`.
- Barometry makes 3D viable under multipath: p90 2.4 m → 1.0 m.
- IMU gating saves 36 % of procedures at the same accuracy, once there is a settling burst.
- The first result was misleading: I suspected filter overconfidence and added motion-onset
  handling, but the real cause was the missing settling burst and an unfair comparison.

**Decisions, requirements v0.3.**

- Power source: CR2032.
- 3D, using differential barometry from the BME688 (it is on every TAG).
- IMU in tiers by energy cost.
- Monitoring of mesh state plus sensor sideband.

**TAG board support.** Found and handled two gaps:

- **embassy-nrf has no nRF54L DC/DC or crystal load-capacitor config.** `board.rs` ports Zephyr's
  formula using FICR trims; the values come from the boards' devicetrees.
- **TAG antenna switch:** SKY13348 on P1.09/P1.10, one line per antenna. It is driven from the
  SDC's antenna callback; antenna count and path limit go through raw `sdc_cfg_set`, because the
  nrf-sdc builder lacks setters. Diversity can be turned off per device.

**Antenna-path combining**
([spike](spikes/2026-11-01-antenna-path-combining.md)). Diversity cuts p90 by ~40 % under moderate
multipath. Picking the lowest-residual path is *not* better on average. Residual-weighted is the
default.

**Ranging quality metric.** Adjacent-channel coherence is blind to multipath: a +1.8 m bias showed
coherence 0.9998. Switched quality to the phase-fit residual.

**Rust CS stack feasibility** ([spike](spikes/2026-11-01-rust-cs-stack.md)). Pure Rust works.
CS needs `bt-hci` 0.11, so TrouBLE and nrf-sdc are pinned to git revisions. Gaps: nrf-sdc CS
bindings (P-1), TrouBLE CS event hook (P-2), our own estimator (P-4).

**Toolchain.** The rustup auto-install of 1.97 (pinned in `rust-toolchain.toml`) left the clippy
component registered but without binaries; parallel cargo runs during the install are the likely
cause. Fixed with `rustup component remove/add clippy --toolchain 1.97`.

**Requirements captured (v0.1).**

- Peer-to-peer CS mesh of nRF54L15 nodes that self-surveys.
- Tags located against it, solving on-device, with optional central collection.
- Building scale; Rust for firmware and host.
