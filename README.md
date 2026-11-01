# navgrid

Ambient indoor positioning mesh on nRF54L15. Nodes range each other peer-to-peer with Bluetooth LE
Channel Sounding, self-survey their positions, and serve as the reference for locating tags on assets and people
(think "find my keys" with a HUD pointer).
Rust everywhere: firmware and host.

- [Requirements](docs/requirements.md): scope, phases, FR/NFR, open questions
- [Architecture](docs/architecture.md): link roles, ranging pipeline, energy, Rust stack status, tracking, find my keys, decision log
- [Engineering records](docs/engineering/): log, known defects, todo, spikes

## Layout

```
crates/navgrid-proto     no_std  wire messages (postcard + COBS), shared by firmware and host
crates/navgrid-ranging   no_std  CS channel response → distance (phase slope, RTT); channel simulator
crates/navgrid-solver    no_std  multilateration (target); cooperative mesh solver later
host/navgrid             CLI: `sim`, `decode`
firmware/                separate workspace for thumbv8m.main-none-eabihf
  node/                  single firmware image (node / target role from config)
```

## Host

```sh
cargo test --workspace
cargo run -p navgrid -- sim                                   # synthetic CS → ranges → 3D fix (baro height)
cargo run -p navgrid -- sim --multipath 3 --antenna-paths 4   # reflection + TAG↔TAG diversity
cargo run -p navgrid -- sim --height fixed                    # 2D
cargo run -p navgrid -- sim --trials 500 --multipath 3 --antenna-paths 4   # compare path combiners
cargo run --release -p navgrid -- track                       # walking target: ranging rate vs accuracy, IMU gating on/off
cargo run -p navgrid -- decode capture.bin [--json]           # decode COBS/postcard frames
```

## Firmware

Needs `probe-rs` and an nRF54L15 TAG or DK. Run from `firmware/` so its `.cargo/config.toml`
applies; the board is a build-time feature:

```sh
cd firmware
cargo tag                  # nRF54L15 TAG: flash + defmt log over RTT
cargo dk                   # nRF54L15 DK
cargo build --release --features board-tag   # build only
```

On boot the node sets up DC/DC and the crystal load caps, selects antenna 0, logs its ID, config
and the controller's CS capabilities, then advertises as `navgrid` (every 1 s by default). Role, diversity,
advertising and sensor periods come from `NodeConfig` (defaults until provisioning exists). If the SDC memory pool size is wrong,
the log prints the exact number needed; set `SDC_MEM` in `node/src/main.rs` to it.

Firmware dependencies are pinned git revisions, because Channel Sounding needs `bt-hci` 0.11 and
the crates.io releases of TrouBLE / nrf-sdc are older. See architecture §6 and D-8.

## Status

Phase P0 (bring-up). The scaffold builds and is tested on the host; nothing has run on hardware
yet. What's next: [todo](docs/engineering/todo.md). What's known broken:
[defects](docs/engineering/defects.md).
