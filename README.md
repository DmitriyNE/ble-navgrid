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

On boot the node clears GRTC state left by earlier firmware, sets the crystal load caps, selects
antenna 0, and logs its ID, config and the controller's CS capabilities. If the SDC memory pool
size is wrong, the log prints the exact number needed; set `SDC_MEM` in `node/src/main.rs` to it.

### Programming a TAG

- **Connection:** the TAG plugs into the DK's `DEBUG OUT` header. The DK's onboard J-Link then
  debugs the TAG instead of the DK's own chip.
- **Power:** a CR2032 **or** 3.3 V on the DK's `VDD SWD0` pin (jumper from a `VDDIO` pin), never
  both.
- **Probe:** with more than one probe attached, set `PROBE_RS_PROBE=<VID:PID:serial>` from
  `probe-rs list`.
- **Autodetect:** `probe-rs info` fails to autodetect the chip (its memory APs have no ROM
  table), but `--chip nRF54L15`, which the runner passes, works.
- **Verify:** the runner verifies every flash. If verification fails, flash again; an unverified
  image runs corrupted code and faults at random places (DEF-16).

### Two-board Channel Sounding (P0 bring-up)

The role is chosen at build time until provisioning exists:

```sh
cd firmware
cargo tag                                                    # board A: reflector; note "navgrid node <id>"
NAVGRID_ROLE=initiator NAVGRID_PEER=<id of A> cargo tag      # board B: initiator
```

With two probes attached, pick one per build with `probe-rs list` and probe-rs's probe selector.

- **The initiator** connects at 20 ms, pairs (LE Secure Connections, Just Works), then configures
  CS and starts procedures: mode 2, all 72 channels, 1×1 antennas, 2M PHY, every 10 connection
  events.
- **Both boards** log each CS subevent as one line:
  `CSR <I|R> <procedure> <freq comp> <ref power> <paths> <steps> [step bytes]`. Continuations are
  logged as `CSC`. The step bytes keep the HCI layout, so the host can join the two sides by
  procedure counter.

Firmware dependencies are pinned revisions of our forks of bt-hci, nrf-sdc and TrouBLE, which add
the Channel Sounding support upstream lacks. See architecture §6 and D-8.

## Status

Phase P0 (bring-up). The scaffold builds and is tested on the host; nothing has run on hardware
yet. What's next: [todo](docs/engineering/todo.md). What's known broken:
[defects](docs/engineering/defects.md).
