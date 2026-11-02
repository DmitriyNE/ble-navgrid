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
crates/navgrid-mesh      no_std  mesh state: records, sync between neighbours, routing to sinks and targets
host/navgrid             CLI: `sim`, `track`, `cs`, `decode`, `monitor`, `calibrate`, `mesh-sim`
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
cargo run -p navgrid -- cs initiator.log reflector.log [--json]  # raw CS captures → distance per procedure
cargo run -p navgrid -- cs --ipt initiator.log                   # same, inline PCT transfer: initiator only
cargo run -p navgrid -- monitor /dev/cu.usbmodem…3 [--json] [--calibration cal.json]  # frames from a sink DK's VCOM1
cargo run -p navgrid -- calibrate --out cal.json a.jsonl=0.90 b.jsonl=2.00   # range offsets from captures at measured distances
cargo run --release -p navgrid -- mesh-sim --nodes 500 --sinks 4   # mesh state protocol: staleness, commands, bytes, memory
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

Images link at `0xB000` and run under the bootloader (`firmware/bootloader`, architecture §13), so
a device needs the bootloader once: `navgrid flash --bootloader …` (below). `cargo tag` then
flashes and logs as before, but it leaves the bootloader's state alone. After an update
experiment, `navgrid flash` clears it.

On boot the node clears GRTC state left by earlier firmware, sets the crystal load caps, selects
antenna 0, and logs its ID, config and the controller's CS capabilities. If the SDC memory pool
size is wrong, the log prints the exact number needed; set `SDC_MEM` in `node/src/main.rs` to it.

### Programming a TAG

- **Connection:** the TAG plugs into the DK's `DEBUG OUT` header. The DK's onboard J-Link then
  debugs the TAG instead of the DK's own chip.
- **Power:** a CR2032 **or** the DK, never both. For DK power:
  - set the DK's VDD to 3.0–3.3 V in Board Configurator (nRF Connect for Desktop); the default is
    1.8 V. If it times out talking to the board controller, replug the DK's USB;
  - jumper any `VDD:IO` pin of the GPIO headers to `VDD SWD0`, one pin of the 3-pin header P5
    right of `DEBUG OUT`. To find it, measure with a cell in: it reads the cell voltage, the other
    two read 0 V.
- **Probe:** with more than one probe attached, set `PROBE_RS_PROBE=<VID:PID:serial>` from
  `probe-rs list`.
- **Autodetect:** `probe-rs info` fails to autodetect the chip (its memory APs have no ROM
  table), but `--chip nRF54L15`, which the runner passes, works.
- **Flashing:** the runner disables probe-rs's double buffering, which corrupts a word in most
  pages on the TAG (DEF-16), and verifies every flash. It runs SWD at 100 kHz, because at the
  default 400 kHz the probe loses the TAG within seconds of CS starting (DEF-19). A flash takes
  about 2 minutes.

### Two-board Channel Sounding (P0 bring-up)

Every device runs the same image; its role comes from its provisioning page, which flashing
firmware leaves alone. Provision once per device, through its probe (`probe-rs list`):

```sh
cargo run -p navgrid -- fleet init                         # once: key, registry (~/.config/navgrid, or --fleet DIR)
cargo run -p navgrid -- provision --probe <A>              # board A: reflector; gets N001 and a label
cargo run -p navgrid -- provision --probe <B> --peer <id of A>
cargo run -p navgrid -- fleet list
cargo run -p navgrid -- flash firmware/target/thumbv8m.main-none-eabihf/release/navgrid-node \
    --probe <A> --probe <B>                                # firmware onto several devices at once, recorded
    # add --bootloader firmware/bootloader/target/thumbv8m.main-none-eabihf/release/navgrid-bootloader
    # once per device (build it with `cd firmware/bootloader && cargo build --release`)
cd firmware && cargo tag                                    # each board; note "navgrid node <id>"
```

An unprovisioned device reflects and pairs Just Works. Provisioned devices encrypt their links
under the fleet key without pairing, and drop a peer that isn't in the fleet.

To get ranges on the host without a probe on the initiator, add a sink. Provision a DK with
`--role sink` and flash it with `cargo dk` (its node ID is in the boot log), provision the initiator
with `--sink <sink id>` as well, and `navgrid monitor` reads the DK's VCOM1. To reach a DK whose
`DEBUG OUT` holds a TAG, unplug the TAG first; the debugger then targets the DK's own chip.

- **The initiator** connects at 30 ms, runs the feature exchange (the SDC needs it before CS),
  moves the link to 2M with 251-byte packets, encrypts it under the fleet key, then configures CS and starts procedures: mode 2, all 72 channels, 2M PHY, inline PCT transfer. Two
  TAGs use both antennas each (2×2, four antenna paths: about 11 procedures/s); otherwise 1×1
  (about 17/s).
- **Built with `NAVGRID_LOG_STEPS` set**, both boards log each CS subevent as one line:
  `CSR <I|R> <procedure> <freq comp> <ref power> <proc done> <subevent done> <abort> <paths>
  <steps> [step bytes]`. Continuations are logged as `CSC`. The step bytes keep the HCI layout,
  so the host can join the two sides by procedure counter.
- **With inline PCT transfer** (`INLINE_PCT` in `cs.rs`, on), the initiator also logs
  `distance <procedure>: … m` from its own tones, combining the antenna paths.

Firmware dependencies are pinned revisions of our forks of bt-hci, nrf-sdc and TrouBLE, which add
the Channel Sounding support upstream lacks. See architecture §6 and D-8.

## Status

Phase P0 (bring-up). Two TAGs run Channel Sounding on our stack and log raw steps; `navgrid cs`
turns the two captures into distances (first run: median 2.34 m, p10–p90 2.28–2.39 m). What's
next: [todo](docs/engineering/todo.md). What's known broken: [defects](docs/engineering/defects.md).
