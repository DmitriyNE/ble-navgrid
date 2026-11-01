# Spike: can the firmware be Rust, with Channel Sounding?

**Question.** Is BLE Channel Sounding on nRF54L15 reachable from a pure-Rust firmware stack
(Embassy, `nrf-sdc`, TrouBLE, `bt-hci`), and what is missing?

**Answer.** Yes. The controller and HCI layers are there. Missing pieces: about 20 lines of
bindings in `nrf-sdc` (P-1), an event hook in TrouBLE (P-2), our own distance estimation (P-4),
and nRF54L DC/DC and crystal setup in `embassy-nrf` (P-5). Status table: architecture §6.

## Method

- **Crate versions:** crates.io API.
- **Source checks:** shallow clones of `embassy-rs/bt-hci`, `embassy-rs/trouble` and
  `alexmoon/nrf-sdc`, grepped for CS commands, events, builder support and HCI dispatch.
- **bt-hci history:** release tags checked for when CS appeared.
- **NCS facts:** the v3.4.1 tree and docs: samples, host API, `cs_de`, RAS, SDC limits.
- **TAG board:** devicetree in `sdk-zephyr` `ncs-v3.4.1`, antenna-switch code in `sdk-nrf`
  v3.4.1, and Zephyr's nRF54L SoC init for crystal load capacitors and DC/DC.

## Findings

**Rust crates** (2026-11-01):

| Crate | Version / revision | CS status |
|---|---|---|
| `bt-hci` | 0.11.0 (2026-10-08) | All LE CS commands (0x0089–0x0096) and events, step-data parser. **0.10.x has none.** |
| `nrf-sdc` / `nrf-mpsl` | 0.4.0 on crates.io (uses bt-hci 0.10); git `a22ca71` (2026-10-08, bt-hci 0.11) | Bundled SDC is from NCS v3.4.0 (Core 6.3). `support_channel_sounding_*` builder flags exist. **No `sdc_cmd!` bindings for the 16 `sdc_hci_cmd_le_cs_*` functions**; no setter for `SDC_CFG_TYPE_CS_COUNT` / `CS_CFG`. Builder config is applied eagerly, so raw `sdc_cfg_set` between `Builder::new()` and `build()` works. |
| `trouble-host` | 0.8.0 on crates.io (bt-hci 0.10); git `866ee71` (2026-10-28, bt-hci 0.11) | `Stack::command()` sends any HCI command. **CS LE events hit `warn!("Unknown LE event!")` and are dropped.** No RAS. |
| `embassy-nrf` | 0.11 / git `3cd51e6` | nRF54L15 supported. **No `dcdc` or `internal_capacitors` config for nRF54L** (only for other chips). The PAC exposes `REGULATORS.VREGMAIN.DCDCEN` and `OSCILLATORS.XOSC32KI.INTCAP` / `XOSC32M.CONFIG.INTCAP`. |

- **CS create config:** in `bt-hci` 0.11, `LeCsCreateConfigParams::reserved` sits in the byte
  Core 6.3 names `cs_enhancements`, which enables inline PCT transfer. The SDC struct
  (`sdc_hci_cmd_le_cs_create_config_t`) has the same layout.
- **CS capabilities:** the SDC return struct for "read local supported capabilities" is packed and
  identical to the HCI return parameters, so `bt-hci`'s `FromHciBytes` parses it. The node
  firmware uses this at boot.

**Nordic NCS v3.4.1** (latest stable, LTS, 2026-09-17; Zephyr 4.4.2):

- **Samples** in `samples/bluetooth/channel_sounding/`:
  - `ras_initiator` / `ras_reflector` use the GATT Ranging Service.
  - `ipt_initiator` / `ipt_reflector` (new in v3.4.0) use inline PCT transfer: no RAS, mode 2 only,
    weaker attack resilience.
  - All support `nrf54l15dk` and have `nrf54l15tag` overlays (2 antennas, 4 paths).
- **Host API** in `zephyr/bluetooth/cs.h`, with `bt_conn_cb` callbacks
  `le_cs_read_remote_capabilities_complete`, `le_cs_config_complete`,
  `le_cs_security_enable_complete`, `le_cs_procedure_enable_complete` and
  `le_cs_subevent_data_available`. These were renamed between NCS 2.9 and 3.0.
- **`cs_de`** (`CONFIG_BT_CS_DE`, still experimental, CMSIS-DSP) gives IFFT, phase-slope and RTT
  estimates per antenna path. Since v3.3.0 the report building is the app's job. Nordic calls it
  a simple reference.
- **SDC limits:**
  - Multiple concurrent CS links are allowed (`CONFIG_BT_CTLR_SDC_CS_COUNT`, default 1); no
    tested maximum is published.
  - Central ACL event spacing defaults to 90 ms with CS on. Tested connection intervals are 7.5,
    10, 20 and 50 ms.
  - Not supported: multiple configs per connection, front-end modules, coexistence, NADM,
    2M-2BT PHY, 10 ns RTT.
- **Antenna switch:** up to 4 paths and 4 antennas through a GPIO switch that must settle within
  6 µs (`nordic,bt-cs-antenna-switch`). SDC entry points are `sdc_cs_antenna_switch_callback_set`
  and `sdc_cfg_cs_cfg_t { max_antenna_paths_supported, num_antennas_supported }`.
- **Rates in sample settings:** ~30 ms per estimate with IPT, 100–200 ms with RAS. No official
  figure exists.
- **Accuracy:** no Nordic specification. DevZone staff: "the best you can expect is ±0.5 m".

**nRF54L15 TAG board** (`boards/nordic/nrf54l15tag`, ncs-v3.4.1):

- **Antenna switch:** SKY13348 on P1.09 (V1) and P1.10 (V2), active high, one line per antenna
  (`multiplexing-mode = 0`; NCS drives it break-before-make).
- **Crystals:** LFXO 9 pF and HFXO 8 pF, both on internal load capacitors. The DK uses 17 pF and
  15 pF. Both boards run vregmain in DC/DC mode.
- **Peripherals:**
  - RGB LED1 on P2.08 / P2.10 / P2.09 (active low), button on P0.00;
  - BME688 on I2C21;
  - ADXL367;
  - BMI270 on SPI22.

## Sources

- https://github.com/nrfconnect/sdk-nrf/releases/tag/v3.4.1
- https://github.com/nrfconnect/sdk-nrf/tree/v3.4.1/samples/bluetooth/channel_sounding
- https://github.com/nrfconnect/sdk-zephyr/blob/ncs-v3.4.1/include/zephyr/bluetooth/cs.h
- https://github.com/nrfconnect/sdk-nrf/blob/v3.4.1/include/bluetooth/cs_de.h
- https://github.com/nrfconnect/sdk-nrf/blob/v3.4.1/subsys/bluetooth/controller/Kconfig
- https://github.com/nrfconnect/sdk-nrf/blob/v3.4.1/subsys/bluetooth/controller/cs_antenna_switch.c
- https://nrfconnectdocs.nordicsemi.com/ncs/3.4.1/nrfxlib/softdevice_controller/doc/channel_sounding.html
- https://github.com/nrfconnect/sdk-nrfxlib/blob/v3.4.1/softdevice_controller/doc/scheduling.rst
- https://github.com/nrfconnect/sdk-zephyr/tree/ncs-v3.4.1/boards/nordic/nrf54l15tag
- https://github.com/nrfconnect/sdk-zephyr/blob/ncs-v3.4.1/soc/nordic/nrf54l/soc.c
- https://devzone.nordicsemi.com/f/nordic-q-a/129255/channel-sounding-performance-results/571472
- https://devzone.nordicsemi.com/f/nordic-q-a/116560/channel-sounding---speed-and-multiple-devices/514117
