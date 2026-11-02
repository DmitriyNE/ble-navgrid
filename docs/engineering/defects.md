# Known defects and limitations

What is wrong or unverified in what exists **now**. Planned-but-missing features live in
[todo.md](todo.md). Conventions: [README](README.md).

## Open

| ID | Area | Defect | Impact |
|---|---|---|---|
| DEF-1 | firmware | CS runs on two TAGs and both log raw steps over RTT, but the distance is computed only offline by `navgrid cs`: no on-device ranging and no step transfer between the boards. | Ranging needs both boards on probes. |
| DEF-2 | firmware | The node boots on a TAG (crystals start, DC/DC on), but the DK boot, the exact crystal load-cap values (ported from Zephyr, FICR-trimmed) and the TAG antenna-switch callback are unverified. | Frequency offset or RF misbehaviour would show only in an RF test or a CS run. |
| DEF-3 | firmware | Antenna 0 is selected outside CS by driving SKY13348 V1 high. NCS leaves both lines low, and the switch truth table has not been checked. | Ordinary BLE traffic could go through the wrong or an isolated path. |
| DEF-4 | firmware | `CS_LINKS` (6) is a guess. `SDC_MEM` is measured on a TAG (95,448 bytes); the one-antenna figure (77,016) is derived, not measured. | A wrong size fails boot with "Memory buffer too small" and the needed size, or warns "too big". |
| DEF-5 | firmware | `NodeConfig` is a compiled-in default. There is no flash storage or provisioning, so every device boots as a Node with diversity on. | All devices behave the same. |
| DEF-6 | ranging | Phase-slope distances read about 2.4 m long on the TAG pair (no board offset calibrated); sign and scale check out against hardware and Nordic's estimators. RTT units (0.5 ns) and sign are still unchecked. | Absolute ranges are off by a constant until calibrated. |
| DEF-7 | ranging | Placeholder range σ model `0.05 + 2·residual`. Clean ranges get σ ≈ 0.18 m against a real ≈ 0.005 m. | Solvers under-weight good ranges; in simulation the barometer prior then dominates z (clean `--height baro` p90 0.38 m vs 0.02 m free). |
| DEF-8 | sim | Channel model: one reflector, no amplitude noise, never a blocked direct path. The tracking sim assumes perfect still/moving detection and round-robin node choice. | Simulated numbers are directional only. |
| DEF-9 | solver | The tracker has only a constant-velocity model; one range/s or less while walking loses track. | Walking needs ~2 ranges/s unless optional dead reckoning (FR-T7b) is added. |
| DEF-10 | proto | No telemetry envelope (age, hops). Records carry only origin uptime. | The sink cannot timestamp forwarded records yet (FR-S4). |
| DEF-11 | firmware | The node only advertises a name: no FR-M4 payload, no scanning or central role, no mesh logic. | Skeleton only. |
| DEF-14 | firmware | The bring-up role and peer are build-time environment variables (`NAVGRID_ROLE`, `NAVGRID_PEER`), and pairing is Just Works with no bonding, so every reconnect pairs again. | Bench only. Provisioning (DEF-5) and fleet-key LTKs (D-7) replace both. |
| DEF-21 | host | `navgrid cs` maps tones to antenna paths only for one path: with more, the step's antenna permutation index orders the tones, and the permutation tables aren't implemented. Procedures are keyed by counter alone, so a capture spanning a reconnect merges unrelated procedures, and mode-1 RTT values are paired in step order. | Diversity captures give no distance yet. Capture one connection per file pair. |
| DEF-20 | deps | TrouBLE `unwrap!`s every LE meta event, so one bt-hci can't parse (e.g. CS capabilities complete v2, subevent 0x38) panics the node. | Don't unmask newer LE events until bt-hci parses them. |
| DEF-22 | deps | embassy-nrf's GRTC time driver hard-codes CC[11], one of the channels nrf-mpsl hands to MPSL (CC[7]–[11]). Moving it to CC[1] didn't change DEF-18, and MPSL wasn't seen using CC[11]. | Latent: MPSL timing could break if it ever uses CC[11]. |
| DEF-23 | deps | nrf-mpsl's nRF54L critical section builds its reserved-interrupt mask with `=` per NVIC word; TIMER10 (IRQ 133) overwrites RADIO_0 (IRQ 138) in word 4, so every critical section disables the radio interrupt. Not DEF-18's cause. | Radio interrupt latency during app critical sections. Fixed on fork branch `fix-cs-reserved-irqs` (`36c49bc`), not yet pinned. |
| DEF-12 | firmware | Building without a board feature prints the intended `compile_error!` plus a follow-on "unresolved import `consts`". | Cosmetic. |

## Fixed

| ID | Defect | Fix |
|---|---|---|
| DEF-13 | bt-hci 0.11 parsed CS step data column-major; controllers send it row-major. | Fork `DmitriyNE/bt-hci` `2d8d65e`, used by the firmware since the switch to the forks. |
| DEF-15 | Nothing parsed the `CSR`/`CSC` capture lines. | `navgrid cs` (host `capture.rs`) with step decoding in `navgrid-ranging::steps`; checked end to end on simulated captures. |
| DEF-16 | probe-rs 0.32 double-buffered flashing of a TAG (DK onboard J-Link) replaced one word near the start of most pages with its successor; the corrupted images faulted at random places. | Runner passes `--disable-double-buffering` (and `--verify`). Not yet reported upstream. |
| DEF-17 | On the nRF54L15, the SDC hit internal assert `50:701` when a legacy `LE Create Connection` found its peer. The assert's meaning is unknown (Nordic internal); OpenBikeComputer #736 saw the same. | The initiator uses `connect_ext` with `support_ext_scan` / `support_ext_central`. |
| DEF-18 | Every CS subevent on both sides ended aborted with reason 0x3 (scheduling conflict or limited resources), with no steps. Cause: embassy-nrf leaves the nRF54L instruction cache off, and the SDC's per-step CS work can't keep up running from RRAM. | `board::early_init` enables `ICACHE`. 255/255 procedures ranged, no aborts ([spike](spikes/2026-11-02-cs-subevent-aborts.md)). |
| DEF-19 | While CS ran, the DK's J-Link lost the TAG: SWD protocol errors, USB transport errors, then re-enumeration or a hang. Not supply sag: it happened on DK power at 3.3 V too. | Runner runs SWD at 100 kHz (probe-rs default 400 kHz). No dropout in about 20 minutes of CS on two probes; at 400 kHz, one after 8 s ([spike](spikes/2026-11-02-cs-subevent-aborts.md)). |
