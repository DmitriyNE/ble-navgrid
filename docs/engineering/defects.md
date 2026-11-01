# Known defects and limitations

What is wrong or unverified in what exists **now**. Planned-but-missing features live in
[todo.md](todo.md). Conventions: [README](README.md).

## Open

| ID | Area | Defect | Impact |
|---|---|---|---|
| DEF-1 | firmware | The CS bring-up flow is only partly proven on hardware. Two TAGs connect (extended create connection) and pair, but the initiator's `LE CS Read Remote Supported Capabilities` gets no completion event within 5 s, so config, security and procedures haven't run. The reflector's side hasn't been logged yet. | No CS data yet. |
| DEF-2 | firmware | The node boots on a TAG (crystals start, DC/DC on), but the DK boot, the exact crystal load-cap values (ported from Zephyr, FICR-trimmed) and the TAG antenna-switch callback are unverified. | Frequency offset or RF misbehaviour would show only in an RF test or a CS run. |
| DEF-3 | firmware | Antenna 0 is selected outside CS by driving SKY13348 V1 high. NCS leaves both lines low, and the switch truth table has not been checked. | Ordinary BLE traffic could go through the wrong or an isolated path. |
| DEF-4 | firmware | `CS_LINKS` (6) is a guess. `SDC_MEM` is measured on a TAG (93,192 bytes); the one-antenna figure (74,760) is derived, not measured. | A wrong size fails boot with "Memory buffer too small" and the needed size, or warns "too big". |
| DEF-5 | firmware | `NodeConfig` is a compiled-in default. There is no flash storage or provisioning, so every device boots as a Node with diversity on. | All devices behave the same. |
| DEF-6 | ranging | The phase-slope sign convention, and RTT units (0.5 ns) and sign, come from reading the spec. Not checked against hardware or `cs_de`. | Ranges could be mirrored or scaled until P0 cross-checks. |
| DEF-7 | ranging | Placeholder range σ model `0.05 + 2·residual`. Clean ranges get σ ≈ 0.18 m against a real ≈ 0.005 m. | Solvers under-weight good ranges; in simulation the barometer prior then dominates z (clean `--height baro` p90 0.38 m vs 0.02 m free). |
| DEF-8 | sim | Channel model: one reflector, no amplitude noise, never a blocked direct path. The tracking sim assumes perfect still/moving detection and round-robin node choice. | Simulated numbers are directional only. |
| DEF-9 | solver | The tracker has only a constant-velocity model; one range/s or less while walking loses track. | Walking needs ~2 ranges/s unless optional dead reckoning (FR-T7b) is added. |
| DEF-10 | proto | No telemetry envelope (age, hops). Records carry only origin uptime. | The sink cannot timestamp forwarded records yet (FR-S4). |
| DEF-11 | firmware | The node only advertises a name: no FR-M4 payload, no scanning or central role, no mesh logic. | Skeleton only. |
| DEF-14 | firmware | The bring-up role and peer are build-time environment variables (`NAVGRID_ROLE`, `NAVGRID_PEER`), and pairing is Just Works with no bonding, so every reconnect pairs again. | Bench only. Provisioning (DEF-5) and fleet-key LTKs (D-7) replace both. |
| DEF-15 | host | Nothing parses the `CSR`/`CSC` capture lines yet, so raw steps cannot be turned into distances. | Next host task. |
| DEF-12 | firmware | Building without a board feature prints the intended `compile_error!` plus a follow-on "unresolved import `consts`". | Cosmetic. |

## Fixed

| ID | Defect | Fix |
|---|---|---|
| DEF-13 | bt-hci 0.11 parsed CS step data column-major; controllers send it row-major. | Fork `DmitriyNE/bt-hci` `2d8d65e`, used by the firmware since the switch to the forks. |
| DEF-16 | probe-rs 0.32 double-buffered flashing of a TAG (DK onboard J-Link) replaced one word near the start of most pages with its successor; the corrupted images faulted at random places. | Runner passes `--disable-double-buffering` (and `--verify`). Not yet reported upstream. |
| DEF-17 | On the nRF54L15, the SDC hit internal assert `50:701` when a legacy `LE Create Connection` found its peer. The assert's meaning is unknown (Nordic internal); OpenBikeComputer #736 saw the same. | The initiator uses `connect_ext` with `support_ext_scan` / `support_ext_central`. |
