# Known defects and limitations

What is wrong or unverified in what exists **now**. Planned-but-missing features live in
[todo.md](todo.md). Conventions: [README](README.md).

## Open

| ID | Area | Defect | Impact |
|---|---|---|---|
| DEF-1 | firmware | The firmware does not range yet: no connection, security or CS procedure flow. The upstream gaps that blocked it (P-1, P-2, P-6) are fixed in our forks. | No CS on hardware until the bring-up flow lands. |
| DEF-2 | firmware | Nothing has run on a board. Unverified: the DC/DC enable, the crystal load-cap values (ported from Zephyr, FICR-trimmed), and the TAG antenna-switch callback. | First boot may hang on clocks or misbehave on RF. Check the boot log and an RF sanity test first. |
| DEF-3 | firmware | Antenna 0 is selected outside CS by driving SKY13348 V1 high. NCS leaves both lines low, and the switch truth table has not been checked. | Ordinary BLE traffic could go through the wrong or an isolated path. |
| DEF-4 | firmware | `SDC_MEM` (112 KiB) and `CS_LINKS` (6) are guesses. | Boot fails with "Memory buffer too small" and the needed size; set `SDC_MEM` to it. |
| DEF-5 | firmware | `NodeConfig` is a compiled-in default. There is no flash storage or provisioning, so every device boots as a Node with diversity on. | All devices behave the same. |
| DEF-6 | ranging | The phase-slope sign convention, and RTT units (0.5 ns) and sign, come from reading the spec. Not checked against hardware or `cs_de`. | Ranges could be mirrored or scaled until P0 cross-checks. |
| DEF-7 | ranging | Placeholder range σ model `0.05 + 2·residual`. Clean ranges get σ ≈ 0.18 m against a real ≈ 0.005 m. | Solvers under-weight good ranges; in simulation the barometer prior then dominates z (clean `--height baro` p90 0.38 m vs 0.02 m free). |
| DEF-8 | sim | Channel model: one reflector, no amplitude noise, never a blocked direct path. The tracking sim assumes perfect still/moving detection and round-robin node choice. | Simulated numbers are directional only. |
| DEF-9 | solver | The tracker has only a constant-velocity model; one range/s or less while walking loses track. | Walking needs ~2 ranges/s unless optional dead reckoning (FR-T7b) is added. |
| DEF-10 | proto | No telemetry envelope (age, hops). Records carry only origin uptime. | The sink cannot timestamp forwarded records yet (FR-S4). |
| DEF-11 | firmware | The node only advertises a name: no FR-M4 payload, no scanning or central role, no mesh logic. | Skeleton only. |
| DEF-12 | firmware | Building without a board feature prints the intended `compile_error!` plus a follow-on "unresolved import `consts`". | Cosmetic. |

## Fixed

| ID | Defect | Fix |
|---|---|---|
| DEF-13 | bt-hci 0.11 parsed CS step data column-major; controllers send it row-major. | Fork `DmitriyNE/bt-hci` `2d8d65e`, used by the firmware since the switch to the forks. |
