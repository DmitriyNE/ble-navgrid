# CS subevent aborts (DEF-18)

**Question.** Why does every CS subevent end aborted, on both TAGs, with subevent abort reason
`0x3` (scheduling conflict or limited resources) and no steps?

## Setup

- **Reflector:** TAG `df96e04a5900` on DK `001057770413`.
- **Initiator:** TAG `ce36dfcc4c2d` on DK `001057774420`.
- **Power:** both TAGs without cells, powered from their DK at 3.3 V (`VDD:IO` jumpered to
  `VDD SWD0`).
- **Logging:** both sides logged at once over RTT, SWD at 100 kHz.
- **Baseline:** firmware at `e4c01b7`: mode 2, 72 channels, 1×1 antenna configuration, 2M,
  16 ms subevents, a procedure every 10 connection events, 50 ms ACL, CS event length 5 ms.

## How to reproduce

```sh
cd firmware
cargo build --release --features board-tag                       # reflector image
NAVGRID_ROLE=initiator NAVGRID_PEER=df96e04a5900 cargo build --release --features board-tag
probe-rs run --chip nRF54L15 --probe <DK> --speed 100 --disable-double-buffering --verify <elf>
```

Each run flashed both boards, then logged 1–4 minutes of CS. "Aborted" means `CSR` lines with
subevent done status `0xF`, abort `0x30` and 0 steps.

## Results

Each run adds its change to the ones above it, except where noted.

| Run | Change | Procedures (R / I) | Result |
|---|---|---|---|
| 1 | Baseline (SWD 400 kHz) | 10 / 74 | All aborted. The reflector's probe lost the TAG at 8 s (DEF-19). |
| 2 | Data length 251 B / 2120 µs (negotiated both ways); SWD 100 kHz | 445 / 445 | All aborted. No probe dropouts. |
| 3 | CS event length 16 ms, equal to the subevent length as the SDC docs say | 128 / 128 | All aborted. |
| 4 | ACL interval 100 ms | 83 / 83 | All aborted. |
| 5 | Back to 50 ms. NCS's controller setup: one CS role per build, no mode-3, one CS link, T_PM 10 µs | 129 / 129 | All aborted. |
| 6 | One antenna (one path), so the SDC never calls the antenna switch | 112 / 111 | All aborted. |
| 7 | HFXO left to MPSL (embassy `HfclkSource::Internal`), as in NCS | 109 / 109 | All aborted. |
| 8 | Runs 5–7 reverted except T_PM; embassy's GRTC time driver moved from CC[11] to CC[1] | 110 / 110 | All aborted. |

In every run, each procedure produced exactly one `CSR` line: one aborted subevent with procedure
done status `0x0`. No mode-0 results either; the initiator's frequency compensation stayed 0.

## Also ruled out

- **At a 20 ms interval (log, 2026-11-02):** SDC support for PHY update, DLE and LE power
  control; a 2.5 ms ACL event length; 90 ms central spacing; a 2M ACL; 5 ms subevents; 1/1/1
  links.
- **By inspection:**
  - every CS command parameter equals NCS v3.4.1 `ras_initiator`;
  - the CPU runs at 128 MHz, as in Zephyr;
  - the LFCLK is the crystal at 50 ppm;
  - nrf-mpsl implements the nRF54L low-latency callbacks (`POWER.TASKS_CONSTLAT`);
  - the SDC is NCS v3.4.0's. v3.4.1 changes only `LL_CS_REQ` interval validation and fixes for
    IPT and disconnect asserts.

## Conclusion so far

None of these settings is the cause: interval, data length, CS event length, tone length, roles,
mode-3, link count, antenna switching, HFXO ownership, or GRTC CC[11] sharing. What still differs
from NCS is the host (TrouBLE instead of Zephyr) and the Rust MPSL/SDC integration.

**Next:** run Nordic's `ras_initiator` / `ras_reflector` (NCS v3.4.1, board
`nrf54l15tag/nrf54l15/cpuapp`) on the same two TAGs.
- If they range, the fault is in our stack, and we compare HCI traffic.
- If they abort too, it is the SDC or the boards.

## Side findings

- **DEF-19:** probe dropouts happen on DK power too, so supply droop isn't the cause. At 100 kHz
  SWD no probe dropped in about 20 minutes of CS, across both probes (runs 2–8); at 400 kHz one
  dropped after 8 s.
- **DEF-22:** embassy-nrf's GRTC time driver uses CC[11], which nrf-mpsl hands to MPSL (CC[7]–[11]).
- **Board controller:** DK `001057774420`'s board controller timed out for both nrfutil and Board
  Configurator until its USB was replugged.
- **Reading a DK's VDD:** without the GUI, `nrfutil device x-execute-batch` with SMP group 64,
  command 0 returns the board-controller config. `[1, 3300]` means VDD is 3300 mV.
