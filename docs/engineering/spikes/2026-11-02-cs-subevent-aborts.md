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

## Continued: Nordic's samples, then the cause

Same setup, same day. NCS v3.4.1 installed with `nrfutil sdk-manager`.

**Nordic's samples work on these TAGs.** `ras_initiator` and `ras_reflector`, built for
`nrf54l15tag/nrf54l15/cpuapp` with `-S rtt-console`, ranged with no aborts. They logged 374
estimates in about 2.5 minutes: IFFT 2.16–2.18 m, phase slope about 2.47 m. So the boards and the
radio are fine, and the fault is in our stack. Note that the sample's default is mode 2 with sub
mode 1, not plain mode 2.

| Run | Change | Procedures (R / I) | Result |
|---|---|---|---|
| 9 | Sub mode 1, as Nordic's default | 104 / 104 | All aborted. |
| 10 | SDC and MPSL libraries from NCS v3.4.1 (fork copy) | 100 / 100 | All aborted. |
| 11 | Nordic's timing: 20 ms ACL, procedure every 5 events, 80 ms maximum | 555 / 555 | All aborted. |
| 12 | Nordic's reflector with our initiator | – / 852 | All of our initiator's aborted: the fault is on our initiator's side. |
| 13 | Central ACL event spacing 90 ms (NCS's default when CS is on) | 515 / 515 | All aborted. |
| 14 | HCI byte trace on both stacks (below) | 443 / 443 | All aborted; the CS commands match. |
| 15 | NCS's controller setup: 1/1/1 links, privacy support | 383 / 383 | All aborted. |
| 16 | MPSL HFXO start-up latency 1650 µs (the DT value NCS passes) | 451 / 451 | All aborted. |
| 17 | RRAM standby in the MPSL low-latency hooks, as NCS | 462 / 461 | All aborted. |
| 18 | nrf-mpsl critical section reserving RADIO_0 (DEF-23) | 416 / 416 | All aborted. |
| 19 | **Instruction cache enabled** (with runs 9–18 still in) | 462 / 462 | **Steps on both sides.** 161 distances; the other procedures aborted, reason unspecified or no CS_SYNC. |
| 20 | Committed firmware (`34ae288`) plus only the cache | 108 / 108 | **No aborts**, 108 distances, median 2.281 m (p10–p90 2.267–2.327 m). |
| 21 | Run 20 plus the full config-complete log (committed) | 255 / 255 | **No aborts**, 255 distances, median 2.342 m (p10–p90 2.282–2.386 m). |

**HCI comparison (run 14).** NCS's SDC driver was patched for the run to print every command and
event; nrf-sdc printed each command's parameter bytes and both struct sizes. The CS commands were
byte-identical (Set Default Settings `0000 01 fe 14`, Create Config, Set Procedure Params
`… 8000 0500 0500 0000 803e00 803e00 00 02 80 01 ff ff`, Procedure Enable), the Rust and C struct
sizes matched, and both controllers reported the same config (T_IP1 30, T_IP2 20, T_FCS 60,
T_PM 10 µs). Only setup differed: event masks, host buffer size, default data length.

## Conclusion

**Cause:** embassy-nrf doesn't enable the nRF54L instruction cache; Zephyr's SoC init does
(`soc/nordic/nrf54l/soc.c`). Without it the CPU runs from RRAM, and the SDC can't keep up with
its per-step CS work, so it aborts every subevent with 0x3. Ordinary connections need far less
real-time CPU and worked throughout.

**Fix:** `board::early_init` sets `ICACHE.ENABLE`. Nothing else from runs 9–18 is needed (run 20).

**Not needed but noted:** NCS also passes MPSL the HFXO start-up time, keeps the RRAM in standby
during radio activity, and spaces central ACL events by 90 ms when CS is on. Run 19's aborts came
from its experimental settings (20 ms, procedure every 5 events, sub mode 1, heavy logging); they
are not explained yet.

**Possibly related:** the SDC assert 50:701 on a legacy `LE Create Connection` (DEF-17) is also
a timing failure inside the controller, and NCS's sample uses that command without trouble.
Untested with the cache on.

## Distance check

Final firmware (run 21's build) against Nordic's samples on the same pair, at two hand-measured
separations. Each cell is a median over 1–4 minutes.

| Separation | Ours (phase slope) | Nordic IFFT | Nordic phase slope |
|---|---|---|---|
| ~10 cm | 2.34 m (p10–p90 2.28–2.39) | 2.17 m | 2.47 m |
| ~90 cm | 3.34 m (p10–p90 3.27–3.40) | 2.93 m (2.93–3.22) | 3.56 m (3.42–3.62) |

- **Sign and scale are right:** moving about 0.8 m changed ours by 1.0 m.
- **Fixed offset:** ours reads about 2.4 m long. Nordic's samples show a similar offset (their
  `BOARD_DISTANCE_OFFSET` is 0 on the TAG). Ours stays within 0.15–0.2 m of Nordic's phase slope.
- **Not settled:** a calibration needs measured distances. With these hand estimates, offset and
  close-range multipath can't be separated.
