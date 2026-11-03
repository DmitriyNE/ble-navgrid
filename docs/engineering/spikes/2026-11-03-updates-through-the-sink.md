# Updates from the host through the sink

**Question.** Can the host put an update into the mesh through the sink, without a probe on any
node?
- Does the upload to the sink over VCOM1 arrive intact?
- Does the sink's transfer to a node over the sink link complete, and does the node then pass it
  on over its CS link as before?

## Setup

- **Boards:**
  - **The sink:** the DK (N003, `c00cc5545178`) on its own probe (…0413).
  - **N001** (`ce36dfcc4c2d`, initiator): on the other DK's probe (…4420), on DK power.
  - **N002** (`df96e04a5900`, reflector): on battery.
  - In the first runs N001 sat on …0413, which then reached N001 instead of the sink's chip.
- **The path** (`update.rs`, `sign.rs`):
  - `navgrid update FILE --sink PORT` sends 4 KiB `Data` messages with a CRC-32 each, one after
    each of the sink's answers.
  - The sink checks the signature and stages a TAG update as one it carries, not one it installs.
  - It offers the update to the node on its link, and again whenever it starts carrying another.
  - N001 listens for update channels on its sink link as well as its CS link.
- **Images:** A is `0.1.0`, B `0.1.1` and B2 `0.1.2`, all for the TAG board.

## How to reproduce

```sh
cargo run -p navgrid -- sign b.elf --out b.nup
cargo run -p navgrid -- flash dk.elf --probe <sink probe>
cargo run -p navgrid -- flash a.elf --probe <N001 probe>
probe-rs attach --chip nRF54L15 --probe <sink probe> --speed 1000 dk.elf > sink.log &
probe-rs attach --chip nRF54L15 --probe <N001 probe> --speed 100 a.elf > n001.log &
cargo run -p navgrid -- update b.nup --sink <sink VCOM>
```

`navgrid flash` blanks the staging page, so a reflashed sink has to be sent the update again.
TrouBLE's debug log (`DEFMT_LOG="info,trouble_host=debug"`, `DEFMT_RTT_BUFFER_SIZE=16384`)
showed nothing about the failures below: it logs neither incoming L2CAP signals nor sends that
wait for the link.

## Results

### Host to sink

| Upload | Result |
|---|---|
| 1, 2 | 64–94 s, then the signature didn't match. The sink checked it before writing the last, partial page. |
| 3–6 | 52–77 s, checked in 0.93 s, stored. About 36 UART framing errors per upload; the CRC dropped each damaged frame and the host resent it after 1 s. Keeping the crystal on made no difference. |

### Sink to N001

| Run | N001's firmware | Result |
|---|---|---|
| 1 | A, battery | The sink sent everything and waited for N001's `Done` in vain. N001 had written every full page but not the last, partial one, and nothing to the staging page. Ranging went on. |
| 2 | A2: writes time out after 10 s | N001 stopped receiving after 448 KiB, with no write timing out. N001's log showed no channel error and no disconnect: N001 was waiting on chunks. |
| 3, 4 | A3, then with TrouBLE's debug log | The sink offered after about 90 s on the link. N001 never answered the request for the channel; the sink's L2CAP timed out after 30 s. Neither side logged anything else. |
| 5 | A5: the telemetry sender yields after each frame | Offered after 140 s on the link: N001 accepted, took 507,512 bytes in 273 s (1.86 KB/s), and answered `Done`. It then checked, swapped and confirmed B2: the tag read `0.1.2`, the state `0xD0`. |

In every run N001's CS ran at 11.1/s with no empty procedures, and no page write took over
500 ms.

### N001 to N002

On B2, N001 passed it to N002 over the CS link in about 70 s. N001's ranging then stopped for
34.6 s, the gap of a restart, check and swap. On its next offer N002 declined B2. A node does
that for its own build, and N002 has no probe to read its tag. It also declines while its image
is unconfirmed, but it confirms on its first fleet link, and ranging had resumed.

## Conclusion

- **The cause of the sink-to-node failures was starved sends on N001's sink link.** N001's
  telemetry sender and its update receiver are futures in one task. They share TrouBLE's per-link
  send lock, which goes to whichever asks first. The sink link carries at most 8.3 uplink
  frames/s (DEF-42), so N001's outbox is always full. The telemetry future therefore took the
  lock again each time it released it, and the update channel's sends got none:
  - its accept, in runs 3 and 4;
  - its returned credits, later in runs 1 and 2, once the backlog had built up.
- **The TAG-to-TAG transfer never hit this:** the receiver sends nothing else on that link.
- **The fix:** the telemetry sender yields after each frame. The backlog itself (DEF-42) is still
  there. Also worth raising with TrouBLE: a lock that one sender can take again and again.
- **Throughput from the sink:** 1.86 KB/s, one 224-byte chunk per 120 ms event, so 4.5 min for a
  500 KB image. The host's upload, about 70 s, isn't the bottleneck.
- **A receiver must not restart before its `Done` has left.** It now waits for the sender to
  close the channel, and on the 120 ms sink link that `Done` arrived.

## Follow-up: N002's build

With N002 back on a probe, its tag reads `navgrid-firmware 0.1.2+51eed786-dirty` and its
bootloader state `0xD0`: it runs B2, confirmed, received from N001 without a probe.
