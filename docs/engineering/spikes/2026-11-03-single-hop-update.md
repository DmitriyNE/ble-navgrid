# Single-hop update over a CS link

**Question.** Can a node pass its installed update to a neighbour over their fleet link (D-36)?
- Does the image arrive intact, and do the check, swap and confirm then run as with a probe-staged
  update?
- What does writing the update slot cost the ranging on the same link (DEF-38)?

## Setup

- **Boards:** TAG N001 (`ce36dfcc4c2d`, initiator) ranges with TAG N002 (`df96e04a5900`,
  reflector) and reports to the DK sink, which the monitor reads. Both run under the bootloader on
  DK power.
- **Transfer** (`firmware/node/src/update.rs`): on SPSM `0x0083`, N001 offers the update it was
  installed from (its header is kept at `0x179000`). N002 asks for it from byte 0 and N001 streams
  224-byte chunks. N002 collects 4 KiB pages, writes them into the update slot and the header into
  the staging page, then restarts into the check at boot.
- **Images:** A is `0.1.0`, B the same source as `0.1.1`, signed with the fleet's signing key.
  Each run flashes A onto both TAGs, then stages B on N001 through the probe.

## How to reproduce

```sh
# A, and B as 0.1.1 (bump `version` in firmware/Cargo.toml for the second build)
cargo run -p navgrid -- sign b.elf --out b.nup
cargo run -p navgrid -- flash a.elf --probe <N001 probe> --probe <N002 probe>
cargo run -p navgrid -- monitor <sink VCOM> --json > monitor.jsonl &
probe-rs attach --chip nRF54L15 --probe <N002 probe> --speed 100 a.elf > n002.log &
cargo run -p navgrid -- update b.nup --probe <N001 probe>
```

To see which build a TAG runs, read its firmware tag rather than trusting a log decoded with the
wrong ELF: run 1's log of B, decoded with A's ELF, looked like A. An attached probe-rs reports a
restart into the bootloader as an exception at `__stext` (`0x7c0`): that is the reset, not a
fault.

## Results

| Run | RRAM writes | Result |
|---|---|---|
| 1 | Blocking, from the receive loop | 491,032 bytes in 67.3 s (7.3 KB/s). N002 restarted, verified, swapped and confirmed B; both TAGs then carried the 0.1.1 tag and the confirmed state (`0xD0`). N001 ranged at 10.4/s during the transfer, 53 procedures missing in 67 s, against 11.1/s with no transfer running (runs 3 and 4, after theirs). Empty procedures ("too few channels") came in steps of about 0.54 s, one 4 KiB page at the transfer rate. |
| 1, reset of N001 | – | N001 offered B again on reconnecting; N002, on B, declined, and ranging resumed at once. |
| 2 | Blocking | N002 hit SoftDevice Controller assert `131:2004` 0.73 s into the transfer, at its first page write. The transfer began 90 ms after CS procedures started. |
| 3 | MPSL timeslots (`nrf_mpsl::Flash`) | No assert, and N001 ranged at 11.2/s during the transfer with no procedure missing (11.1/s after). The data rate stayed at 7.4 KB/s. But N002 lost the last 128-byte chunk: N001 closed the channel right after `Done`, which dropped what N002 hadn't read yet. |
| 4 | Timeslots, and the receiver acknowledges `Done` | 500,720 bytes in 67.3 s. N002 answered `Done`, then restarted, verified, swapped and confirmed B (`0xD0`, the 0.1.1 tag), keeping the header to pass on. N001 ranged at 11.1/s during the transfer with no procedure missing. Ranging stopped for 35.4 s while N002 checked, swapped and reconnected. |

## Conclusion

- **Unbuffered RRAM writes and the radio don't mix.** The nRF54L15 takes up to 65 µs per word, so
  a 4 KiB page keeps the RRAM busy for tens of milliseconds. Interrupts fetching code from it wait
  meanwhile. Run 1 lost 7 % of its procedures; run 2 crashed the controller. After boot all RRAM
  writes now go through `nrf_mpsl::Flash`, which writes a few lines per MPSL timeslot, as NCS
  does. The boot-time writes (epoch, factory reset, the staged check) still write directly,
  before MPSL starts.
- **The transfer runs at one chunk per connection event:** 224 bytes per 30 ms, about 7 KB/s, so
  a 490 KB image takes 70 s. The 2.5 ms ACL event length (DEF-26) may be what holds it to one;
  not checked.
- **The sender must wait for the receiver** before closing the channel.
- **A node that hasn't confirmed its image declines updates:** its update slot still holds the
  image a revert would restore.
