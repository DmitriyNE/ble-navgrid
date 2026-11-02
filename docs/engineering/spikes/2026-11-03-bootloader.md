# embassy-boot on the TAG

**Question.** Can embassy-boot be the bootloader (D-34), with the firmware under it?
- Does the firmware run under it, MPSL and the SoftDevice Controller included?
- Do a swap to a new image, confirmation, persistence and a revert of a bad image all work on the
  nRF54L15's RRAM?

## Setup

- **Boards:** TAG N002 (`df96e04a5900`) on DK `001057774420`, the reflector. TAG N001
  (`ce36dfcc4c2d`) ranges with it as initiator, and the DK sink records N001's ranges.
- **Bootloader:** `firmware/bootloader`, embassy-boot-nrf 0.12 at the Embassy revision the node
  pins, in its own workspace (it brings cortex-m's critical section, the node nrf-mpsl's).
  - It uses the RRAM driver as `NorFlash` and starts watchdog `WDT0` (WDT31) with 5 s, paused
    while a debugger halts the core.
  - It is 5.4 KB in release.
- **Firmware:** `firmware.rs`:
  - takes over the watchdog and feeds it every second;
  - reads the bootloader state;
  - confirms a swapped-in image (`mark_booted`) once a fleet link is up, and restarts if that
    doesn't happen within 5 min.
  - The image links at `0xB000`.
- **Updates by hand:** the probe writes an image's raw binary (`llvm-objcopy -O binary`) into the
  update slot, then the state region: 16 bytes of `0xF0` (swap) followed by blanks.

## How to reproduce

```sh
cd firmware/bootloader && cargo build --release        # --features defmt logs over RTT
cd .. && cargo build --release --features board-tag
cd .. && cargo run -p navgrid -- flash firmware/target/thumbv8m.main-none-eabihf/release/navgrid-node \
    --bootloader firmware/bootloader/target/thumbv8m.main-none-eabihf/release/navgrid-bootloader --probe <P>
# an update by hand: B's binary into the update slot, then the swap marker
probe-rs download --chip nRF54L15 --probe <P> --speed 100 --disable-double-buffering --binary-format bin --base-address 0xBF000 b.bin
probe-rs download --chip nRF54L15 --probe <P> --speed 100 --disable-double-buffering --binary-format bin --base-address 0x8000 state-swap.bin
probe-rs reset --chip nRF54L15 --probe <P>
```

## Results

| Step | Result |
|---|---|
| Bootloader with an 8 KiB state region | Panicked: embassy-boot asserts `2 + 4 × pages` 16-byte state lines (722 for a 720 KiB slot, 11.3 KiB). With 12 KiB it runs; layout below. |
| A (0.1.0) under the bootloader | Boots at `0xB000`, takes over the watchdog, encrypts a fleet link and runs CS. No reset over several minutes. |
| Swap to B (0.1.1): B in the update slot, swap marker, reset | B boots and logs "new image, unconfirmed". Its fleet link encrypts at 0.3 s, and it confirms at 1.15 s; the confirm write takes 75 ms. |
| Swap time | N001's ranging stopped for 33.2 s across the reset. Less the 4 s supervision timeout and about 3 s to reconnect and start CS, the swap of the whole 720 KiB slot took about 25 s. |
| After the swap | N001 ranged at 11.1/s with N002 on B. |
| Reset | B again, with no swap and no message: the confirmed state holds. |
| Swap to C (0.1.2), which panics at boot | With the probe attached, panic-probe's breakpoint halted C, and the watchdog, paused on debug halt, never fired. Once the probe detached, the watchdog reset it. The bootloader swapped B back (state `0xC0`, revert), and B boots logging "reverted to the previous image". |

Probe sessions around the revert added several boots to N002's counter: attaching and reading may
reset the target.

## Conclusion

- **embassy-boot works as our bootloader** (D-34 accepted). It is 5.4 KB, the firmware runs under
  it unchanged at `0xB000`, MPSL and the SDC work with the moved vector table, and swap,
  confirmation, persistence and revert behave as designed.
- **The layout,** now in both `memory.x` files and `navgrid_proto::layout`:

  | Region | Start | Size |
  |---|---|---|
  | bootloader | `0x000000` | 32 KiB |
  | state | `0x008000` | 12 KiB |
  | active slot | `0x00B000` | 720 KiB (the image is 415 KiB) |
  | update slot | `0x0BF000` | 724 KiB |
  | spare | `0x174000` | 28 KiB |
  | provisioning, boot epoch | `0x17B000` | 4 + 4 KiB |

- **Bench flashing:** `navgrid flash --bootloader` writes the bootloader once per device.
  Every `navgrid flash` blanks the state region, so nothing pending in the update slot gets
  swapped over a fresh image.
- **Swapping the whole slot takes about 25 s.** Two ways to shorten it: a slot closer to the
  image size (512 KiB would leave 23 % headroom), or the RRAM driver's buffered write mode.
- **Not tested yet:**
  - RRAM writes during active CS. The confirm ran before CS started, but an update over BLE will
    write the update slot while links run (NCS puts these writes in MPSL timeslots).
  - The 5-minute confirm timeout.
  - Signed images (step 2 of the plan).
- **A debugger holding a crashed image** keeps it from reverting, because the watchdog pauses on
  debug halt. Without a debugger it reverts.
