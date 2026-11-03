# Revert after an update over the air

**Question.** Does a bad image that arrives over the air put the previous one back by itself?
- Does the 5-minute confirm deadline restart it, and does the bootloader then revert?
- Does the bad image stay where it landed instead of spreading?
- Does it stay away once reverted, or come back at the next offer?

## Setup

- **Boards:**
  - **The sink** (N003), on USB.
  - **N001** (initiator), on probe …0413.
  - **N002** (reflector), on probe …4420.
  - Both TAGs flashed by probe with C5 (`0.1.0+dca10e40`).
- **The bad image R** (`0.1.6`) is C5 built with the `never-confirm` feature, a test aid that
  stops the image confirming itself.
- **Two policy changes in C5 and R** (`update.rs`, `firmware.rs`):
  - a node offers its installed image only once it has confirmed it;
  - after a revert, the kept header still names the build that failed, and the node declines
    offers of it.

## How to reproduce

```sh
cargo build --release --features board-tag,never-confirm   # with `version` bumped
cargo run -p navgrid -- sign r.elf --out r.nup
cargo run -p navgrid -- flash c5.elf --probe <N001 probe> --probe <N002 probe>
cargo run -p navgrid -- update r.nup --sink <sink VCOM>
```

To read a TAG's log across the swaps, wait for each swap to finish before choosing the ELF.
Read mid-swap, the firmware tag still showed the old build, and R's log came out undecoded.

## Results

| Step | Result |
|---|---|
| The sink offers R to N001 | Taken on the open link: 504,928 bytes in 340 s. N001 checked it, swapped and ran R. |
| R unconfirmed on N001 | R kept ranging with N002 and didn't offer itself: N002 still had C5 and a bootloader state never written (`0xff`). |
| The deadline | About 5 min after R started, N001 restarted, and the bootloader put C5 back. Afterwards N001 had C5's tag, no trace of R's, and state `0xD0` (confirmed). |
| The sink offers R again | `update: declining 0.1.6+dca10e40-dirty: this node reverted from it`. |

R's own log lines ("not offering…", "reverted from…") didn't decode (see above). The device
states and the decline show the same outcome.

## Conclusion

- **The revert works over the air:** an image that never confirms is gone about 5.5 min after it
  starts, with the previous image confirmed again and nothing else to do.
- **A bad image stays on the node it reached:** it is passed on only once confirmed. In the mesh,
  a node that can't get back into the mesh stops the update there.
- **A reverted build stays away:** the node declines it from then on. The kept header remembers
  only the last failed build; a newer good update replaces it.
- **The carrier doesn't learn that a build failed:** the sink kept offering R until the host gave
  it another update. The desired-firmware record and its rollout (stage 4) will have to carry
  that back.
