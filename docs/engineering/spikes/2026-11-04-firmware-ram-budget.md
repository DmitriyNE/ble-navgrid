# The firmware's RAM and flash budget after the mesh rewrite

**Question.** Does the mesh firmware (links, sessions, replica, updates) fit the nRF54L15's
256 KiB of RAM and the 720 KiB image slot? Nothing had linked since the mesh rewrite: clippy
passes without linking, and no build had been flashed (DEF-57).

**Answer.** Not as written: the TAG build was 125 KiB over RAM and 13 KiB over flash, the DK
141 KiB over. After the changes below both link with about 22 KiB of RAM between the statics and
the top of the stack and 200 KiB of flash to spare. The link count is 8 per node for now, not 14.

## Method

`RUSTC_BOOTSTRAP=1 cargo rustc --release --features board-tag -p navgrid-node -- -Zprint-type-sizes`
in `firmware/` prints every type's size, including the async state machines, on the stable
compiler; the linker's overflow message gives the total, and `llvm-size -A` the sections once it
links. (A fresh `--target-dir` fails on embassy-crypto's size check; use the normal one.)

## Where the RAM went (TAG, before)

| What | KiB | Why |
|---|---|---|
| The main task's future | 274 | holds everything below it |
| ├ 14 link tasks (`serve_link`) | 151 | 11 KiB each: the update receiver's 4 KiB page, a `Sync` message (860 B) in the session, another in `exchange`, buffers for offers and channels |
| ├ SDC memory | 97 | as the SDC asked for 6 central + 8 peripheral links and 6 CS configurations |
| ├ TrouBLE host resources | 20 | 14 connections, 42 channels |
| Mesh state (`STATE`) | 48 | the replica's 8 peers × 2 × 64 summaries of 24 B, the session driver's 4 summary lists, in `.data` (copied from flash, as `None` isn't zero) |
| Record tables | 25 | 160 records of 136 B as structs |
| Everything else | 30 | packet pool, CS burst state, logger, outboxes |

## What changed

| Change | Saved (TAG) |
|---|---|
| Records and the outbox postcard-encoded in byte arenas (DEF-29): 50 B a record against 136 B (host: 160 B); 16 KiB / 320 records and 6 KiB / 80 on a TAG, 24 KiB / 480 and 6 KiB / 80 on a DK | holds twice the records in the same RAM |
| The update page out of each link's future into one static (one transfer at a time), and 1 KiB rather than 4: RRAM needs no erase | 56 KiB |
| `Summary` packed to 11 B (kind byte, 48-bit node, hash) | 24 KiB |
| The mesh state in uninitialised RAM, not `.data` | 49 KiB of flash |
| Session frames in one static pair, messages decoded and encoded under the state lock rather than held in the link's future | 14 KiB |
| 8 links per node (6 central, 6 peripheral) rather than 14 | 44 KiB |
| `opt-level = "s"` | 182 KiB of flash (673 → 491 KiB) |

A link task is now 5 KiB. The link simulator at 500 anchors and 100 tags gives the same figures
at 8 links as at 14 (anchors hold 5.1 on average, every walk gets a range within a second,
1.76 ranges/s while moving).

`mesh-sim` now prints what the tables take encoded, as a device keeps them. At 500 nodes with
4 sinks a node's records and outbox peak at 5.6 KiB at p50 and 8.6 KiB at most, against the
TAG's 22 KiB of arenas; each sink's 920–1180 records and 140–170 in custody take 43–56 KiB,
twice a DK's arenas. At 20 nodes a node peaks at 4.2 KiB (5.5 max) and the sink at 7.9 KiB. The
device's old outbox of 32 entries refused 678 pushes at 500 nodes and left the host 85.7 %
current; 80 entries refuse 7, the same as without limits (86.6 %).

## Where it stands

| | TAG | DK |
|---|---|---|
| `.text` + `.rodata` + `.data` | 519 KiB | 523 KiB |
| `.bss` + `.data` + `.uninit` | 233 KiB | 235 KiB |
| Headroom for the stack | 23 KiB | 21 KiB |

The boot log prints the headroom (`ram: N B between the statics and the top of the stack`).
Of the 233 KiB: the main task 154 KiB (SDC 97, link tasks 40, host resources 12), mesh state 26,
record tables 24, the update page 1, the rest 28.

## On the DK: the stack, measured

The first flash of the mesh build reset itself before its first log line. `probe-rs run` showed
only `Exception` with frames at `0x7c0`, which is the bootloader's reset vector: probe-rs catches
resets by default, so that is what a chip in a reset loop looks like from the debugger. Boot
markers, then a stack painted in `__pre_init` and a stack limit register (`MSPLIM`), showed the
cause: the stack overflowed its 27 KiB and overwrote the radio stack's statics, which sit just
below it, and the SDC then jumped through a corrupted callback pointer (a UsageFault with the
program counter in RAM). Two things used the stack:

- **Init, 29 KiB** before the mesh starts (`embassy_nrf::init`, MPSL, the SDC's own
  initialisation, the replica). Still unexplained in detail; the boot line prints it.
- **The mesh's joined future.** `join4(drive, scan, advertise, serve_links)` with
  `serve_links` a `join_array` of eight link loops of 5 KiB each: a joined future is built by
  value on the stack before it is stored in the task, so the first poll needed 40 KiB. The mesh
  now runs as tasks (`Mesh::spawn`): the driver, the scanner, the advertiser and one task per link
  slot, each in its own static pool, with the BLE host, the mesh and everything they borrow in
  statics. Nothing larger than a link loop is ever built on the stack.

The SDC on the DK (one antenna, 6 + 6 links, 6 CS configurations) asked for 74,296 bytes against
the 80,640 given; the constant follows it. With the sink's tables at 8 KiB and 2 KiB for this
run, the DK boots with 56 KiB of headroom and 29 KiB used by the boot line, and the four TAGs on
the old build connect to it, open their telemetry channels, and are kept as hello-less peers.

## Left

- The TAG's SDC figure (two antennas, 6 + 6 links) comes from its first boot on the mesh build;
  the DK's is 74,296 bytes. The TAG's tables are halved until its stack and SDC figures are in.
- 14 links need another 30 KiB: offers and channels still hold their buffers per link (1.3 KiB
  and 1.1 KiB), and the session driver holds its class list as 16-byte classes (2.5 KiB).
- A DK sink holds about 500 records: a mesh of some 60 nodes (the sinks in the 500-node run
  hold 9 records a node). Beyond that the sink must stop keeping what it only forwards to the
  host (DEF-67).
- The replica remembers 64 summaries per peer for delta sync; a degree-6 node's neighbourhood
  has about 94 classes, so delta sync may be falling back to full exchanges there (DEF-66).
