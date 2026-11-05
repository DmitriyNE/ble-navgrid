# Handoff

Where the work stands when a session ends, so the next one (another machine, another session)
picks it up without the conversation. Overwritten at each handoff; the log keeps the history.

## 2026-11-05, evening: from the laptop to the work desktop

### Bring-up on the new machine

1. **The TrouBLE fork.** The firmware builds against a local clone of TrouBLE beside this
   repository (`firmware/Cargo.toml`, `[patch."https://github.com/GreatKet/trouble.git"]`,
   path `../../trouble/host`). It carries one commit, the DEF-71 fix, kept here as
   `firmware/patches/trouble-0001-runner-survives-one-links-errors.patch`. Recreate it:

   ```sh
   cd ..   # the directory that holds ble-navgrid
   git clone https://github.com/GreatKet/trouble.git trouble
   cd trouble && git checkout -b navgrid fa9886d3f92d0b9a8adf7323a9b10b991a131d2c
   git am ../ble-navgrid/firmware/patches/trouble-0001-runner-survives-one-links-errors.patch
   ```

   Publishing the fork (and pointing the patch at its git revision, and offering the fix
   upstream) is in the todo and needs the user's go-ahead.
2. **The fleet directory** `~/.config/navgrid` is not in git and must be copied by hand. It holds
   the fleet key and the update signing key (secrets: never commit them), the registry
   (`devices.json`, with barometer offsets and the routes-only flag), the survey history
   (`survey.jsonl`, `survey-start`), barometer readings (`baro.jsonl`), the approximate positions
   (`approx.json`), the plan's placement (`floorplan.json`), the mirror switch (`frame.json`),
   kept build ELFs (`builds/`, for decoding device logs) and labels.
3. **The floor plan** lives in `floorplans/` (the office's PDF and its rendered
   `xpanceo_301.png`), excluded from git through `.git/info/exclude`, not `.gitignore`. Copy the
   folder and add the same exclude line on the new machine. Never commit it.
4. **Hardware.** Two DKs: the sink N003 (J-Link `001057770413`) and a second DK whose own chip is
   dead, used only as a TAG adapter (J-Link `001057774420`). Serial port names change between
   machines: `ls /dev/cu.usbmodem*`; the sink's data port is the one ending in `3` of the pair
   for its J-Link. N007 (the mobile) sits in the dead DK's adapter, untouched: **read its RAM
   log before anything else** (recipe below), it is there to find out why it holds no links
   (DEF-76). Plugging a TAG into the adapter can reset it, so the log after a replug may only
   cover the time since.
5. **Run the page:**

   ```sh
   cargo build --release -p navgrid
   ./target/release/navgrid serve --sink /dev/cu.usbmodem<…>3 --solve-every-s 120 \
       --floorplan floorplans/xpanceo_301.png
   ```

   http://127.0.0.1:8080, `?mode=3d` for the 3D view.

### The fleet

| Node | Role | Notes |
|---|---|---|
| N001, N002, N004, N005, N006 | anchors (TAGs) | mounted around the office; barometers calibrated |
| N003 | sink (DK) | routes only: no barometer, its one antenna path reads up to 2.7 m apart burst to burst |
| N007 | mobile (TAG) | in the dead DK's adapter on the desk; barometer calibrated |

- **Firmware.** The sink runs 0.1.0+8a0e7a9b. The TAGs ran 8a0e7a9b at handoff, with the
  DEF-75 snapshot build (the same firmware, built from the clean tree at commit 2fcd6cd) uploaded
  to the sink and set as every TAG's wanted build; they take it over the air while the sink
  runs. An earlier attempt named 0.1.0+fee502a8 as wanted while the image was a dirty build
  (`-dirty` in its id: an uncommitted `Cargo.lock`), so nothing took it: **build images only
  from a clean tree**, and check the id with
  `strings ELF | grep -oE '0\.1\.0\+[0-9a-f]{8}(-dirty)?'`. N007 doesn't link (DEF-76), so it
  will need the wire: flash it in the adapter after reading its log.
- **Settings in force** (controller records in the mesh, re-applied after reboots): survey
  every 60 s on the anchors, Environment records every 60 s on the TAGs, N003 ranging off.
  Barometer defaults: a reading every 10 s, mean of 6, heights allowed.
- **Barometer offsets** (registry, side by side at 19:00–19:11): N001 +10.4, N002 −18.5, N004
  −10.4, N005 −3.0, N006 +1.5, N007 +18.7 Pa.
- **The survey** started afresh at 19:16 after the anchors went back (roughly) to their places.
  N001, N002 and N004 are placed approximately on the plan (±1 m), so the solve is in the plan's
  frame: rms about 2 with N004–N005 (reads ~4.5 m long) and N006–N002 (~2.9 m short) left out.
- **No measured pins yet.** Heights are barometric, relative to one anchor at 0.

### Open threads, in order

1. **DEF-76: the mobile holds no links.** N007 is taken to be moving (no accelerometer driver),
   so it should keep three links and range continuously; it sat with none for minutes, ranging
   in short spells. Read its log, then look at its connects (backoff after failures), its heard
   table, and whether the routes-only sink takes its candidates.
2. **Live tracking needs a fast mode** (the user, 2026-11-05): a mobile asks the anchors for a
   much faster burst rate while it is tracked, and gets out of it when it stops. Today a tag
   bursts 4 procedures at a time when its links rotate, and a burst reads ±1–3 m on its own; the
   page's tag fix (medians per anchor over the shortest window holding three anchors,
   `solve::fix_tags`) is only as good as that. Design first: who asks (the tag, or the host for
   a tag on the page), what the anchors give up for it, how long it lasts.
3. **DEF-75 evidence.** Once the TAGs run the snapshot build, a node that goes silent and is
   then reset reports its last state at WARN in its first log lines (`last run at … s:
   advertising stage …`). The leading suspect is link slots held by links already gone, which
   blocks both advertising and connecting.
4. **The ranging estimator under multipath** (DEF-51): pairs read metres long and short, steady
   burst to burst. The solve copes (one-sided loss, triangle screen, consensus); the estimate
   should not need it. First path, and the four antenna paths compared.
5. **Pins** for two or more anchors, with z, so heights are above the floor.
6. The rest of `todo.md`: publishing the TrouBLE fork, time-aligned barometry, still tags and
   updates, the survey's sigma model, the plan in 3D.

### Working agreements

These live in the assistant's per-machine memory, which doesn't travel; they are the user's.

- **Rust everywhere**, host and firmware; C SDKs (NCS, `/opt/nordic/ncs/v3.4.1`) as reference
  only.
- **Commit locally without asking**, in the repository's style (`CLAUDE.md`: terse, no prefixes,
  no trailers); **pushing needs the user's go-ahead**.
- **Bench requests first**, as concrete numbered actions; analysis after.
- **Software first:** code is cheap, bench time and hardware are not.
- **Fix causes, not symptoms:** no watchdog or restart bandaids for software faults; keep a
  wedged node as it is and read it (its log is in RAM; a reset loses it); a silently fatal state
  may become a loud one (panic with a crash record), never a masked one.
- **Never run probe traffic through the sink's J-Link while its radio is busy** (DEF-19): flash
  or read TAGs through the dead DK's adapter.
- **New record kinds go out only after every node runs a build that decodes them** (DEF-64).

### Recipes

**A TAG's RAM log through the adapter** (J-Link `001057774420`), decoded with the ELF of the
build it runs (`~/.config/navgrid/builds/`, or rebuild that commit):

```sh
E=~/.config/navgrid/builds/<build>-tag.elf; P=1366:1069:001057774420
NM=$(ls $(rustc --print sysroot)/lib/rustlib/*/bin/llvm-nm | head -1)
B=$($NM -S $E | grep -E "logger.*BUFFER" | awk '{print $1" "$2}')
BA=$(echo $B | awk '{print $1}'); BS=$((16#$(echo $B | awk '{print $2}')))
R=$($NM $E | awk '/ _SEGGER_RTT$/ {print $1}')
W=$(probe-rs read --chip nRF54L15 --probe $P --speed 100 b32 $((0x$R + 0x24)) 1 | tail -1 | awk '{print $2}')
probe-rs read --chip nRF54L15 --probe $P --speed 100 b8 0x$BA $BS \
  | awk '{for(i=2;i<=NF;i++) printf "%s", $i}' | xxd -r -p > ram.bin
python3 -c "b=open('ram.bin','rb').read(); w=int('0x$W',16); open('ram.rot','wb').write(b[w:]+b[:w])"
defmt-print -e $E < ram.rot
```

**Flash a TAG by wire**, recorded in the registry: `navgrid flash ELF --probe 1366:1069:001057774420`
(add `--bootloader firmware/bootloader/target/thumbv8m.main-none-eabihf/release/navgrid-bootloader`
and `navgrid provision --probe … --role …` first for a new puck).

**A rollout over the air:** build both boards from a clean tree, `navgrid sign ELF --out
tag.update`, stop the page (it holds the port), `navgrid update tag.update --sink PORT`, then
`navgrid control --sink PORT N00x firmware 0.1.0+<id>` per TAG (or `all`; it sends one record
at a time now), restart the page. The sink itself goes by probe: `navgrid flash dk.elf --probe
1366:1069:001057770413`.
