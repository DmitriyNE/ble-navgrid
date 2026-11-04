# Engineering log

Newest first. Conventions: [README](README.md).

## 2026-11-04

**Delta sync** (DEF-28 fixed). Replicas remember per peer the summaries of their last complete
session; the hello says so, and when both remember, each sends only the classes whose hash
changed (a class dropped goes as a zero hash) and the other rebuilds the full picture from its
memory. Either side without memory, or a session that broke off (the firmware forgets it), means
a full exchange. The in-memory session and the frame-driven driver both do it and still send the
same bytes. Simulator: sessions 650 → 115 B a side at p50 with 20 nodes, 125 B at 500 nodes and
4 sinks (summaries 30 %, versions 37 %); per node 74 kB/h at p50, 117 kB/h at the busiest relay;
peak node memory 20 KiB at p50, 34 KiB max; a sink at 500 nodes holds 1 000–1 200 records,
175–215 KiB, which the host holds for it.

**Handover, work counters, several sinks** (S4, first items). A walking tag with all its links
swaps the weakest for an anchor heard 6 dB stronger (`Settings::hysteresis_db`), and its connects
now come before its bursts, which never pause while it moves (a bug the simulator had hidden: a
tag that lost a link never made another). The mesh Health record counts bursts, procedures and
sessions since boot, for the energy accounting (NFR-6). `navgrid serve --sink` repeats for
several sinks: their records merge in the one replica and all of them get the controller's
records. The link simulator reads the same with the handover in (2.0 ranges/s over 1.9 anchors
at 3 × 90 ms, four paths).

**The host as a service** (S3). `navgrid serve --sink PORT` owns the sink's port: every frame
goes into the host replica, the monitor's lines and a JSON log the offline commands read; the
mesh is solved on a schedule (`--solve-every-s`) and the positions written back through the sink
and into the host's own view; `http://127.0.0.1:8080` serves one page (floor plan, nodes, links,
health, commands, a control form, the find pointer, pin placement, the recent frames) over a JSON
endpoint (`/api/state`, `/api/pointer`, `/api/solve`, `/api/control`, `/api/pin`), plain `std`
HTTP, one thread a connection. `--stream FILE` takes a FIFO or a file of raw frames instead of a
port, for replays and a bench without a sink; pins placed on the page persist in `pins.json` in
the fleet directory. Tested against a FIFO end to end and with unit tests on the frame, solve,
pointer, pin and control paths. The serialport crate can't open a pseudo-terminal on macOS
(IOSSIOSPEED), hence the stream option.

**The pointer and the anchors' own fixes.** `navgrid find --from N009 --to N010 --heading-deg 90`
gives the find-my-keys pointer (bearing relative to the seeker's heading, elevation, distance)
from the mirrored positions: a node's own fix first, else the controller's. An anchor without a
pin places itself once a minute from its links to pinned or controller-placed neighbours, held
near their height, and writes its own Position record (FR-M2, first cut); the beacon still
advertises the controller's position when there is one, so the host can compare the two (D-28).

**Positions, first cut** (S2). `navgrid_solver::mesh` solves every node at once from the links:
seeded outward from the pins by multilateration (two-anchor seeds by clearance from the nodes
already placed), refined jointly with a robust cut that starts wide and halves down to three
sigmas so a bounced link is cut rather than spread, a height prior holding unpinned anchors near
the pins' height, and sigmas from the inverse normal matrix. Node records carry their pin;
`navgrid solve` runs the solver over the links a sink mirrored, takes the chip's offset off, and
with `--sink` writes Position records to the mesh; an anchor without a pin advertises the
controller's position of it. A tag multilaterates from its bursts against the anchors' advertised
positions, held near carry height, and writes its own Position record at most every 3 s; its links
sync every 5 s. Untested on hardware with the rest.

**Commands through the mesh** (stage 4, second half). `navgrid control --sink PORT N042 blink`
signs a desired-state record with the controller key, AES-CMAC of the fleet key (`navgrid-mesh`
`auth`: the MAC covers origin, version, target and command, so relays carry but can't forge or
replay), sends it down the sink's serial port (`UpdateMsg::Record`) and waits for the sink to mirror
it back; the sink injects it and sessions push it toward the target; the node checks the MAC every
5 s among its pending commands, acts (blink raises the attention flag; environment, diversity and
advertising report Unsupported until settings change at runtime) and writes the Reported record,
which comes back up to the host. Built, untested on hardware with the rest.

**Sync sessions on devices** (stage 4, first half; DEF-62). The mesh crate's session runs frame by
frame now (`Driver`, bounded buffers: two classes of the peer's versions, records and pushes drawn
from the replica as they go), checked byte for byte against the in-memory session. The firmware
keeps a replica (`state`: 160 records, an outbox of 32, 48 routes) with the node record, health
every 30 min and a Link record per burst; every link runs a session at link-up, whose hellos are
the role exchange, and the central starts one every minute after, one session at a time on a
node. A sink mirrors each record it keeps to the host (`Message::Record`, protocol 6, frames up to
192 B) and all of them every 5 min; the monitor prints them. The beacon's hop count comes from the
replica. The dedicated sink link is gone for good (DEF-1 fixed). Untested on hardware with the
links; the controller's commands, the MAC and the mesh view are next.

**The mesh firmware** (stage 3, D-37; DEF-57). `mesh.rs` replaces the bring-up roles: every node
advertises its beacon while it has room, scans now and then, connects as central to the
neighbours the policy wants with its role's parameters and, for a tag, the k-th link's event
spacing, and serves up to 14 links in slots. Each link opens with a hello each way over the sync
channel (the mesh wire's `Header`, now with the role), so both ends know the other's role and sink
flag and the peripheral can admit or refuse. The central runs the policy's bursts: an anchor link
switches to 30 ms by connection parameter update, runs the procedures the controller counts,
removes its CS configuration and switches back; a tag's links stay fast. CS keeps its state per
connection handle and one burst at a time. Telemetry rides any link whose peer is a sink; the sink
forwards it to its UART from any link; updates ride any link, offered by whoever carries one. The
dedicated sink link, the provisioning peer and sink, and `report::set_link` are gone. Built for
both boards, untested on hardware: the desk mesh is next.

**Mesh links, the policy and the simulator** ([spike](spikes/2026-11-04-link-scheduling.md),
D-37). `navgrid-link` holds the link policy the firmware and the simulator share: neighbours from
beacons, the lower anchor connects, admission by the beacon's free links, one burst per node at a
time, surveys every 15 min and on motion, a moving tag rotating short bursts over its links and
settling when it stops. `navgrid link-sim` runs it over a radio timeline with the measured CS
timing and counts empties from other links' connection events on both ends. It settled three
things before any hardware: tags connect at the fast interval and never switch (a parameter update
per burst left 0.26 ranges/s), a tag's links share the radio at the fast interval times their
count with events spaced by the fast interval planned for the full count (greedy spacing left the
third link two-thirds empty), and three 90 ms links with four paths give about two ranges a second
over two anchors with half the procedures empty in the model; one path at 45 ms gives eight without
diversity. The anchors' slow-link events dominate the empties; the bench measures the real figure.
The dedicated sink link goes: telemetry rides any link to a sink.

**Roles: anchor, asset, mobile; the sink is a flag.** `Role` is now Anchor / Asset / Mobile (was
Node / Asset / Person / Sink): "node" stays the word for any mesh member, "mobile" covers whatever
is tracked at the fast rate (a HUD wearer, a robot), and any device with a host link is a sink
whatever its role, since a powered, stationary device with a CS radio is a good anchor
(requirements §8). Provisioning page format 5 carries `NodeConfig::sink`; formats 1–4 still decode
(a stored Sink role becomes anchor + sink), the beacon carries the flag in bit 2 and reads role
byte 3 from older firmware as anchor + sink, and the registry migrates `Sink` entries on load.
`navgrid provision --role anchor|asset|mobile [--sink]`; the bring-up pointer is `--report-to`.
Nothing flashed yet; no device needs re-provisioning.

**Morning after: the estimators on the raw tones** ([spike](spikes/2026-11-03-four-tags-proximity.md),
last section). Phase slope, windowed delay profile, matrix pencil and root-MUSIC side by side on the
four tone captures (`scripts/tones_estimators.py`, `scripts/tones_components.py`). The 3–3.5 m
responses are three or four components: a weakened direct-plus-floor pair, a ceiling bounce about
2.2 m longer, something at 10–12 m. The pencil at the singular-value order recovers the direct
component on every path at 3.5 m on edge and on two of four at 3.5 m flat, 0.3–1.2 m short against
the slope's 1.6–2 m long, but is 1–2 m noisy procedure to procedure and no better outside the fade,
so the shortest-coherent slope plus geometry stays the plan and the pencil is a host diagnostic. The
"harmonic ladder" was three components inside one Hann lobe. The desk pair's 1.3–1.9 m offset is
the same by every estimator: a real delay, per antenna path (DEF-6).

**After midnight: raw tones over the air, and the shortest coherent path**
([spike](spikes/2026-11-03-four-tags-proximity.md), last section). Initiators send one antenna
path's raw tones a second through the sink (`Tones`, protocol 5) and `navgrid tones` runs the host
estimators on them. Four captures on the chairs at 3.0 and 3.5 m, on edge and flat: the host
reproduces the device's phase slope; at 3.5 m flat one of the four antenna paths still read the
direct path (5.1 m) while the blend gave 6.5, so the device now combines paths by the shortest
coherent one (commit 770ae73); where all four paths read long the profile holds no earlier path for
a super-resolution estimator to find. An unexplained harmonic ladder (3.65, 7.15, 10.5 m) on one path
at 3.5 m on edge.

**After midnight, later: the chairs with the pucks on edge** ([spike](spikes/2026-11-03-four-tags-proximity.md),
last section). Three quarters vertical by power (the two chips sit 120° apart, each 60° from the
horizon on edge): the 3–4 m fade is shallower (3.2 m over true at worst against 4.4 flat, the RTT's
excess 1.9 m against 2.9) and no tones are lost, but it is still there. Polarization helps; placement
(a metre below the ceiling, unequal node heights) matters more.

**After midnight: the chair sweep** ([spike](spikes/2026-11-03-four-tags-proximity.md), last
section). N005–N004 on chair seats 0.45 m up, 0.50 to 4.00 m in 25 cm steps: true + 1.2–2.0 m up to
2.75 m with the RTT rising alongside, then a step of 2–3.5 m at 3.00 m that holds to 4.00 m, and at
3.50 m three procedures in four had too few tones for an estimate. The 3 m edge is the floor
reflection's first-order fade at this height (2h²/d = λ at 3.3 m), wide because the path ratio then
changes slowly, matching last night's pinned 3–5 m and recovery at 6 m.

## 2026-11-03

**Late night: a node high over a tag on the floor ranges**
([spike](spikes/2026-11-03-four-tags-proximity.md), last section). N004 flat on a wardrobe at
2.30 m, N005 flat on the floor from directly below to 3 m sideways (true 2.30–3.78 m): the phase
slope read true + 1.5–2.0 m at every point, tight, quality 243–247, and the RTT rose with it; flipping
the floor puck moved it 0.16 m. The same pucks were pinned on the chairs an hour earlier. Two low
pucks at one height sit in the floor reflection's fades (two-ray, not the antenna pattern, whose
plots are near-uniform); a node high over the floor is the product's geometry, and it works at
these distances. Data in `captures/2026-11-03-home/night2/`.

**Late night: a round-trip-time referee, no factor of two, and the chairs**
([spike](spikes/2026-11-03-four-tags-proximity.md), last section). Two chair-seat points read
twice the distance, so mode-1 steps went into the procedure and the initiator now reports its mean
RTT as a second `Range` (method `Rtt`, reflector turnaround not transferred, so a per-pair bias;
unreceived steps dropped; reported 100 m up), delivered to the four TAGs over the air in three
rounds through the sink. Over 1–4 m the RTT tracked the phase slope within 0.5 m: the phase slope
measures a real path, and from 3 to 4 m neither method moved, so it isn't the direct one. Also
found: `navgrid cs` without `--ipt` was reading the device's own combined tones back, so the
"raw tones" checks earlier today were not independent (DEF-54); the capture script passes it now.

**Night: the calibration series, N005–N004 on batteries 0.35 m up**
([spike](spikes/2026-11-03-four-tags-proximity.md), last section). Nine points from 1 to 6 m: not
an offset. Only 2 m read true + 1.3 m; 3–5 m were pinned near 7.5 m; flipping one puck moved the 6 m
reading by 1.5 m; and every point was tight with quality 230–246, so the quality figure can't flag
a wrong path (DEF-53). The cabled pair at 0.6 m read the same all evening. Undecided, and the
next test: the same pair 1.5 m up, same side up, at 3 m.

**Evening: is the ranging broken? No** ([spike](spikes/2026-11-03-four-tags-proximity.md), the
last section). Yesterday's firmware against today's at one placement read the same to 4 cm, 2.0–2.3
m over the true 0.6 m, the known offset (DEF-6). Flat on a wooden desk the readings had been 2–6×
long with a component 9 dB stronger than the direct path 2–2.5 m further out; off the desk surface
they were tight. The first-path estimator found the direct path under that component in every
run where the two were separable: its first real-data result. Six captures kept under
`docs/engineering/captures/2026-11-03-home/`.

**A first-path estimator, on the host** (DEF-51).

- **Built:** `navgrid_ranging::first_path`, orthogonal matching pursuit over a windowed delay
  profile: strongest component first, joint least-squares amplitudes, each distance refined
  against the joint residual, up to four components; the earliest is the distance. No
  allocation, so it can go on the device.
- **Learned on the way:** the round-trip response is the one-way channel squared, so a weak
  direct path beside a strong reflection is 20 dB down, not 10, and there's a cross term at the
  two paths' mean. A plain profile merges components closer than 4 m; subtract-and-search
  leaves artefacts from a 10 cm error on the strong one; the joint fit with refinement is what
  made the floor case pass in simulation (1.5 m under a 10 dB stronger 6 m bounce, within 0.3 m).
- **On the 40 cm capture:** median 1.81 m against the phase slope's 1.96 m; a weak earlier
  component taken in about one procedure in ten. `navgrid cs` now reports both.

**Four TAGs: two CS pairs, and what placement does to ranging**
([spike](spikes/2026-11-03-four-tags-proximity.md)).

- **Brought up:** N004 (reflector) and N005 (initiator ranging with N004), one probe session
  each with the bootloader, image and enrolment. All four TAGs report to the sink at once, both
  pairs range at 10.6/s (11.0 alone).
- **Both pairs read 10–20 m over 2 m,** and a day of moving TAGs about found why, in order:
  - **not the batteries** and **not the other pair's procedures**;
  - **a board or an object within about 0.5 m** of a TAG (another TAG, dead or alive, a DK, a
    metal jar) scatters its readings or makes them tightly wrong; **1 m clear** reads clean;
  - **flat on the floor,** 1.5 m apart, a tight 7–10 m: the floor reflection cancels the direct
    path;
  - **6.1 m through office partitions:** 12–20 m, scattered, a quarter of the procedures empty.
- **The short-range "3×" was the calibration offset:** every clean reading, the morning's
  included (those TAGs were 40 cm apart, the cables' reach), sat 1.5–2.5 m above the truth. A
  raw capture at 40 cm (`NAVGRID_LOG_STEPS`, both boards, `navgrid cs`) gives 1.96 m from the
  tones themselves against 1.88 m on the device: the radio's delay, DEF-6, not the estimator.
- **What follows:** the phase-slope estimator has no multipath rejection (DEF-51); keys on the
  floor need a first-path estimator, developed on captures from the floor. The monitor's
  battery forecast is fooled by fresh cells' first-hour sag (DEF-52).

**Reflectors report, through extended advertising with legacy PDUs** (DEF-50).

- **N002 reports:** its own sink link runs beside its CS link; the sink served N001 and N002 at
  once, Health from both every minute.
- **What it took:**
  - **Extended commands for the beacons:** the SDC refuses an extended connect after legacy
    advertising.
  - **Legacy PDUs, though:** with extended PDUs N001's CS link landed anywhere relative to its
    sink link, and 5 fresh boots of 5 lost every 4th procedure. The SDC's ACL event spacing
    (DEF-26) doesn't apply to connections made from extended advertising. With legacy PDUs
    through the extended set: 3 fresh boots, 11.0/s.
  - **A 1 s interval for a reflector's sink link:** at 120 ms, a multiple of the CS link's 30 ms,
    it sat on 2 procedures in 4. At 1 s it drifts across them, 14–22 empty procedures a minute.
  - **250 ppm declared for the sink's sleep clock:** at 1 s with 50 on both sides the link lost
    sync after its first packet (DEF-2). On the TAGs 250 changed nothing; it was ruled out on the
    way.
- **Also:** Health no longer waits for a CS link's IDs, so reflectors send it. An initiator now
  sets them again on each uplink: after the first drop it had stopped sending ranges for good,
  a bug from this morning's refactor. The sink's uploads stuck again (DEF-46); its erase and
  header writes now have deadlines, and the host waits 15 s.

**A sink for several nodes, and reflectors' reports (blocked).**

- **The sink** serves up to 4 nodes: one future advertises and hands each connection to a free
  slot. The "carrying an update" signal is now a `Watch`, so every link offers, and the sink
  offers only what it carries; it was offering its own DK image to TAGs. Tested with N001 only.
- **A node takes one update at a time:** a reflector listens on its CS link and its sink link,
  and two transfers would write one slot.
- **A reflector provisioned with a sink** opens its own sink link and reports Health. On N002
  the connect failed every time with Command Disallowed: the SDC refuses extended commands after
  legacy advertising (DEF-50). Meanwhile N001 lost every 4th CS procedure; with N002 back to no
  sink, none in 350. Next: extended advertising for the beacons.

**Logs over the air at a configured level.**

- **Built:**
  - **The logger** is defmt-rtt's RTT code, vendored, plus a copy of each frame at or above the
    device's report level. It tells a frame's level from its string index and defmt's level
    markers.
  - **The setting:** `navgrid provision --report-level`, in provisioning format 4; errors only by
    default.
  - **The host:** `Boot` and `Log` records name their build and board. `navgrid flash` and `sign`
    keep each ELF in `builds/`, and the monitor decodes frames and crash PCs with it.
- **On the bench:**
  - **N001 at warn level** sent `WARN the last run crashed: … (main.rs:298)` at each boot of the
    crash test.
  - **Its boot reports** read "hard fault at 0x0002f87a in …crash test task…".
  - **When N002 was taken away,** N001 sent one "too few channels" warning per procedure for 4 s:
    the cost of a dev level.
- **Fixed:** with its CS peer away, an initiator couldn't connect its sink link (DEF-47), so the
  sink, updated meanwhile, saw no node for 5 min and reverted. The peer connect now gives up
  after 10 s.

**Boot and crash reports.**

- **Built:** every boot queues a `Boot` record (protocol 4) with the build, boot number, reset
  reasons, the image state and the last crash. The panic and HardFault handlers leave a record in
  `.uninit` RAM at `0x20023090`, 116 KiB below the bootloader's stack, then restart; `panic-probe`
  is gone. The sink sends its own reports to its UART.
- **On the bench:** with a test aid (`crash-test`) that crashes 20 s after boot, N001 reported
  each crash at its next boot: "hard fault at 0x0002f61e", which `llvm-symbolizer` puts at the
  test's `udf` in `crash.rs:122`, and "panic at line 120: crash.rs: crash test at boot 80". The
  sink's first report read: reset pin and soft reset, image new, no crash.
- **Found:** the sink's serial receive once stopped answering mid-upload until a power cycle
  (DEF-46).

**Revert after an update over the air** ([spike](spikes/2026-11-03-revert-after-ota.md)).

- **On the bench:** a test image that never confirms (`never-confirm`) went from the host through
  the sink to N001. N001 ran it, restarted at the 5-minute deadline, and the bootloader put the
  previous image back, confirmed.
- **Two policy changes, both seen working:**
  - a node offers its installed image only once confirmed, so N002 never got the bad one;
  - a node declines a build it reverted from, so the sink's next offer of it was turned down.
- **Left open:** the sink, as the carrier, doesn't learn that a build failed, and kept offering
  it until the host sent another.

**The sink link's packet length** (DEF-42,
[spike](spikes/2026-11-03-updates-through-the-sink.md#follow-up-the-sink-links-packet-length-def-42)).

- **Full length only for updates:** the sink link keeps short packets, and the receiving node
  switches it to full length for an update and back after. N001's uplink is back to 11.1/s at
  rest, outbox empty.
- **Credits four chunks at a time:** with one packet per event during a transfer, TrouBLE's
  credit after each chunk took all of them, and the uplink carried nothing for the 4.5 min.
  Batched, it carries 6.6/s, and the transfer runs at 1.5 KB/s instead of 1.8. Ranges and Health
  are still dropped during an update (DEF-45).
- **The sink updated itself** over serial, and all three devices now run the same commit, the
  TAGs as 0.1.4 from the sink and from N001.

**Updates from the host through the sink, working**
([spike](spikes/2026-11-03-updates-through-the-sink.md)).

- **On the bench:** `navgrid update b2.nup --sink` sent 0.1.2 to the sink in 71 s. The sink
  passed it to N001 over the sink link in 273 s; N001 checked, swapped and confirmed it (tag
  `0.1.2`, state `0xD0`), then passed it to N002 over the CS link in about 70 s. N002 restarted
  into it (a 34.6 s ranging gap); its tag then read `0.1.2` and its state `0xD0`.
- **The cause of DEF-41:** N001's telemetry sender kept TrouBLE's per-link send lock to itself.
  Its outbox is always full on the sink link (DEF-42), so each time it released the lock it took
  it again, and the update channel's accept and returned credits never went out. It now yields
  after each frame.
- **Still open:** the uplink cap itself (DEF-42) and the UART framing errors (DEF-43).

**Updates from the host through the sink, first half.**

- **The sink DK:** reflashed under the bootloader and enrolled as N003, on the current protocol.
- **Boards:** an update's header now names its board (format 2), and the image carries a
  `navgrid-board` tag that `navgrid sign` reads. A device declines another board's update.
- **Host to sink, working:** `navgrid update FILE --sink PORT` sends an update over VCOM1, a 4 KiB
  page per message with a CRC-32, each after the sink's answer. The sink checks the signature
  (0.93 s), then installs its own board's update, or keeps a TAG update in its update slot to
  offer to the nodes on its links. B (490 KB) went over in 63–77 s.
- **Fixed on the way:**
  - the sink checked the signature before the last, partial page was written;
  - a receiver restarted 100 ms after its `Done`, before a slow link could carry it, and now
    waits for the sender to close the channel;
  - a write that gets no timeslots within 10 s now fails instead of holding the transfer.
- **Sink to node, not working** (DEF-41): in three runs the sink's transfer to N001 stalled near
  the end twice, and once N001 never answered the channel request. N001's CS held 11.1/s
  throughout.
- **Also found:** the sink link at 2M and 251-byte packets caps the uplink at 8.3 frames/s
  (DEF-42), and the sink's UART receive takes framing errors (DEF-43).

**Single-hop updates over the air** ([spike](spikes/2026-11-03-single-hop-update.md)).

- **Built:** a node passes the update it was installed from to a peer over their fleet link, on
  SPSM `0x0083` (`update.rs`). The peer writes it into its update slot and the staging page, then
  restarts into the existing check, swap and confirm.
- **On the bench:** N001 got 0.1.1 through the probe and passed it to N002 over their CS link:
  500 KB in 67 s (7 KB/s). N002 verified, swapped and confirmed it, and ranging held 11.1/s
  throughout the transfer. A peer already on the build declines it.
- **Found on the way:**
  - **RRAM writes during CS:** blocking 4 KiB page writes cost 7 % of the procedures, and once
    tripped SoftDevice Controller assert `131:2004`. After boot, RRAM writes now go through MPSL
    timeslots (`nrf_mpsl::Flash`). DEF-38 is now only the swap time.
  - **Closing the channel too early:** the sender dropped it right after the last chunk, and the
    receiver lost the tail. The receiver now acknowledges.
  - **Updates on an unconfirmed node:** they would overwrite the image a revert needs, so such a
    node declines them.
- **Flashing:** `navgrid flash` now also blanks the kept and staged update headers, so a
  probe-flashed image never offers itself under a stale header.

**Signed updates.**

- **Keys:** `navgrid fleet init` adds an Ed25519 signing key, and `navgrid provision` writes its
  public half onto the device (provisioning format 3).
- **Update files:** `navgrid sign` makes a signed update file: a header, then the image for the
  active slot. `navgrid update FILE --probe P` stages it: the image into the update slot, the
  header into a staging page.
- **The check:** the device checks the signature at boot, before the radio starts, then has the
  bootloader swap the update in.
- **On N002:**
  - **Signed 0.1.1:** verified in 1.13 s, swapped in, and confirmed.
  - **A byte flipped in the image:** rejected.
  - **Signed with another fleet's key:** rejected.
  - **Rejections:** both left the running image and the confirmed state alone.
- **Cost:** ed25519-dalek and SHA-512 add 60 KB to the image, now 475 KB of the 720 KiB slot.

**DEF-26 fixed again.** Spacing a sink link that connects second never kept its events off the
CS subevents. At 26 ms or 27.5 ms after the CS link's anchor, every fourth procedure stayed empty
(8.3/s). Only the sink link first, with the CS link 2.5 ms after it, holds. So when the sink link
connects under a CS link, the initiator now reconnects the CS link after it, losing about 1.5 s of
ranging. Four resets: 11.1/s with no empty procedures. Three of them re-placed the CS link; in the
fourth the sink link came first.

**The bootloader works** ([spike](spikes/2026-11-03-bootloader.md)).

- **Bootloader:** embassy-boot on the TAG, 5.4 KB in its own workspace, with the image moved to
  `0xB000`. MPSL, the SDC and CS run unchanged under it.
- **Firmware side:** the image takes over the bootloader's watchdog. A new image confirms itself
  once a fleet link is up.
- **On N002:**
  - **Swap:** to a 0.1.1 build took about 25 s for the whole slot; the confirm wrote in 75 ms.
  - **Persistence:** the confirmed image survived a reset.
  - **Revert:** a build that panics at boot was reverted after the watchdog reset it.
- **Layout:** the state region needs 12 KiB, not 8 (four progress lines per page); the layout is
  now in both `memory.x` files and `navgrid_proto::layout`.
- **Flashing:** `navgrid flash --bootloader` writes the bootloader, and every flash blanks the
  state region. Both TAGs now run under the bootloader.
- **Found on the way: DEF-26 is back.** With the CS link up before the sink link, every fourth
  procedure is empty (8.3/s), in every run today, the run before the bootloader included. Making
  the uplink wait until CS runs didn't help, so it was reverted. Only the sink-first order holds
  11.1/s.

**Firmware versions tracked; updates designed (FR-O2, architecture §13).**

- **Identity:** every build carries its version, commit and dirty flag (`0.1.0+1950d455-dirty`),
  stamped by `build.rs` and embedded as text in the image.
- **Reporting:** devices log it at boot, and report it in Health, the sink's Hello and node
  records (protocol version 2).
- **Recording:** `navgrid flash ELF --probe A --probe B` flashed both TAGs in parallel in
  2 min 19 s and recorded each device's build in the registry. The monitor shows the build and
  marks a mismatch with the registry.
- **Decided:**
  - updates over the air come single hop before stage 3, and mesh-wide after stage 4 (D-36);
  - images are signed with their own Ed25519 key (D-35);
  - the bootloader is to be embassy-boot (D-34), which supports the nRF54L15. A spike comes
    first.

## 2026-11-02

**Operations items 1–3: fleet numbers, LED and button, battery.**

- **Registry:**
  - `navgrid fleet init` sets up the fleet directory: the key, `devices.json` and `labels/`;
  - `navgrid provision` reads the device's ID through the probe and keeps or assigns its fleet
    number (provisioning page format 2, format 1 still read);
  - it then writes an SVG label with a QR code; `navgrid fleet list` shows the fleet.
  - The bench TAGs are N001 (initiator) and N002 (reflector).
- **TAG LED and button:**
  - boot colour, the short-press status display (connectivity, then battery, as colour plus
    pattern), a long-press acknowledgement, and a factory reset when held at power-up;
  - on the bench the display matched: N001 showed `Sink(1)` and N002 `Mesh`, both with a good
    battery.
- **Battery:** the SAADC reads VDD, as the lowest of 16 samples a minute (3.32–3.34 V from the
  DK). It goes into the beacon (format 2: fleet number and flags) and into Health reports, once
  a minute over the uplink. `navgrid monitor` shows `N001 … health: battery 3339 mV good`, and a
  days-to-low trend after an hour.
- **Not on air yet:** a node stops advertising while it is linked, so beacons and the attention
  flag are off air then (DEF-35). The battery thresholds are untested on a coin cell (DEF-36).

**Operations designed (FR-O4–O7, architecture §13).** Decided:

- **Enrolment** at a bench through the probe (D-29).
- **One shared fleet key**, re-enrolling the fleet if one is lost (D-30).
- **Labels:** sequential fleet numbers stored on the device (D-31).
- **Both ways of working:** every operation through the host and the mesh, plus a field tool
  (laptop + DK as a portable sink) that does them in seconds on site (D-32).

The TAG has one RGB LED and one button, and no usable NFC, so the LED and button are
event-driven: boot code, status, identify, install mode, factory reset at power-up. Plan in the
[todo](todo.md).

**Stage 2: devices provisioned at runtime, links encrypted under the fleet key, beacons on air.**

- **Provisioning:** `navgrid provision` writes a CRC-checked page (role, diversity, pin, fleet
  key, bring-up peer and sink) to the second-to-last RRAM page through the probe, and resets the
  device. `navgrid fleet-key` makes the key file. Flashing firmware leaves the page alone; the
  build-time role variables are gone (DEF-5, DEF-14 fixed).
- **Boot epoch:** each boot writes the next of 256 slots in the last RRAM page. It counted 1 → 7
  across resets and reflashes. The mesh epoch is now `u32`.
- **Fleet keys (D-7):**
  - **How:** the link key is AES-CMAC of both addresses under the fleet key, added to TrouBLE as
    a non-bonded key, so the links encrypt without pairing.
  - **Bench:** on the TAG pair and the sink, both links encrypt about 0.2 s after connecting, and ranging
    holds 11.1/s with the uplink up.
  - **Wrong key:** a reflector given another fleet key got no link through: MIC failure on
    every attempt.
  - **Pairing:** a provisioned node drops a peer that pairs (DEF-34).
- **Beacon (FR-M4):** 17 bytes of manufacturer data. The initiator heard the reflector's and the
  sink's at −48 to −58 dBm during a 3 s scan at boot.
- **Found on the way:**
  - TrouBLE runs one security procedure at a time (DEF-32): the uplink got `InvalidState` while
    the CS link encrypted, then waited on the central role forever.
  - Releasing the central role between connecting and encrypting let the CS link connect
    without its spacing, which emptied every 4th procedure (8.3/s). A link now counts as up as
    soon as it connects.

**Mesh state, stage 1: the `navgrid-mesh` crate and `navgrid mesh-sim`**
([spike](spikes/2026-11-02-mesh-state-sim.md)). The crate:

- **Records and merging:** single-writer records, newest version per key.
- **Classes and scope:** a node keeps its own records and its 2-hop neighbourhood's.
- **Sync:** per-class hashes, then versions, then records.
- **Custody pushes:** toward sinks along the hop-count gradient, and back toward targets along
  learned routes.
- **Commands:** desired state with reports.
- **Memory:** fixed-capacity tables; it builds for the nRF54L15.

Simulated at 20 and 500 nodes:

- **Steady state:** at 60 s sessions, neighbourhood replicas are current ~97 % of the time.
- **Commands:** round trip 1.6 min at p50 with 20 nodes, and 4.8 min with 500 nodes and 4 sinks.
- **Partitions:** replicas are back to normal 4–12 min after a one-hour partition.
- **Cost per node** is set by degree: 140 records and ~650 B per session at 500 nodes.
- **Sinks:** one sink can't carry 500 nodes; about one per 125 can.

§12's per-origin digest broke under relaying and was replaced. Strict push priority starved bulk
records and was replaced by commands first, then oldest first. Budget chosen: 2 KB per side per
session. Full summaries are most of the traffic (DEF-28).

**Mesh state, routing and control designed (FR-C, architecture §12).** The requirements for the
full system: a shared state the mesh agrees on, routing to keep it, positions and the link graph
built on it, sideband and commands carried by it, and a controller that observes and steers it.
Decided:

- **Consensus is convergence** (D-24). Every record has one writer and a per-origin sequence; the
  highest wins. Merging is order-free, so replicas agree across partitions without voting.
- **Scope** (D-25): a node holds its own records, its 2-hop neighbourhood and routing state;
  sinks and the host hold everything.
- **Routing** (D-26, proposed): sync on ranging contacts, a hop-count gradient up to sinks, the
  reverse path down to targets.
- **Commands are desired state** (D-27): written and authenticated by the controller, applied
  once by the target, which reports what it applied.
- **Positions** (D-28): the host solves and publishes them as records first; on-node solving
  follows.

Plan in the [todo](todo.md): the `navgrid-mesh` crate and a simulator first, then runtime
configuration, mesh links, sync, and positions.

**The uplink no longer costs ranging (DEF-26 fixed).** The initiator schedules its two central
links apart, following the SDC docs' advice for CS with several central links:

- **ACL event length 2.5 ms** (SDC default 7.5 ms), so the CS subevent ends about 26 ms into each
  30 ms CS-link interval.
- **Sink link at 120 ms**, a multiple of 30 ms, so its offset from the CS link stays fixed.
- **Spacing per connection:** the central ACL spacing is set just before each connection. A sink
  link that comes second goes 26 ms after the CS link; a CS link that comes second goes 2.5 ms
  after the sink link.
- **Result:** 11.1 ranges/s on-device and at the host, no empty procedures, both with the sink
  first and after the uplink reconnected under a running CS link.
- **The reconnect:** it came from DEF-19. The reflector's probe on the sink DK dropped, and the
  DK's debugger re-enumerated.

**Calibration infrastructure, nothing calibrated yet.** Calibration lives on the host:

- **Model:** a JSON file holds one constant offset per (initiator, reflector, antenna paths,
  method), with the sample count and p10–p90 spread it was fitted from.
- **Fit:** `navgrid calibrate --out cal.json capture.jsonl=<metres> …` fits it from
  `monitor --json` captures at measured distances (median of reported − true), keeping entries
  for other keys.
- **Apply:** `navgrid monitor --calibration cal.json` adds each range's calibrated distance
  (`calibrated_mm` in JSON).
- **Smoke test:** a fit on a live capture at a rough 90 cm turned the stream into 0.80–0.94 m. The
  file was not kept.

**Ranges reach the host over the air.** The initiator TAG streams each on-device range to a
sink, and `navgrid monitor` prints it.

- **Sink:** a DK (`001057770413`, node `c00cc5545178`) built with `NAVGRID_ROLE=sink`. It writes
  a `Hello` frame at boot, then forwards every SDU of its L2CAP channel (SPSM `0x0081`) to UART20,
  which is the DK's **VCOM1**. This was the DK's first boot of our firmware; the derived SDC memory
  was exact.
- **Initiator:** built with `NAVGRID_SINK=<sink id>`. It keeps a 100 ms link to the sink next to
  its CS link, sharing the central role, and retries every 5 s if the link drops. It queues a
  `Range` per estimate (σ from the shared placeholder model, quality from coherence).
- **Result:** 154 ranges in 20 s at the host, every one the initiator produced, median 3.39 m.
- **Cost:** the sink link takes about a third of CS procedures (11 → 7.8/s, DEF-26). At 500 ms
  the link dropped after its 4 s supervision timeout every time.

**Faster ranging: 11 estimates/s on four paths.** Details:
[spike](spikes/2026-11-02-cs-ranging-rate.md).

- **Timing:** a 30 ms connection, with procedure timing by path count (four paths: two 22 ms
  subevents every 3 events). Four paths now give 11.1 estimates/s and one path 16.7/s, both with
  about 0.15 m p10–p90 spread; before, it was 2/s.
- **Logging:** raw step logs are now opt-in (`NAVGRID_LOG_STEPS` at build time), because they
  outrun 100 kHz SWD at these rates.

**IPT on, 2×2 antenna diversity.** Details: [IPT](spikes/2026-11-02-cs-inline-pct-transfer.md),
[diversity](spikes/2026-11-02-cs-antenna-diversity.md).

- **Fork:** `DmitriyNE/nrf-sdc` main is now `6381257`, which the firmware pins. It has three
  merged changes: the critical-section fix (DEF-23), nrfxlib v3.4.1, and RRAM standby in the
  low-latency hooks. IPT needed both of the last two (DEF-24); `INLINE_PCT` is now on.
- **Diversity:**
  - `navgrid-ranging::steps` maps tones to antenna paths with the Core permutation tables.
  - The initiator picks the tone antenna configuration from both antenna counts (A2:B2 between
    TAGs).
  - On-device ranging keeps one response per path and combines them by residual.
- **Result at about 90 cm:** two captures gave median 3.72 m (p10–p90 3.62–3.87) from 4 paths.
  Paths 0–2 agree; path 3 is consistently bad (DEF-25). No accuracy gain over one path at this
  placement yet.

**Inline PCT transfer: the initiator ranges alone, with SDC v3.4.1.** Details:
[spike](spikes/2026-11-02-cs-inline-pct-transfer.md).

- **What's in:** IPT in Create Config, the SDC extended feature set, and an on-device phase-slope
  estimate on the initiator (`distance <procedure>: … m` log lines). `navgrid cs --ipt`
  estimates from the initiator's capture alone.
- **v3.4.0 doesn't work:** our reflector applied the IPT phase wrongly (DRGN-29459). Mixed pairs
  with Nordic's samples pinned it on our reflector's controller.
- **v3.4.1 works:** our pair gave 3.88 m on-device (coherence 0.986), equal to the host estimate.
- **Not pinned yet:** the fork update is on a local branch, so `INLINE_PCT` is off in main
  (DEF-24).
- **DEF-19 reopened:** one probe dropout at 100 kHz in about 25 runs.

**Distance check.** At about 10 cm and 90 cm, ours read 2.34 m and 3.34 m; Nordic's samples
read 2.17/2.93 m (IFFT) and 2.47/3.56 m (phase slope). Sign and scale are right (DEF-6); both
stacks share a fixed offset of about 2.4 m, which waits for a measured calibration.

**CS ranges on our stack (DEF-18 fixed).** The cause was the instruction cache: embassy-nrf
leaves it off on the nRF54L, Zephyr turns it on, and without it the SDC can't keep up with CS
step processing. With `ICACHE` enabled, 255 of 255 procedures ranged with no aborts, median
2.342 m (p10–p90 2.282–2.386 m). Details: [spike](spikes/2026-11-02-cs-subevent-aborts.md).

- **Narrowing it down:** NCS v3.4.1's `ras_*` samples ranged on the same TAGs, so the hardware
  was fine. With Nordic's reflector, our initiator still aborted everything, so the fault was ours.
  An HCI byte trace showed our CS commands identical to NCS's.
- **Ruled out first:** the newer SDC, Nordic's timing and controller setup, central ACL spacing,
  HFXO latency, RRAM standby, sub mode 1.
- **Found on the way:** nrf-mpsl's nRF54L critical section disables the radio interrupt (DEF-23);
  fixed on fork branch `fix-cs-reserved-irqs`, not yet pinned.
- **Config log:** the initiator now logs the full config-complete event (modes, steps, channel
  map, T_IP1/T_IP2/T_FCS/T_PM).

**Lab session: probe dropouts fixed, subevents still abort (DEF-18, DEF-19).** Both TAGs on DK power
at 3.3 V, each logged by its own DK at the same time. Details: [spike](spikes/2026-11-02-cs-subevent-aborts.md).

- **DEF-19 fixed:** the probe lost the TAG on DK power too, so it was never the coin cell. At
  100 kHz SWD (probe-rs defaults to 400 kHz) no probe dropped in about 20 minutes of CS. The
  runner now passes `--speed 100`; a flash takes about 2 minutes.
- **DEF-18 unchanged:** every subevent still aborts with reason 0x3. Eight runs ruled out:
  - the 100 ms interval and data length 251;
  - a 16 ms CS event length and NCS's controller setup (roles, mode-3, link count, 10 µs tones);
  - one antenna, MPSL owning the HFXO, and moving embassy's timer off GRTC CC[11].
- **Kept from the runs:** data length 251 on connect, CS event length equal to the subevent length
  (as the SDC docs say), and T_PM 10 µs (as NCS sets).
- **Found on the way:** embassy's GRTC timer shares CC[11] with MPSL (DEF-22). A DK's board
  controller can time out until its USB is replugged. nrfutil can read the DK's VDD without
  Board Configurator.
- **Next:** Nordic's RAS samples on the same TAGs. NCS v3.4.1 is installing.

**Host capture parser (DEF-15).** `navgrid cs <files>` turns raw CS captures into one distance per
procedure.

- **Input:** it reads the `CSR`/`CSC` lines from probe-rs output (colour codes and other lines are
  ignored) and joins initiator and reflector by procedure counter. Mode-2 tones on each channel are
  paired in order, then go through phase slope per antenna path, path combining, and RTT from
  mode-1/3 steps.
- **Output:** text or JSON, with abort reasons in words and the initiator's frequency compensation,
  plus a summary (median and p10–p90 distance, aborted subevents per side).
- **Decoder:** step decoding lives in `navgrid-ranging::steps` (`no_std`, reusable on the device).
  It covers framing, all four modes for both roles, PCT and frequency formats, and tone quality.
- **Simulator:** `sim::for_each_tone` exposes each side's PCTs. `sim::response` now uses it and
  gives identical output.
- **Checked:**
  - a simulated 4.2 m link, encoded exactly as the firmware logs it and split over `CSR`/`CSC`
    subevents, comes back within 10 cm on all 72 channels;
  - today's real captures parse and show the 0x3 aborts and the −6.06 ppm offset.
- **Limits (DEF-21):** one antenna path only (permutation tables missing), and one connection per
  capture.

**CS runs end to end, but every subevent aborts (DEF-18).** Reflector `df96e04a5900` and initiator
`ce36dfcc4c2d`, each logged in turn while the other ran on its coin cell.

- **Capabilities stall: fixed.**
  - **Symptom:** `LE CS Read Remote Supported Capabilities` never completed, in either direction,
    with v1 or v2 unmasked, and with no link-layer timeout.
  - **Cause:** the SDC stalls when that command has to start the LL feature exchange itself.
  - **Fix:** run the exchange from the host on connect (`LE Read Remote Features`), as Zephyr does.
    Capabilities, config, CS security and procedure enable then all succeed.
- **Every subevent aborts.**
  - **Diagnostics added:** `CSR`/`CSC` lines now carry the procedure and subevent done status and
    the abort reasons, and the procedure-enable log shows the negotiated timing.
  - **Result:** every subevent on both sides ends aborted (`0xF`) with subevent abort reason `0x3`
    (scheduling conflict or limited resources), and no steps. Negotiated: event interval 2, one
    subevent per event, subevent length as requested.
  - **Radio works:** sometimes the initiator reports a real frequency compensation (−6.06 ppm) and
    a reference power of −30 dBm, so mode-0 exchange happens.
- **No effect at a 20 ms interval:**
  - SDC support for PHY update, DLE and LE power control (as NCS enables them; +1,608 bytes, SDC
    now 95,448);
  - NCS's 5 ms CS event length (`sdc_hci_cmd_vs_cs_params_set`, through raw FFI for now);
  - a 2.5 ms ACL event length on the reflector;
  - 90 ms central ACL event spacing;
  - an ACL update to 2M (as Zephyr's central does);
  - 5 ms subevents;
  - 1/1/1 links instead of 6/8/6.
- **Kept anyway, matching NCS:** the support flags, the CS event length and the 2M update. The
  spacing and link-count experiments were reverted.
- **Nordic's bug:** Nordic acknowledges an SDC bug where all subevents abort for some subevent
  length and interval combinations (DevZone 129138, no fix date). Their workaround is a longer
  interval.
- **Next:** the initiator now asks for 50 ms (one of Nordic's tested intervals) with 16 ms
  subevents. That build is ready but untested, because the probe failed first (DEF-19).
- **Related:** unmasking the v2 capabilities event made TrouBLE panic, since bt-hci can't parse
  subevent `0x38` and TrouBLE unwraps (DEF-20). The host runner's errors are now logged instead of
  being dropped silently.

**First two-TAG run: connects and pairs, then CS stalls.** The reflector is TAG `df96e04a5900` on
its coin cell; the initiator is TAG `ce36dfcc4c2d` on the probe.

- **Legacy connect asserts (DEF-17).**
  - **Symptom:** `LE Create Connection` made the initiator's SDC hit internal assert `50:701`
    about 0.3–0.5 s in, when the peer's advertisement was heard.
  - **Same report elsewhere:** OpenBikeComputer issue #736 hit the identical assert on an
    nRF54L15 DK and fixed it with the extended command set.
  - **Fix:** `connect_ext` (LE Extended Create Connection) plus `support_ext_scan` and
    `support_ext_central`. The SDC now needs 93,840 bytes (+648).
- **Ruled out:**
  - **GRTC channel overlap:** embassy's time driver uses CC[11] while nrf-mpsl claims channels
    7–11. Sampling the registers every 50 ms showed MPSL using 8 and 10 (interrupts, domain 3)
    and 9 (PPI), and never 7 or 11.
  - **Interrupt priority:** moving the time and GPIOTE interrupts off priority 0 didn't change
    the assert. They stay at 2 anyway, because mpsl.h reserves 0 for MPSL.
- **Result:** connected at 0.46 s and encrypted (Just Works) at 1.1 s.
- **Stall:** `LE CS Read Remote Supported Capabilities` was accepted, but no completion event,
  nor any other LE event, arrived within 5 s. The link then stayed up for the rest of the 60 s
  capture.
  - **Ruled out on the initiator:** event mask bit 43 is set, the binding matches the C struct,
    and a parse failure would have panicked.
  - **Next:** the reflector's side, which no capture has seen yet.

**First boot on a TAG.** The node boots to advertising on an nRF54L15 TAG (through the DK's
`DEBUG OUT`) and reports its CS capabilities: 1 config, 2 antennas, 4 paths, both roles, mode 3,
2M CS_SYNC PHY. Four fixes were needed.

- **Hang in `embassy_nrf::init`: stale GRTC state.**
  - **Symptom:** the CPU sat in `GRTC_2` forever. GRTC channel 6 had its domain-2 interrupt
    enabled and `EVENTS_COMPARE[6]` latched. embassy's time driver unmasks `GRTC_2` but owns and
    clears only channel 11.
  - **Cause:** probably the Nordic demo's system timer. The GRTC survives every reset short of a
    power cycle, and the TAG has stayed on its coin cell (the pre-init defmt timestamp read 315 s).
  - **Fix:** `board::early_init` now clears every GRTC interrupt enable, compare channel and
    event before `init`. This belongs in embassy-nrf too (P-5).
- **DC/DC:** `embassy_nrf::init` already enables DC/DC on nRF54L, after the FICR trims and clocks,
  so our earlier enable is gone. Enabling it before the trims had coincided with the SWD link
  dropping; that was not pinned down.
- **SDC builder order.**
  - `cs_count` doesn't count toward memory until the `support_*` calls have run.
  - `cs_cfg` with 2 antennas returns `EINVAL` until the antenna-switch callback is set.
  - The builder now runs support, then callback, then configuration. Measured need: 93,192 bytes
    on a TAG (the guess was 112 KiB). For one antenna the figure is 74,760, derived from 1 KiB per
    extra path per CS link.
- **CS capabilities read:** it returned *Command Disallowed* when run on the bare SDC before
  TrouBLE's init, which is what sets the CS host-support feature bit. It now goes through
  `stack.command`, which waits for init.

**probe-rs double buffering corrupts TAG flashes (DEF-16).**

- **Symptom:** every first run of a newly flashed image faulted, each time somewhere different:
  - an undefined instruction in the FICR trim loop;
  - an unaligned trap in P-256 point arithmetic;
  - a jump to `0x1e`;
  - a BusFault inside the SDC.

  Reruns of the same image booted cleanly, 7 of 7.
- **Verify fails:** with `--verify`, the first write of a new image failed verification, so the
  faults were the CPU running a mis-programmed image. SWD protocol and USB transport errors came
  up during these sessions too.
- **What goes wrong:**
  - **Method:** download the reflector and initiator builds alternately (every flash then rewrites
    most pages), read the flash back, and diff it per 4 KiB page.
  - **Damage:** with double buffering, 51–53 of 79 pages were wrong. Each bad page had exactly one
    word, at offset 0x4–0xF, replaced by its successor: one word is lost and the next lands twice.
    That looks like a retried auto-increment SWD write landing one address early, while the CPU's
    RRAM writes stall the bus (double buffering uploads the next page while the CPU programs the
    current one).
  - **Fix:** with `--disable-double-buffering`, 3 of 3 read-backs were exact. Download time is
    unchanged (21–22 s).
- **Runner:** `--disable-double-buffering --verify`. Since then, flashing alternate images
  through `cargo tag` verified and booted 3 of 3, and no SWD errors have recurred. Verify adds
  ~15 s per flash.
- **Setup:** probe-rs 0.32.0 (the latest release), the DK's onboard J-Link, TAG on its coin cell.
  The flash algorithm itself checks out: per page it enables writes, waits for READY, copies,
  commits the write buffer and waits again.

## 2026-11-01

**CS bring-up firmware** (not yet run on hardware).

- **Roles:** the build-time role picks initiator or reflector. The initiator connects at 20 ms,
  pairs, and runs Nordic's `ras_initiator` sequence and values: default settings, remote
  capabilities, create config, CS security, procedure parameters, procedure enable. Each async
  step waits for its completion event, with a 5 s timeout.
- **Capture:** both sides log raw subevents (`CSR`/`CSC` lines with HCI-layout step bytes), so the
  first measurement needs neither RAS nor inline PCT transfer.
- **Pairing crypto:** TrouBLE's `security` feature needs embassy-crypto drivers. RustCrypto
  supplies AES-128 and P-256. The RNG is ChaCha20 seeded from CRACEN at boot, because the SDC owns
  CRACEN from then on.
- **Size:** flash 193 → 323 KB (pairing crypto, central role), RAM 145 KB.
- **Cross-check:** Zephyr's own HCI struct for a CS step (`mode, channel, length, data[]`)
  independently confirms the row-major layout behind the bt-hci fix.

**Firmware on the patched forks.** All three forks are pushed and public, and match the local
`main`s. The firmware now pins `DmitriyNE/bt-hci` `4886657` (plus `bt-hci-transport` from the same
checkout), `DmitriyNE/nrf-sdc` `f705f4c` and `GreatKet/trouble` `fa9886d` (with `channel-sounding`).
The raw-FFI stopgaps are gone: CS config goes through the builder, and the capabilities read
through `LeCsReadLocalSupportedCapabilities`. Clippy is clean on both boards.

**Upstream patches, in progress** (local clones in `~/temp/ble-patches`, nothing pushed).

- **bt-hci (P-6), reviewed:** branch `fix-cs-step-data-layout`, commit `2d8d65e`. Row-major step
  parsing, validated at parse time, with tests on the real layout.
- **nrf-sdc (P-1), awaiting review:** branch `cs-hci-commands`, commits `4473bbc` (13 CS command
  bindings) and `5bc77b2` (`cs_count`, `cs_cfg`, `cs_antenna_switch_callback`). Every bt-hci
  parameter and return type was checked against its C struct with temporary compile-time size
  assertions (16 of 16 match; a deliberately wrong one fails), plus field order by eye. `LeCsTest`
  is left out for now.
- **TrouBLE:** local GreatKet `main` merged with embassy-rs `main` (`866ee71`, our pinned
  revision). It was 73 commits behind and on bt-hci 0.10.
- **TrouBLE (P-2), awaiting review:** branch `feat/channel-sounding`.
  - `bc4c6d0` fixes an existing upstream bug: the `iso` feature alone didn't compile, because the
    `LeSetHostFeature` import was gated on `subrating` only. A CI line now covers it.
  - `0eaa329` adds a `channel-sounding` feature: CS event-mask bits, eight `EventHandler` hooks,
    and the *Channel Sounding (Host Support)* feature bit (Zephyr sets it; we had missed it).
  - An end-to-end compile check (our firmware with all three patched repos and a real CS
    `EventHandler`) passed for both boards, then was reverted.
  - The fork's own blank-line commit in `peripheral.rs` fails upstream's `cargo fmt --check`. Drop
    it before an upstream PR.
- **TrouBLE: connection parameter update response identifier**, awaiting review. Branch
  `fix/conn-param-update-res-identifier`, commit `83b66a2`, separate from CS so each can go
  upstream alone.
  - The bug still applied on current upstream (`866ee71`), not only on 0.8.0: answering a peer's
    L2CAP Connection Parameter Update Request used a fresh signalling identifier instead of the
    request's, so the peer discarded it and dropped the link after 30 s.
  - `ConnectionParamsRequest` now carries the request's identifier (constructor is `pub(crate)`,
    so the public API is unchanged), and all four accept/reject send paths echo it.
  - Regression test: a request fed through `signal()` reaches the application, via
    `Connection::next()`, carrying its identifier. Breaking the fix on purpose makes it fail.
  - No upstream issue or PR existed.
- **All three local `main`s now carry the patches** (merged `--no-ff`, not pushed): bt-hci
  `4886657` (2 ahead of `origin`), nrf-sdc `f705f4c` (3 ahead), TrouBLE `fa9886d` (79 ahead).
  Our firmware builds against all three for both boards (temporary check, reverted).
- **TrouBLE local `main` = `fa9886d`:** both branches merged (`--no-ff`), 79 commits ahead of
  GreatKet `origin/main`, not pushed. The merged result passes the build, clippy and test matrix,
  and our firmware builds against it for both boards (temporary check, reverted).
- **Integration check:** our firmware builds for both boards against the patched bt-hci and
  nrf-sdc, using the new APIs in place of the raw-FFI stopgaps. That was a temporary change,
  reverted. When switching over, override `bt-hci-transport` along with `bt-hci`, or two copies of
  its traits clash.
- **A false alarm, for the record:** I briefly thought bt-hci's cached-capabilities params lacked
  `t_ip1/t_ip2`. My grep regex skipped field names containing digits; the compile-time size check
  settled it.

**Upstream patch list pinned down** (architecture §6).

- **nrf-sdc (P-1):** 14 CS command bindings, with layouts checked field by field against the C
  structs, plus CS config setters and an antenna callback setter. Events need nothing.
- **TrouBLE (P-2):** CS bits are missing from the LE event mask, and CS events have no hook.
- **New: bt-hci bug (P-6, DEF-13).** The CS step-data parser is column-major, but the SDC sends
  steps row-major. Zephyr's host feeds raw HCI step bytes to a per-step parser against the same
  SDC, and bt-hci's test uses hand-written column-major bytes. To confirm with a raw dump in P0.

**Referencing from commissioning presets (Q15).** For now the mesh is referenced from presets on
a few nodes: absolute coordinates (pins), or preset bearings between node pairs. Learning from HUD
headings moved to later.

- Added `frame::fit_bearings`. Each preset is a heading observation (mesh direction of the pair
  vs world bearing), weighted by squared baseline, which reuses the heading fit.
- Added `Frame::anchored` (origin from one pin).
- The heading fits now take iterators, which removes a fixed-size buffer.
- Tests: two non-parallel bearings give yaw and handedness exactly; one bearing gives yaw only;
  0.1 m mesh noise on 5–9 m baselines keeps yaw within 3°.

**Decisions, requirements v0.5.**

- **Heading** comes from another sensor (the HUD's magnetometer), not from tags.
- **New requirement: absolute mesh referencing** (FR-A), so mesh positions and external headings
  share a world frame.
- **Q13** (HUD details) is not relevant for now. **Q14:** the asset registry is host-only for now.
- **Monitoring** gets a thin web UI later (FR-S6).

**`navgrid_solver::frame`.**

- Mesh→world transform; pin fit (2D Kabsch, tries both handednesses, reports misfit).
- Heading-observation fit: circular mean of world − mesh heading, with handedness chosen by
  agreement and a margin reported.
- The HUD pointer.
- Unit tests: 40 straight walks with 15° compass error recover yaw within 5° and the right
  handedness. A single walking direction yields zero handedness margin, as it should. A
  self-surveyed mesh cannot tell its mirror image without varied directions or 3 pins.

**Repo rules and initial commit.** Commit messages are terse, 1–2 sentences, with no trailers or
prefixes (see `CLAUDE.md`). Engineering records live here. The scaffold went in as the initial
commit on `main`.

**Decisions, requirements v0.4.**

- **Battery life:** 1 month is the goal, 1 week is acceptable. No year-long squeeze. At ≈ 310 µA
  average, the gyro on person tags is affordable while moving.
- **Monitoring:** our own small tool (`navgrid monitor`, terminal view plus JSON-lines log).
- **Tags** go on assets and people. Driving use case: **find my keys** with a HUD pointer. That
  makes person heading (no magnetometer: gyro aligned to the walking direction) and a direct
  person↔asset final approach first-class (requirements FR-F, architecture §10).
- **Roles:** `Role` is now Node / Asset / Person / Sink, and the default advertising interval is
  back to 1 s.

**Tracking and height**
([spike](spikes/2026-11-01-tracking-height-imu.md)).

- Added the one-measurement-at-a-time `Tracker` and `navgrid track`.
- Barometry makes 3D viable under multipath: p90 2.4 m → 1.0 m.
- IMU gating saves 36 % of procedures at the same accuracy, once there is a settling burst.
- The first result was misleading: I suspected filter overconfidence and added motion-onset
  handling, but the real cause was the missing settling burst and an unfair comparison.

**Decisions, requirements v0.3.**

- Power source: CR2032.
- 3D, using differential barometry from the BME688 (it is on every TAG).
- IMU in tiers by energy cost.
- Monitoring of mesh state plus sensor sideband.

**TAG board support.** Found and handled two gaps:

- **embassy-nrf has no nRF54L DC/DC or crystal load-capacitor config.** `board.rs` ports Zephyr's
  formula using FICR trims; the values come from the boards' devicetrees.
- **TAG antenna switch:** SKY13348 on P1.09/P1.10, one line per antenna. It is driven from the
  SDC's antenna callback; antenna count and path limit go through raw `sdc_cfg_set`, because the
  nrf-sdc builder lacks setters. Diversity can be turned off per device.

**Antenna-path combining**
([spike](spikes/2026-11-01-antenna-path-combining.md)). Diversity cuts p90 by ~40 % under moderate
multipath. Picking the lowest-residual path is *not* better on average. Residual-weighted is the
default.

**Ranging quality metric.** Adjacent-channel coherence is blind to multipath: a +1.8 m bias showed
coherence 0.9998. Switched quality to the phase-fit residual.

**Rust CS stack feasibility** ([spike](spikes/2026-11-01-rust-cs-stack.md)). Pure Rust works.
CS needs `bt-hci` 0.11, so TrouBLE and nrf-sdc are pinned to git revisions. Gaps: nrf-sdc CS
bindings (P-1), TrouBLE CS event hook (P-2), our own estimator (P-4).

**Toolchain.** The rustup auto-install of 1.97 (pinned in `rust-toolchain.toml`) left the clippy
component registered but without binaries; parallel cargo runs during the install are the likely
cause. Fixed with `rustup component remove/add clippy --toolchain 1.97`.

**Requirements captured (v0.1).**

- Peer-to-peer CS mesh of nRF54L15 nodes that self-surveys.
- Tags located against it, solving on-device, with optional central collection.
- Building scale; Rust for firmware and host.
