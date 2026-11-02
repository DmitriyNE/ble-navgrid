# Mesh state protocol in simulation

**Question.** Does the mesh state design (architecture §12) converge, and what does it cost? At
20 and 500 nodes:
- how stale replicas get;
- how long commands take;
- bytes per contact and memory per node;
- what a partition does.

## Setup

- **Code:** the `navgrid-mesh` crate and `navgrid mesh-sim`, as committed with this spike. A
  session is the in-memory `navgrid_mesh::session`, the same steps devices will run over L2CAP.
- **Deployment:**
  - nodes on a jittered square grid with 6 m spacing;
  - mesh links between nodes within 13 m, nearest pairs first, at most 6 per node;
  - sinks at the nodes nearest to evenly spaced points on the middle row.
- **Traffic, per node:**
  - a sync session on every link every 60 s (`--contact`);
  - health every 30 min and environment every 10 min;
  - a link record from the initiator after each ranging burst, every 15 min;
  - a node record whenever its hops or neighbours change, and hourly.
- **Controller:**
  - writes the frame at the start, and every node's position at 30 min and then hourly;
  - blinks a random node every 5 min and retries every 15 min until acknowledged;
  - hands each record to the sinks that have a route to its node.
- **Limits:** `--budget` bytes of records and pushes per side per session (summaries and
  versions uncapped). Tables are effectively unlimited, so peaks are measured.
- **Staleness:** sampled about every 5 min after the first hour. For each record key it is how
  long the replica has lacked the newest version; 0 means current.
  - *Neighbourhood:* every node's node, link and environment records, at each node within 2 hops.
  - *Positions:* the controller's position of each node, at the node and its neighbourhood.
  - *Host:* every node record, at the union of the sinks.
- **Not modelled:** radio loss, link drops mid-session (sessions are atomic), clock drift,
  reboots, moving nodes, tags, energy.

## How to reproduce

```sh
cargo build --release -p navgrid
./target/release/navgrid mesh-sim --nodes 20
./target/release/navgrid mesh-sim --nodes 500 --sinks 4
./target/release/navgrid mesh-sim --nodes 500 --sinks 4 --partition 2-3
```

Each run is 6 simulated hours with seed 1, and takes under 10 s.

## Results

At 60 s contacts unless noted. "Current" is the share of samples at lag 0. Bytes are per side
per session, at p50.

| Run | Neighbourhood current, p99 | Host current, p99 | Command ack p50 / p90 | Bytes | Sent per node | Node memory p50 / max | Largest relay outbox |
|---|---|---|---|---|---|---|---|
| 20 nodes, 1 sink | 96.8 %, 22 s | 97.1 %, 23 s | 94 s / 2.8 min | 501 B | 185 kB/h | 12.6 / 16.6 KiB | 14 |
| 20 nodes, 300 s contacts | 85.9 %, 3.8 min | 86.9 %, 4.5 min | 8.4 / 11.2 min | 765 B | 55 kB/h | 12.7 / 17.1 KiB | 17 |
| 500 nodes, 1 sink | 96.6 %, 31 s | 58.1 %, 19.9 min | 7.8 / 11.7 min | 645 B | 249 kB/h | 17.5 / 188.7 KiB | 1,175 |
| 500 nodes, 4 sinks | 96.6 %, 32 s | 86.7 %, 3.5 min | 4.8 / 9.1 min | 644 B | 248 kB/h | 17.2 / 29.2 KiB | 72 |
| … seed 2 | 96.8 %, 30 s | 87.1 %, 3.4 min | 5.2 / 8.8 min | 640 B | 247 kB/h | 17.1 / 32.4 KiB | 98 |
| 500 nodes, 8 sinks | 96.6 %, 31 s | 88.1 %, 3.2 min | 4.8 / 8.5 min | 644 B | 249 kB/h | 17.2 / 34.1 KiB | 98 |
| 500 nodes, 4 sinks, budget 1 KB | 96.6 %, 32 s | 85.8 %, 3.9 min | 4.8 / 9.1 min | 644 B | 248 kB/h | 17.2 / 33.9 KiB | 110 |
| 500 nodes, 4 sinks, 300 s | 84.1 %, 4.4 min | 41.7 %, 32.2 min | 28.3 / 44.9 min | 1,006 B | 75 kB/h | 17.9 / 53.6 KiB | 250 |
| … budget 8 KB | 84.1 %, 4.4 min | 44.5 %, 27.9 min | 28.3 / 44.9 min | 1,006 B | 75 kB/h | 17.9 / 52.4 KiB | 241 |

The 2 KB budget held records back in 0.01 % of sessions at 60 s with 4 or 8 sinks, and in
0.17 % with one sink. At 300 s contacts, 4 of the 72 commands were still in flight when the run
ended.

**Partition:** every link between the left and right halves cut from hour 2 to hour 3.

| Run | After the heal, back to normal | Commands |
|---|---|---|
| 20 nodes, 1 sink | neighbourhood 5.9 min, positions 5.9 min, host 12.0 min | Those to the far side waited out the cut: ack max 46 min. |
| 500 nodes, 4 sinks | neighbourhood 9.4 min, positions 4.2 min, host 4.2 min | With a sink on each side, unaffected. |

Outside the cut and the hour after it, staleness matched the uncut runs.

**Other figures:**

- **Traffic split at 60 s:** summaries 78–79 %, versions 11–12 %, records 4–5 %, pushes 1–2 %,
  control 4–5 %; 10–11 frames per side per session. At 300 s, summaries 51–53 % and versions
  28–29 %.
- **Records per node, peak:** 111 at 20 nodes, 141–143 at 500. Routes at nodes: at most 76 at
  500 nodes.
- **Sinks:**
  - 20 nodes: 173 records, 26 KiB;
  - 500 nodes with 4 sinks: 924–1,179 records each, 145–184 KiB;
  - 500 nodes with one sink: 3,594 records, 558 KiB.
- **A record in RAM takes 136 B,** about three times its wire size: the node record's eight 64-bit
  neighbour IDs size the record enum.

## Protocol changes found while building it

§12's first draft did not survive contact with the details:

- **Digests:** the draft had a per-origin digest of the highest sequence held. That breaks once
  relays drop records they have forwarded and paths merge: a replica can then claim records it
  never got. It is replaced by two mechanisms:
  - **neighbourhood sync:** an order-free hash per class (records with one subject and one
    scope), then version lists only for the classes whose hashes differ;
  - **custody pushes:** toward sinks and toward targets, dropped from the outbox only once the
    receiver accepts them.
- **Outbox order:** strict priority starved health and environment behind links at congested
  relays: host lag reached 6 h at 300 s contacts. The fix sends commands and reports first, then
  the rest oldest first, and a rewritten key keeps its place in the queue. The worst host lag in
  that run fell to 111 min.
- **Injection:** handing controller records to every sink made each sink queue all 500
  positions. Handing them only to the sinks with a route to the node keeps sink outboxes at
  140–170.

## Conclusion

1. **The design converges.** At 60 s contacts, neighbourhood replicas are current about 97 % of
   the time at any mesh size. A one-hour partition heals in 4–12 min, and commands to a cut-off
   node complete after the heal.
2. **Per-node cost is set by degree, not mesh size,** as D-25 intended. From 20 to 500 nodes,
   records went from 111 to 143 and bytes per session from 500 to 650 B.
3. **One sink cannot carry 500 nodes.** The relay next to it queues 1,175 records (about 190 KiB),
   and the host falls minutes behind. Four sinks, one per ~125 nodes, keep every relay's outbox
   under about 110 records; eight add little. Latency is then hops × contact period.
4. **Summaries are most of the traffic,** about 80 % at 60 s contacts.
   - **Volume:** about 250 kB/h sent per node, and about as much received, which is ~140 B/s.
   - **Energy, estimated:** at 2M PHY (4 µs per byte) that is ~0.6 ms/s of payload airtime. At
     an assumed ~5 mA radio current, that is ~3 µA, about 1 % of NFR-6's 310 µA. This leaves out
     per-packet overhead and the connection events the link holds anyway; stage 3 measures it.
   - **The obvious saving** is delta summaries: send only the classes changed since the last
     complete session with that peer (DEF-28).
5. **The contact period is the latency knob.**
   - At 60 s, a command round trip is 1.6 min at p50 with 20 nodes, and 4.8 min with 500.
   - At 300 s it is 8.4 and 28 min, for 3–4 times less traffic.
6. **Budget: 2 KB per side per session at 60 s contacts** is the starting point for firmware.
   Halving it changed little. At 300 s contacts the hop latency dominates, and 8 KB only trims
   the host's tail.
7. **§12's size estimates:**
   - **Wire size holds:** 20–40 B per record.
   - **RAM is higher:** a record costs 136 B, so a node needs 13–17 KiB at p50 and up to ~34 KiB
     at the busiest relays (500 nodes, 4 sinks). A compact device layout would cut that by about
     a third (DEF-29).
   - **Sinks:** a 20-node sink fits a DK. At 500 nodes, each of 4 sinks holds 145–185 KiB, so
     sinks there lean on the host.
