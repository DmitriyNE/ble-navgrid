//! The node's replica of the mesh state (architecture §12, `navgrid-mesh`): its own records (the
//! node record, health, the links it ranges), its 2-hop neighbourhood's, the controller's for
//! them, and the outbox toward sinks. Sessions over the mesh links (`mesh`) keep it in step with
//! the neighbours' replicas; one session runs at a time, its driver kept here beside the
//! replica. A sink mirrors every record it keeps to the host as `Message::Record`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use navgrid_mesh::wire::Header;
use navgrid_mesh::{
    Arena, Body, CONTROLLER, Command, Config, Device, Driver, Health, Key, Lent, NodeId, NodeInfo, Pending, Received,
    Record, Replica, Reported, Status, Stored, Topic, auth,
};
use navgrid_proto::{MESH_RECORD_LEN, MeshRecord, Message};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;

use crate::{report, status, update};

/// The tables a node keeps, as static arenas of encoded records (about 50 B each, DEF-29): the
/// 2-hop neighbourhood at degree 6 peaks at about 110 records with 20 nodes, 141 with 500, and
/// the outbox at 74 in custody (architecture §12). A sink keeps every record of its mesh: about
/// 1100 at 500 nodes with 4 sinks, so a DK sink's tables serve a mesh of a few hundred nodes.
#[cfg(feature = "board-dk")]
mod tables {
    pub const RECORD_BYTES: usize = 24 * 1024;
    pub const RECORDS: usize = 480;
    pub const OUTBOX_BYTES: usize = 6 * 1024;
    pub const OUTBOX: usize = 80;
    pub const ROUTES: usize = 128;
}
#[cfg(not(feature = "board-dk"))]
mod tables {
    pub const RECORD_BYTES: usize = 16 * 1024;
    pub const RECORDS: usize = 320;
    pub const OUTBOX_BYTES: usize = 6 * 1024;
    pub const OUTBOX: usize = 80;
    pub const ROUTES: usize = 64;
}
pub type Store = Device<
    { tables::RECORD_BYTES },
    { tables::RECORDS },
    { tables::OUTBOX_BYTES },
    { tables::OUTBOX },
    { tables::ROUTES },
>;

// The tables live in `.bss`, not in the replica, which is built on the stack at boot.
static mut RECORDS: Arena<Stored, { tables::RECORD_BYTES }, { tables::RECORDS }> = Arena::new();
static mut OUTBOX: Arena<Pending, { tables::OUTBOX_BYTES }, { tables::OUTBOX }> = Arena::new();
static mut ROUTES: heapless::Vec<navgrid_mesh::Route, { tables::ROUTES }> = heapless::Vec::new();
static TABLES_LENT: AtomicBool = AtomicBool::new(false);

struct State {
    replica: Replica<Store>,
    /// The session under way, if any: one at a time.
    session: Option<Driver>,
    /// The controller's key, to check desired state against (DEF-27).
    controller_key: Option<[u8; 16]>,
}

/// The state, once [`init`] has built it in `STATE_STORE`. (An `Option<State>` here would be
/// 50 KiB of `.data`, copied from flash at boot, as `None` isn't all zeros.)
static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<&'static mut State>>> = Mutex::new(RefCell::new(None));
static mut STATE_STORE: core::mem::MaybeUninit<State> = core::mem::MaybeUninit::uninit();

/// Between health records.
const HEALTH_EVERY: Duration = Duration::from_secs(30 * 60);
/// Between a sink's full mirrors to the host.
const MIRROR_EVERY: Duration = Duration::from_secs(5 * 60);
const TICK: Duration = Duration::from_secs(60);
/// How often the node looks for commands it hasn't acted on.
const COMMANDS_EVERY: Duration = Duration::from_secs(5);

pub fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Starts the replica at boot, once.
pub fn init(node: NodeId, sink: bool, epoch: u32, info: NodeInfo) {
    assert!(!TABLES_LENT.swap(true, Ordering::AcqRel), "the mesh state starts once");
    // SAFETY: the statics are named only here, and this runs once (checked above), so these are
    // the only references to them, for the rest of the run.
    let (records, outbox, routes) = unsafe {
        (
            Lent(&mut *core::ptr::addr_of_mut!(RECORDS)),
            Lent(&mut *core::ptr::addr_of_mut!(OUTBOX)),
            Lent(&mut *core::ptr::addr_of_mut!(ROUTES)),
        )
    };
    let replica = Replica::with_tables(
        Config::new(node, CONTROLLER, sink),
        epoch,
        info,
        now_ms(),
        records,
        outbox,
        routes,
    );
    // The controller's key goes in with `set_controller_key`, once the fleet key is read.
    if sink {
        for s in replica.records() {
            mirror(s.record());
        }
    }
    // SAFETY: the store is named only here, once (TABLES_LENT above); the fields are written in
    // place, each once, before the reference is handed out.
    let state: &'static mut State = unsafe {
        let p = core::ptr::addr_of_mut!(STATE_STORE) as *mut State;
        core::ptr::addr_of_mut!((*p).replica).write(replica);
        core::ptr::addr_of_mut!((*p).session).write(None);
        core::ptr::addr_of_mut!((*p).controller_key).write(None);
        &mut *p
    };
    STATE.lock(|s| *s.borrow_mut() = Some(state));
    status::set_hops(if sink { 0 } else { u8::MAX });
}

pub fn set_controller_key(key: Option<[u8; 16]>) {
    STATE.lock(|s| {
        if let Some(s) = s.borrow_mut().as_mut() {
            s.controller_key = key;
            s.replica.config_mut().controller_key = key;
        }
    });
}

/// On a sink: takes a record from the host (the controller's desired state) into the replica
/// and on toward its target, and mirrors it back so the host sees it went in.
#[cfg_attr(not(feature = "board-dk"), allow(dead_code))]
pub fn inject(bytes: &[u8]) {
    let Ok(record) = postcard::from_bytes::<Record>(bytes) else {
        warn!("mesh state: an undecodable record from the host");
        return;
    };
    let now = now_ms();
    let result = with(|r| r.inject(record.clone(), now));
    match result {
        Some(Ok(rx)) => {
            info!("mesh state: record from the host: {:?}", rx);
            mirror(&record);
        }
        Some(Err(e)) => warn!("mesh state: record from the host refused: {:?}", e),
        None => {}
    }
}

/// Acts on the controller's desired state for this node that it hasn't acted on yet, checking
/// each record's MAC, and writes a Reported record per setting. Returns how many it applied.
pub fn apply_commands() -> usize {
    let now = now_ms();
    let mut pending: heapless::Vec<(navgrid_mesh::Version, Command, bool), 8> = heapless::Vec::new();
    STATE.lock(|s| {
        let s = s.borrow();
        let Some(s) = s.as_ref() else {
            return;
        };
        let me = s.replica.id();
        for (version, command) in s.replica.pending_commands() {
            let key = Key {
                origin: CONTROLLER,
                topic: Topic::Desired(me, command.setting()),
            };
            let genuine = match (s.controller_key, s.replica.get(&key)) {
                (Some(k), Some(st)) => auth::verify(&k, st.record()),
                _ => false,
            };
            let _ = pending.push((version, command, genuine));
        }
    });
    let mut applied = 0;
    for (version, command, genuine) in pending {
        let status = if !genuine {
            warn!(
                "command {:?} v{}.{}: not the controller's; ignored",
                command, version.epoch, version.counter
            );
            Status::Invalid
        } else {
            match command {
                Command::Blink { times } => {
                    info!("command: blink {=u8} times", times);
                    status::attention();
                    Status::Applied
                }
                Command::Diversity(on) => {
                    let antennas = if on { crate::board::ANTENNAS } else { 1 };
                    info!(
                        "command: diversity {}: {=u8} antenna(s) from the next burst",
                        on, antennas
                    );
                    status::set_antennas(antennas);
                    Status::Applied
                }
                Command::Advertising { interval_ms } => {
                    if (20..=10_240).contains(&interval_ms) {
                        info!("command: advertising every {=u16} ms", interval_ms);
                        status::set_adv_interval_ms(interval_ms);
                        Status::Applied
                    } else {
                        warn!("command: advertising interval {=u16} ms is out of range", interval_ms);
                        Status::Invalid
                    }
                }
                Command::Environment { .. } => {
                    warn!("command {:?}: no BME688 driver yet", command);
                    Status::Unsupported
                }
                Command::Firmware(firmware) => {
                    let news = update::want(firmware);
                    if firmware == crate::FIRMWARE {
                        info!("command: firmware {}: running it", firmware);
                        Status::Applied
                    } else if crate::firmware::reverted_from() == Some(firmware) {
                        warn!("command: firmware {}: this node reverted from it", firmware);
                        Status::Invalid
                    } else {
                        // Pending until an offer of it is taken and the node runs it; the record
                        // comes round again every few seconds meanwhile.
                        if news {
                            info!("command: firmware {}: taking offers of it only", firmware);
                        }
                        continue;
                    }
                }
            }
        };
        if status == Status::Applied {
            applied += 1;
        }
        write(Body::Reported(Reported {
            setting: command.setting(),
            applied: version,
            status,
        }));
    }
    let _ = now;
    applied
}

/// Runs `f` on the replica; `None` before [`init`].
pub fn with<R>(f: impl FnOnce(&mut Replica<Store>) -> R) -> Option<R> {
    STATE.lock(|s| s.borrow_mut().as_mut().map(|s| f(&mut s.replica)))
}

/// Writes one of this node's own records.
pub fn write(body: Body) {
    let now = now_ms();
    let (sink, written) = STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return (false, None);
        };
        let sink = s.replica.is_sink();
        match s.replica.write(body.clone(), now) {
            Ok(_) => {
                let key = navgrid_mesh::Key {
                    origin: s.replica.id(),
                    topic: body.topic(),
                };
                (sink, s.replica.get(&key).map(|st| st.record().clone()))
            }
            Err(e) => {
                warn!("mesh state: write refused: {:?}", e);
                (sink, None)
            }
        }
    });
    if let (true, Some(record)) = (sink, written) {
        mirror(&record);
    }
}

/// The links this node syncs with changed.
pub fn set_neighbours(neighbours: impl Iterator<Item = NodeId>) {
    let mut list: heapless::Vec<NodeId, { navgrid_mesh::MAX_NEIGHBOURS }> = heapless::Vec::new();
    for n in neighbours {
        let _ = list.push(n);
    }
    list.sort_unstable();
    let now = now_ms();
    let hops = with(|r| {
        if let Err(e) = r.set_node(|n| n.neighbours = list, now) {
            warn!("mesh state: node record: {:?}", e);
        }
        r.hops()
    });
    if let Some(hops) = hops {
        status::set_hops(hops);
    }
}

/// This node's header for a session with `peer`: the replica poisons the hop count toward the
/// peer its only sink path runs through (DEF-30).
pub fn header(role: navgrid_proto::Role, sink: bool, peer: NodeId) -> Header {
    with(|r| r.header_for(peer)).map_or(
        Header {
            node: NodeId(0),
            role,
            hops: u8::MAX,
            sink,
            delta: false,
        },
        |h| Header { role, sink, ..h },
    )
}

/// Takes the session slot with a fresh driver for a session with `peer`, remembering their last
/// exchange if there was one (DEF-28), or `false` while another session runs.
pub fn begin_session(mine: Header, budget: u32, peer: NodeId) -> bool {
    STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return false;
        };
        if s.session.is_some() {
            return false;
        }
        let memory = s
            .replica
            .memory(peer)
            .map(|(sent, received)| (sent.iter().copied().collect(), received.iter().copied().collect()));
        s.session = Some(Driver::new(mine, budget, memory));
        true
    })
}

/// Frees the session slot; the driver's peer header, if it got one. A complete session is
/// remembered for the next one's delta; one that broke off is forgotten.
pub fn end_session(peer: NodeId) -> Option<Header> {
    let (hops, theirs) = STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return (u8::MAX, None);
        };
        let theirs = match s.session.take() {
            Some(d) if d.is_complete() => {
                let (sent, received) = d.memory_to_keep();
                s.replica.remember(peer, sent, received);
                d.peer()
            }
            Some(d) => {
                s.replica.forget(peer);
                d.peer()
            }
            None => None,
        };
        (s.replica.hops(), theirs)
    });
    status::set_hops(hops);
    theirs
}

/// The session's next frame to send, if any (`None` also when no session runs).
/// Encodes the session's next frame into `out`, giving its length, or `None` when the driver
/// has nothing to send now. (The message itself stays on the stack, under the lock: a `Sync`
/// is most of a kilobyte, too much for each link's future to hold.)
pub fn session_out_frame(out: &mut [u8]) -> Result<Option<usize>, navgrid_mesh::wire::Error> {
    let now = now_ms();
    STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return Ok(None);
        };
        let State { replica, session, .. } = s;
        let Some(msg) = session.as_mut().and_then(|d| d.next_out(replica, now)) else {
            return Ok(None);
        };
        navgrid_mesh::wire::encode(&msg, out).map(|frame| Some(frame.len()))
    })
}

/// What went wrong with a frame from the peer.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum FrameError {
    Decode,
    Protocol(navgrid_mesh::Protocol),
}

/// A frame from the session's peer, decoded in place and fed to the driver. A sink mirrors what
/// it keeps to the host.
pub fn session_in_frame(frame: &mut [u8]) -> Result<(), FrameError> {
    let now = now_ms();
    let mut kept: heapless::Vec<Record, 4> = heapless::Vec::new();
    let result = STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return Err(FrameError::Protocol(navgrid_mesh::Protocol::OutOfOrder));
        };
        let State { replica, session, .. } = s;
        let sink = replica.is_sink();
        let Some(driver) = session.as_mut() else {
            return Err(FrameError::Protocol(navgrid_mesh::Protocol::OutOfOrder));
        };
        let msg = navgrid_mesh::wire::decode(frame).map_err(|_| FrameError::Decode)?;
        driver
            .on_frame(replica, msg, now, |rec, rx| {
                if sink && rx.kept() {
                    let _ = kept.push(rec.clone());
                }
            })
            .map_err(FrameError::Protocol)
    });
    for rec in &kept {
        mirror(rec);
    }
    result
}

pub fn session_complete() -> bool {
    STATE.lock(|s| {
        s.borrow()
            .as_ref()
            .and_then(|s| s.session.as_ref())
            .is_some_and(Driver::is_complete)
    })
}

pub fn session_side() -> Option<navgrid_mesh::session::Side> {
    STATE.lock(|s| s.borrow().as_ref().and_then(|s| s.session.as_ref()).map(Driver::side))
}

/// A sink's copy of a record for the host.
fn mirror(record: &Record) {
    let mut bytes = heapless::Vec::<u8, MESH_RECORD_LEN>::new();
    let _ = bytes.resize_default(MESH_RECORD_LEN);
    match postcard::to_slice(record, &mut bytes) {
        Ok(used) => {
            let len = used.len();
            bytes.truncate(len);
            report::queue(Message::Record(MeshRecord { bytes }));
        }
        Err(_) => warn!("mesh state: a record doesn't fit a frame"),
    }
}

/// Ages the replica, acts on the controller's commands, writes health now and then, and on a
/// sink mirrors everything to the host every few minutes, in case a frame was lost.
pub async fn run(mpsl: &MultiprotocolServiceLayer<'_>) -> ! {
    let mut last_health = Instant::now();
    let mut last_mirror = Instant::now();
    let mut last_tick = Instant::now();
    loop {
        Timer::after(COMMANDS_EVERY).await;
        apply_commands();
        if last_tick.elapsed() < TICK {
            continue;
        }
        last_tick = Instant::now();
        let now = now_ms();
        let (sink, hops, position) = with(|r| {
            r.tick(now);
            let me = r.id();
            let position = r
                .get(&Key {
                    origin: CONTROLLER,
                    topic: Topic::Position(me),
                })
                .and_then(|s| match &s.record().body {
                    Body::Position(p) => Some((p.point, *p.sigma_mm.iter().max().unwrap_or(&0))),
                    _ => None,
                });
            (r.is_sink(), r.hops(), position)
        })
        .unwrap_or((false, u8::MAX, None));
        status::set_hops(hops);
        status::set_position(position);
        if !status::retired() && with(|r| r.is_retired(r.id())).unwrap_or(false) {
            warn!("retired by the controller: no more ranging or syncing");
            status::retire();
        }
        self_fix();
        if last_health.elapsed() >= HEALTH_EVERY {
            last_health = Instant::now();
            let t = mpsl.get_temperature();
            let (bursts, procedures, sessions) = status::work();
            write(Body::Health(Health {
                battery_mv: status::battery_mv().unwrap_or(0),
                temperature_cdeg: (t.raw() * 25) as i16,
                uptime_s: Instant::now().as_secs() as u32,
                queued: with(|r| r.outbox().len() as u16).unwrap_or(0),
                bursts,
                procedures,
                sessions,
            }));
        }
        if sink && last_mirror.elapsed() >= MIRROR_EVERY {
            last_mirror = Instant::now();
            let n = with(|r| {
                for s in r.records() {
                    mirror(s.record());
                }
                r.records().len()
            })
            .unwrap_or(0);
            info!("mesh state: mirrored {=usize} records to the host", n);
        }
    }
}

/// Distributed solving, first cut (FR-M2): an anchor without a pin places itself from its own
/// links to neighbours the controller has placed or that are pinned, held near their height,
/// and writes the fix as its own Position record. The controller's solve, when there is one, is
/// what the beacon advertises; this fix stands in where there is none, and lets the host
/// compare the two (D-28).
fn self_fix() {
    use navgrid_solver::{Height, Options, RangeObs, multilaterate};
    let now = now_ms();
    let fix = with(|r| {
        let me = r.id();
        let own = r.node();
        if own.role != navgrid_proto::Role::Anchor || own.pinned.is_some() {
            return None;
        }
        let position_of = |peer: NodeId| -> Option<([f32; 3], f32)> {
            let pinned = r
                .get(&Key {
                    origin: peer,
                    topic: Topic::Node,
                })
                .and_then(|s| match &s.record().body {
                    Body::Node(n) => n.pinned.map(|p| (p, 100u32)),
                    _ => None,
                });
            let solved = r
                .get(&Key {
                    origin: CONTROLLER,
                    topic: Topic::Position(peer),
                })
                .and_then(|s| match &s.record().body {
                    Body::Position(p) => Some((p.point, *p.sigma_mm.iter().max().unwrap_or(&0))),
                    _ => None,
                });
            let (p, sigma_mm) = pinned.or(solved)?;
            Some((
                [p.x as f32 / 1000.0, p.y as f32 / 1000.0, p.z as f32 / 1000.0],
                sigma_mm as f32 / 1000.0,
            ))
        };
        let mut obs: heapless::Vec<RangeObs, { navgrid_mesh::MAX_NEIGHBOURS }> = heapless::Vec::new();
        let mut z_sum = 0.0f32;
        for s in r.records() {
            let rec = s.record();
            if rec.origin != me {
                continue;
            }
            let Body::Link(l) = &rec.body else {
                continue;
            };
            if s.age_ms(now) > 30 * 60 * 1000 {
                continue;
            }
            let Some((anchor, _)) = position_of(l.peer) else {
                continue;
            };
            z_sum += anchor[2];
            let _ = obs.push(RangeObs {
                anchor,
                distance_m: (l.distance_mm as f32 / 1000.0 - LINK_OFFSET_M).max(0.0),
                sigma_m: (l.sigma_mm as f32 / 1000.0).max(0.3),
            });
        }
        if obs.len() < 3 {
            return None;
        }
        let z = z_sum / obs.len() as f32;
        let mut initial = [0.0f32; 3];
        for o in &obs {
            for (ci, ai) in initial.iter_mut().zip(o.anchor) {
                *ci += ai / obs.len() as f32;
            }
        }
        initial[0] += 0.5;
        initial[1] += 0.3;
        initial[2] = z;
        multilaterate(
            &obs,
            initial,
            &Options {
                height: Height::Observed { z, sigma_m: 0.3 },
                ..Options::default()
            },
        )
        .map(|f| (me, f))
    })
    .flatten();
    let Some((me, f)) = fix else {
        return;
    };
    info!(
        "self fix: {=f32} {=f32} {=f32} m, sigma {=f32} {=f32} {=f32}",
        f.position[0], f.position[1], f.position[2], f.sigma_m[0], f.sigma_m[1], f.sigma_m[2]
    );
    let mm = |v: f32| (v * 1000.0) as i32;
    let smm = |v: f32| (v.max(0.0) * 1000.0) as u32;
    write(Body::Position(navgrid_mesh::Position {
        of: me,
        point: navgrid_proto::PointMm {
            x: mm(f.position[0]),
            y: mm(f.position[1]),
            z: mm(f.position[2]),
        },
        sigma_mm: [smm(f.sigma_m[0]), smm(f.sigma_m[1]), smm(f.sigma_m[2])],
        frame: 1,
    }));
}

/// Taken off a link's distance before it places a node: the chip's delay, until the calibration
/// (DEF-6) puts the right figure on the device.
const LINK_OFFSET_M: f32 = 1.5;

/// How a session went, for the log.
pub fn log_session(peer: NodeId, side: Option<navgrid_mesh::session::Side>, took_ms: u64) {
    match side {
        Some(s) => info!(
            "session with {=u64:012x}: {=u32} frames, {=u32} B sent, took {=u32}, refused {=u32}, held back {}, in {=u64} ms",
            peer.0,
            s.sent.frames,
            s.sent.total(),
            s.took,
            s.refused,
            s.held_back,
            took_ms
        ),
        None => info!("session with {=u64:012x}: no driver", peer.0),
    }
}

#[allow(dead_code)]
fn _received_is_used(r: Received) -> bool {
    r.kept()
}
