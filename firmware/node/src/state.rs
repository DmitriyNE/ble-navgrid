//! The node's replica of the mesh state (architecture §12, `navgrid-mesh`): its own records (the
//! node record, health, the links it ranges), its 2-hop neighbourhood's, the controller's for
//! them, and the outbox toward sinks. Sessions over the mesh links (`mesh`) keep it in step with
//! the neighbours' replicas; one session runs at a time, its driver kept here beside the
//! replica. A sink mirrors every record it keeps to the host as `Message::Record`.

use core::cell::RefCell;

use defmt::{info, warn};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use navgrid_mesh::wire::Header;
use navgrid_mesh::{Body, CONTROLLER, Config, Driver, Fixed, Health, NodeId, NodeInfo, Received, Record, Replica};
use navgrid_proto::{MESH_RECORD_LEN, MeshRecord, Message};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;

use crate::{report, status};

/// Records, outbox entries and routes a node keeps: the 2-hop neighbourhood at degree 6 peaks at
/// about 110 records with 20 nodes, 143 with 500 (architecture §12).
pub type Store = Fixed<160, 32, 48>;

struct State {
    replica: Replica<Store>,
    /// The session under way, if any: one at a time.
    session: Option<Driver>,
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<State>>> = Mutex::new(RefCell::new(None));

/// Between health records.
const HEALTH_EVERY: Duration = Duration::from_secs(30 * 60);
/// Between a sink's full mirrors to the host.
const MIRROR_EVERY: Duration = Duration::from_secs(5 * 60);
const TICK: Duration = Duration::from_secs(60);

pub fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Starts the replica at boot.
pub fn init(node: NodeId, sink: bool, epoch: u32, info: NodeInfo) {
    let replica = Replica::new(Config::new(node, CONTROLLER, sink), epoch, info, now_ms());
    if sink {
        for s in replica.records() {
            mirror(s.record());
        }
    }
    STATE.lock(|s| *s.borrow_mut() = Some(State { replica, session: None }));
    status::set_hops(if sink { 0 } else { u8::MAX });
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

/// Hops to the nearest sink, as the replica knows them.
pub fn hops() -> u8 {
    with(|r| r.hops()).unwrap_or(u8::MAX)
}

/// This node's header for a session.
pub fn header(role: navgrid_proto::Role, sink: bool) -> Header {
    Header {
        node: with(|r| r.id()).unwrap_or(NodeId(0)),
        role,
        hops: hops(),
        sink,
    }
}

/// Takes the session slot with a fresh driver, or `false` while another session runs.
pub fn begin_session(mine: Header, budget: u32) -> bool {
    STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return false;
        };
        if s.session.is_some() {
            return false;
        }
        s.session = Some(Driver::new(mine, budget));
        true
    })
}

/// Frees the session slot; the driver's peer header, if it got one.
pub fn end_session() -> Option<Header> {
    let (hops, peer) = STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return (u8::MAX, None);
        };
        let peer = s.session.take().and_then(|d| d.peer());
        (s.replica.hops(), peer)
    });
    status::set_hops(hops);
    peer
}

/// The session's next frame to send, if any (`None` also when no session runs).
pub fn session_out() -> Option<navgrid_mesh::wire::Sync> {
    let now = now_ms();
    STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let s = s.as_mut()?;
        let State { replica, session } = s;
        session.as_mut()?.next_out(replica, now)
    })
}

/// A frame from the session's peer. A sink mirrors what it keeps to the host.
pub fn session_in(msg: navgrid_mesh::wire::Sync) -> Result<(), navgrid_mesh::Protocol> {
    let now = now_ms();
    let mut kept: heapless::Vec<Record, 4> = heapless::Vec::new();
    let result = STATE.lock(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return Err(navgrid_mesh::Protocol::OutOfOrder);
        };
        let State { replica, session } = s;
        let sink = replica.is_sink();
        let Some(driver) = session.as_mut() else {
            return Err(navgrid_mesh::Protocol::OutOfOrder);
        };
        driver.on_frame(replica, msg, now, |rec, rx| {
            if sink && rx.kept() {
                let _ = kept.push(rec.clone());
            }
        })
    });
    for rec in &kept {
        mirror(rec);
    }
    result
}

pub fn session_complete() -> bool {
    STATE.lock(|s| s.borrow().as_ref().and_then(|s| s.session.as_ref()).is_some_and(Driver::is_complete))
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

/// Ages the replica, writes health now and then, and on a sink mirrors everything to the host
/// every few minutes, in case a frame was lost.
pub async fn run(mpsl: &MultiprotocolServiceLayer<'_>) -> ! {
    let mut last_health = Instant::now();
    let mut last_mirror = Instant::now();
    loop {
        Timer::after(TICK).await;
        let now = now_ms();
        let (sink, hops) = with(|r| {
            r.tick(now);
            (r.is_sink(), r.hops())
        })
        .unwrap_or((false, u8::MAX));
        status::set_hops(hops);
        if last_health.elapsed() >= HEALTH_EVERY {
            last_health = Instant::now();
            let t = mpsl.get_temperature();
            write(Body::Health(Health {
                battery_mv: status::battery_mv().unwrap_or(0),
                temperature_cdeg: (t.raw() * 25) as i16,
                uptime_s: Instant::now().as_secs() as u32,
                queued: with(|r| r.outbox().len() as u16).unwrap_or(0),
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
