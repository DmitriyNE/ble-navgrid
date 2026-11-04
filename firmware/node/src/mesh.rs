//! Mesh links (architecture §3, stage 3): the `navgrid-link` policy driving this node's
//! connections.
//!
//! The node advertises its beacon while it has room for peripheral links and accepts the fleet
//! members the policy admits; it scans now and then (`cs::Events` passes the beacons it hears to
//! [`HEARD`]) and connects, as central, to the neighbours the policy wants, with its role's
//! connection parameters and, for a tag, the k-th link's event spacing (DEF-26). Every link
//! starts with a sync session over the sync channel (`state`), whose hellos tell both ends the
//! other's role and sink flag, and the central runs one every minute after; one session at a
//! time on a node. The central runs the bursts the policy schedules, switching an anchor link to
//! the fast parameters for the burst and back, and writes a Link record per burst. Telemetry
//! rides any link whose peer is a sink; a sink forwards what arrives to its UART. Updates ride
//! any link, offered by whoever carries one.

use core::cell::{Cell, RefCell};

use bt_hci::param::LeConnRole;
use defmt::{info, warn};
use embassy_futures::join::{join, join_array, join3, join4};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use navgrid_link::{Action, Beacon, MAX_LINKS, NodeId, Policy, Role, Settings};
use navgrid_mesh::Body;
use navgrid_mesh::wire::{self, Header, SYNC_SPSM, Sync};
use navgrid_proto::{TELEMETRY_SPSM, UPDATE_SPSM};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

use crate::beacon::Beacons;
use crate::firmware::Rram;
use crate::keys::Fleet;
use crate::{Host, cs, fix, report, state, status, update};

/// Beacons heard while scanning, from the event handler to the policy: who, what, how loud.
pub static HEARD: Channel<CriticalSectionRawMutex, (NodeId, Beacon, i8), 8> = Channel::new();

/// This node's ID, for the event handler's reports (no 64-bit atomics on this core).
pub struct OwnId(BlockingMutex<CriticalSectionRawMutex, Cell<u64>>);

impl OwnId {
    pub fn load(&self) -> NodeId {
        NodeId(self.0.lock(Cell::get))
    }

    fn store(&self, id: NodeId) {
        self.0.lock(|c| c.set(id.0));
    }
}

pub static SELF: OwnId = OwnId(BlockingMutex::new(Cell::new(0)));

/// How often the policy is asked what to do.
const TICK: Duration = Duration::from_millis(500);
/// How long a connection attempt may take; the policy gives up on it at the same time.
const CONNECT_WITHIN: Duration = Duration::from_secs(10);
/// Between sync sessions on a link, which the central starts: anchors every minute; a tag's
/// fixes are only useful fresh, so its links sync every few seconds.
const SESSION_EVERY: Duration = Duration::from_secs(60);
const TAG_SESSION_EVERY: Duration = Duration::from_secs(5);
/// Bytes of records and pushes a side sends per session (architecture §12).
const SESSION_BUDGET: u32 = 2048;
/// How long a side waits for the peer's next frame within a session: a slow link with
/// peripheral latency answers in seconds.
const SESSION_IDLE: Duration = Duration::from_secs(30);
/// How long the peripheral waits for the central's first hello on a new link.
const FIRST_HELLO_WITHIN: Duration = Duration::from_secs(45);
/// How long a side waits for the node's one session slot.
const SESSION_SLOT_WAIT: Duration = Duration::from_secs(20);
/// Sessions failing in a row before the link is dropped and made again.
const SESSION_FAILURES: u8 = 3;
/// How long a connection parameter update may take (six intervals at a 1 s slow interval).
const PARAMS_WITHIN: Duration = Duration::from_secs(15);
/// Anchors scan this long every so often; tags for 1 s every 5 s.
const ANCHOR_SCAN_EVERY: Duration = Duration::from_secs(60);
const ANCHOR_SCAN_FOR: Duration = Duration::from_secs(3);
const TAG_SCAN_EVERY: Duration = Duration::from_secs(5);
const TAG_SCAN_FOR: Duration = Duration::from_secs(1);
/// After a telemetry channel fails, before opening it again.
const UPLINK_RETRY: Duration = Duration::from_secs(5);

/// The node ID of a device address: the address in the low six bytes.
pub fn node_id(addr: &[u8]) -> NodeId {
    let mut id = [0u8; 8];
    let n = addr.len().min(6);
    id[..n].copy_from_slice(&addr[..n]);
    NodeId(u64::from_le_bytes(id))
}

/// A connection to serve, as it came up.
struct NewLink<'a> {
    conn: Connection<'a, DefaultPacketPool>,
    central: bool,
}

/// One of the node's link slots: what the link tasks and the policy's driver share.
struct Slot {
    peer: Cell<Option<NodeId>>,
    burst: Signal<NoopRawMutex, ()>,
    drop: Signal<NoopRawMutex, ()>,
    /// A connection parameter update ended, and whether it succeeded.
    params: Signal<NoopRawMutex, bool>,
}

impl Slot {
    const fn new() -> Self {
        Self {
            peer: Cell::new(None),
            burst: Signal::new(),
            drop: Signal::new(),
            params: Signal::new(),
        }
    }
}

#[cfg(feature = "board-dk")]
type Uart<'s> = Option<&'s report::SinkUart<'static>>;
#[cfg(not(feature = "board-dk"))]
type Uart<'s> = Option<&'s ()>;

/// `'s` is the borrow of the stack, which its central, peripheral and connections carry; `'a`
/// the stack's own lifetime; `'d` the controller's.
pub struct Mesh<'s, 'a, 'd> {
    stack: &'s Host<'a, 'd>,
    fleet: &'s Fleet,
    beacons: &'s Beacons,
    rram: &'static Rram,
    node: NodeId,
    role: Role,
    sink: bool,
    policy: BlockingMutex<NoopRawMutex, RefCell<Policy>>,
    central: Mutex<NoopRawMutex, Central<'s, SoftdeviceController<'d>, DefaultPacketPool>>,
    new_links: Channel<NoopRawMutex, NewLink<'s>, 1>,
    slots: [Slot; MAX_LINKS],
    /// A link went down: the advertiser may have room again.
    freed: Signal<NoopRawMutex, ()>,
    /// The sink's UART, for the telemetry it forwards.
    #[cfg_attr(not(feature = "board-dk"), allow(dead_code))]
    uart: Uart<'s>,
}

impl<'s, 'a, 'd> Mesh<'s, 'a, 'd> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stack: &'s Host<'a, 'd>,
        fleet: &'s Fleet,
        beacons: &'s Beacons,
        rram: &'static Rram,
        node: NodeId,
        role: Role,
        sink: bool,
        uart: Uart<'s>,
    ) -> Self {
        SELF.store(node);
        let mut policy = Policy::with_sink(node, role, sink, Settings::default());
        // Until the accelerometer drives it (FR-T6), a tag is taken to be moving: it keeps links
        // and ranges continuously.
        policy.set_moving(true, 0);
        Self {
            stack,
            fleet,
            beacons,
            rram,
            node,
            role,
            sink,
            policy: BlockingMutex::new(RefCell::new(policy)),
            central: Mutex::new(stack.central()),
            new_links: Channel::new(),
            slots: [const { Slot::new() }; MAX_LINKS],
            freed: Signal::new(),
            uart,
        }
    }

    fn with_policy<R>(&self, f: impl FnOnce(&mut Policy) -> R) -> R {
        self.policy.lock(|p| f(&mut p.borrow_mut()))
    }

    fn now_ms() -> u64 {
        Instant::now().as_millis()
    }

    /// The beacon's free-link count follows the policy, and the node record's neighbours.
    fn links_changed(&self) {
        let (free, neighbours) = self.with_policy(|p| {
            let mut n: heapless::Vec<NodeId, MAX_LINKS> = heapless::Vec::new();
            for l in p.links() {
                let _ = n.push(l.peer);
            }
            (p.free_links(), n)
        });
        status::set_free_links(free);
        state::set_neighbours(neighbours.into_iter());
        status::BEACON_CHANGED.signal(());
    }

    /// Runs the mesh: never returns.
    pub async fn run(&self) {
        self.links_changed();
        join4(self.drive(), self.scan(), self.advertise(), self.serve_links()).await;
    }

    /// Asks the policy what to do, twice a second, and does it: connects as central, starts
    /// bursts, drops links.
    async fn drive(&self) {
        loop {
            Timer::after(TICK).await;
            if status::retired() {
                continue;
            }
            while let Ok((id, beacon, rssi)) = HEARD.try_receive() {
                self.with_policy(|p| p.heard(id, beacon, rssi, Self::now_ms()));
            }
            let now = Self::now_ms();
            self.with_policy(|p| p.tick(now));
            for _ in 0..4 {
                match self.with_policy(|p| p.next(now)) {
                    Action::Nothing => break,
                    Action::Connect(peer) => self.connect(peer).await,
                    Action::Burst(peer) => match self.slot_of(peer) {
                        Some(slot) => slot.burst.signal(()),
                        None => self.with_policy(|p| p.burst_done(peer, Self::now_ms())),
                    },
                    Action::Disconnect(peer) => {
                        if let Some(slot) = self.slot_of(peer) {
                            slot.drop.signal(());
                        }
                    }
                }
            }
        }
    }

    fn slot_of(&self, peer: NodeId) -> Option<&Slot> {
        self.slots.iter().find(|s| s.peer.get() == Some(peer))
    }

    /// Connects to `peer` as central, with the role's parameters and spacing, and hands the link
    /// to a slot. The policy hears about a failure; a success reaches it through the hello.
    async fn connect(&self, peer: NodeId) {
        let (peer_sink, interval_1250, latency, spacing_1250) = self.with_policy(|p| {
            let heard = p.heard_list().iter().find(|h| h.id == peer).map(|h| h.beacon.sink);
            let (interval, latency) = p.connect_params(heard.unwrap_or(false));
            (heard.unwrap_or(false), interval, latency, p.event_spacing_1250us())
        });
        let interval = Duration::from_micros(u64::from(interval_1250) * 1250);
        // Six listened events, and at most the 32 s the spec allows.
        let timeout_ms = (u64::from(interval_1250) * 1250 / 1000 * u64::from(latency + 1) * 6).clamp(4_000, 32_000);
        let config = ConnectConfig {
            connect_params: RequestedConnParams {
                min_connection_interval: interval,
                max_connection_interval: interval,
                max_latency: latency,
                supervision_timeout: Duration::from_millis(timeout_ms),
                ..Default::default()
            },
            scan_config: ScanConfig {
                filter_accept_list: &[Address::random(peer.address())],
                ..Default::default()
            },
        };
        info!(
            "connecting to {=u64:012x}: interval {=u64} ms, latency {=u16}, spacing {=u64} ms",
            peer.0,
            interval.as_millis(),
            latency,
            u64::from(spacing_1250) * 1250 / 1000
        );
        // TrouBLE forgets the key when a link drops.
        self.fleet.trust_address(self.stack, peer.address());
        let connected = {
            let mut central = self.central.lock().await;
            if spacing_1250 > 0
                && let Err(e) = self
                    .stack
                    .command(nrf_sdc::vendor::NordicCentralAclEventSpacingSet::new(
                        u32::from(spacing_1250) * 1250,
                    ))
                    .await
            {
                warn!("central event spacing: {:?}", e);
            }
            with_timeout(CONNECT_WITHIN, central.connect_ext(&config)).await
        };
        match connected {
            Ok(Ok(conn)) => {
                let _ = peer_sink;
                self.new_links.send(NewLink { conn, central: true }).await;
            }
            Ok(Err(e)) => {
                warn!("connect to {=u64:012x}: {:?}", peer.0, e);
                self.with_policy(|p| p.connect_failed(peer, Self::now_ms()));
            }
            Err(_) => {
                warn!(
                    "connect to {=u64:012x}: not within {} s",
                    peer.0,
                    CONNECT_WITHIN.as_secs()
                );
                self.with_policy(|p| p.connect_failed(peer, Self::now_ms()));
            }
        }
    }

    /// Scans for beacons now and then; the event handler feeds [`HEARD`].
    async fn scan(&self) {
        let (every, window) = match self.role {
            Role::Anchor => (ANCHOR_SCAN_EVERY, ANCHOR_SCAN_FOR),
            Role::Asset | Role::Mobile => (TAG_SCAN_EVERY, TAG_SCAN_FOR),
        };
        loop {
            {
                let mut central = self.central.lock().await;
                let mut scanner = Scanner::new(&mut central);
                match scanner.scan_ext(&ScanConfig::default()).await {
                    Ok(session) => {
                        Timer::after(window).await;
                        session.stop().await;
                    }
                    Err(e) => warn!("scan: {:?}", e),
                }
            }
            Timer::after(every).await;
        }
    }

    /// Advertises the beacon while there is room for another peripheral link, and hands each
    /// accepted connection to a slot.
    async fn advertise(&self) {
        let mut peripheral = self.stack.peripheral();
        loop {
            if self.with_policy(|p| p.free_links()) == 0 || status::retired() {
                self.freed.wait().await;
                continue;
            }
            let conn = self.beacons.advertise(&mut peripheral).await;
            self.new_links.send(NewLink { conn, central: false }).await;
        }
    }

    /// The link slots: each serves one link at a time, from the channel of new ones.
    async fn serve_links(&self) {
        join_array(core::array::from_fn::<_, MAX_LINKS, _>(|i| self.slot_loop(i))).await;
    }

    async fn slot_loop(&self, i: usize) {
        loop {
            let link = self.new_links.receive().await;
            self.serve_link(&self.slots[i], link).await;
        }
    }

    /// One link, from its connection to its end: features, encryption, the hello each way, then
    /// its channels, telemetry, bursts and updates until it drops.
    async fn serve_link(&self, slot: &Slot, link: NewLink<'s>) {
        let NewLink { conn, central } = link;
        let peer = node_id(conn.peer_address().addr.raw());
        let handle = conn.handle();
        info!(
            "link to {=u64:012x}: handle {}, {}",
            peer.0,
            handle.raw(),
            if central { "central" } else { "peripheral" }
        );
        if !central {
            self.fleet.trust(self.stack, &conn);
        }
        // Listening from the start: TrouBLE refuses a channel the peer opens before the link is
        // marked listening, and the peer's hello or update offer can come early. The mark stays
        // for the link's life; `channels` and the hello take the requests as they come.
        drop(L2capChannel::listen(self.stack, &conn));
        cs::exchange_features(self.stack, handle).await;
        if central {
            // Zephyr's central moves the ACL to 2M on connect (BT_AUTO_PHY_UPDATE); CS runs on 2M
            // too. And to the largest data length (BT_AUTO_DATA_LEN_UPDATE): 251 octets, 2120 us.
            if let Err(e) = conn.set_phy(self.stack, PhyKind::Le2M).await {
                warn!("set PHY: {:?}", e);
            }
            if let Err(e) = conn.update_data_length(self.stack, 251, 2120).await {
                warn!("data length: {:?}", e);
            }
            if !crate::secure(&conn).await || !crate::encrypted(self.stack, &conn, self.fleet).await {
                self.with_policy(|p| p.connect_failed(peer, Self::now_ms()));
                return;
            }
        } else {
            if let Err(e) = cs::reflect(self.stack, handle).await {
                warn!("CS reflector settings: {}", e);
            }
            if !crate::encrypted(self.stack, &conn, self.fleet).await {
                return;
            }
        }
        // The first session: its hellos say who the peer is.
        let (mut sync, theirs) = match if central {
            self.first_session_as_central(&conn, peer).await
        } else {
            self.first_session_as_peripheral(&conn, peer).await
        } {
            Some(ok) => ok,
            None => {
                warn!("link to {=u64:012x}: no first session", peer.0);
                if central {
                    self.with_policy(|p| p.connect_failed(peer, Self::now_ms()));
                }
                conn.disconnect();
                drain(&conn).await;
                return;
            }
        };
        if !central && !self.with_policy(|p| p.accepts(peer, theirs.role)) {
            warn!("link from {=u64:012x} ({}): no room", peer.0, theirs.role);
            conn.disconnect();
            drain(&conn).await;
            return;
        }
        info!(
            "link to {=u64:012x}: {}{}, {} hops from a sink",
            peer.0,
            theirs.role,
            if theirs.sink { " (sink)" } else { "" },
            theirs.hops
        );
        self.with_policy(|p| p.link_up(peer, central, theirs.role, theirs.sink, Self::now_ms()));
        slot.peer.set(Some(peer));
        let _link = status::Link::up(theirs.sink);
        self.links_changed();

        let events = self.events(&conn, slot);
        let channels = self.channels(&conn);
        let telemetry = async {
            if theirs.sink && !self.sink {
                self.uplink(&conn).await
            } else {
                core::future::pending().await
            }
        };
        let bursts = async {
            loop {
                slot.burst.wait().await;
                if central {
                    self.burst(&conn, slot, peer).await;
                } else {
                    self.with_policy(|p| p.burst_done(peer, Self::now_ms()));
                }
            }
        };
        let offers = self.offers(&conn);
        let dropping = async {
            slot.drop.wait().await;
            info!("link to {=u64:012x}: dropping it", peer.0);
            conn.disconnect();
            core::future::pending::<()>().await
        };
        let sessions = self.sessions(&mut sync, central, peer);
        if let Either::Second(Either::First(())) = select(
            events,
            select(
                sessions,
                join3(join(channels, telemetry), bursts, join(offers, dropping)),
            ),
        )
        .await
        {
            // The sessions gave up: make the link anew.
            conn.disconnect();
            drain(&conn).await;
        }

        slot.peer.set(None);
        cs::forget(handle);
        self.with_policy(|p| p.link_down(peer, Self::now_ms()));
        self.links_changed();
        self.freed.signal(());
    }

    /// The central's first session: opens the sync channel and runs a session on it.
    async fn first_session_as_central(
        &self,
        conn: &Connection<'s, DefaultPacketPool>,
        peer: NodeId,
    ) -> Option<(L2capChannel<'s, DefaultPacketPool>, Header)> {
        let mut channel = match L2capChannel::create(self.stack, conn, SYNC_SPSM, &Default::default()).await {
            Ok(channel) => channel,
            Err(e) => {
                warn!("sync channel: {:?}", e);
                return None;
            }
        };
        match self.session(&mut channel, None, peer).await {
            Ok(theirs) => Some((channel, theirs)),
            Err(e) => {
                warn!("first session with {=u64:012x}: {}", peer.0, e);
                None
            }
        }
    }

    /// The peripheral's first session: takes the sync channel the central opens and its hello,
    /// and runs the session from there.
    async fn first_session_as_peripheral(
        &self,
        conn: &Connection<'s, DefaultPacketPool>,
        peer: NodeId,
    ) -> Option<(L2capChannel<'s, DefaultPacketPool>, Header)> {
        let accepted = with_timeout(FIRST_HELLO_WITHIN, async {
            loop {
                let pending = match L2capChannel::listen(self.stack, conn).next().await {
                    Ok(pending) => pending,
                    Err(e) => {
                        warn!("sync channel: {:?}", e);
                        return None;
                    }
                };
                if pending.spsm() != SYNC_SPSM {
                    warn!("channel {=u16:#06x} before the first session", pending.spsm());
                    continue;
                }
                return pending.accept(self.stack, &Default::default()).await.ok();
            }
        })
        .await;
        let mut channel = match accepted {
            Ok(Some(channel)) => channel,
            _ => return None,
        };
        let first = match with_timeout(FIRST_HELLO_WITHIN, receive_frame(self.stack, &mut channel)).await {
            Ok(Ok(msg)) => msg,
            Ok(Err(e)) => {
                warn!("first hello from {=u64:012x}: {}", peer.0, e);
                return None;
            }
            Err(_) => {
                warn!(
                    "first hello from {=u64:012x}: none within {} s",
                    peer.0,
                    FIRST_HELLO_WITHIN.as_secs()
                );
                return None;
            }
        };
        match self.session(&mut channel, Some(first), peer).await {
            Ok(theirs) => Some((channel, theirs)),
            Err(e) => {
                warn!("first session with {=u64:012x}: {}", peer.0, e);
                None
            }
        }
    }

    /// The sessions after the first: the central starts one every [`SESSION_EVERY`]; the
    /// peripheral answers each hello it receives. Returns when sessions fail
    /// [`SESSION_FAILURES`] times in a row, or the channel dies.
    async fn sessions(&self, channel: &mut L2capChannel<'s, DefaultPacketPool>, central: bool, peer: NodeId) {
        let mut failures = 0u8;
        loop {
            let first = if central {
                Timer::after(if self.role == Role::Anchor {
                    SESSION_EVERY
                } else {
                    TAG_SESSION_EVERY
                })
                .await;
                None
            } else {
                match receive_frame(self.stack, channel).await {
                    Ok(msg @ Sync::Hello(_)) => Some(msg),
                    Ok(_) => continue,
                    Err(e) => {
                        warn!("sync channel to {=u64:012x}: {}", peer.0, e);
                        return;
                    }
                }
            };
            match self.session(channel, first, peer).await {
                Ok(_) => failures = 0,
                Err(e) => {
                    warn!("session with {=u64:012x}: {}", peer.0, e);
                    failures += 1;
                    if failures >= SESSION_FAILURES {
                        return;
                    }
                }
            }
        }
    }

    /// One session over `channel`, with `first` a frame already received from the peer. Waits
    /// for the node's session slot, then exchanges frames until the driver is done.
    async fn session(
        &self,
        channel: &mut L2capChannel<'s, DefaultPacketPool>,
        first: Option<Sync>,
        peer: NodeId,
    ) -> Result<Header, SessionError> {
        let started = Instant::now();
        let mine = state::header(self.role, self.sink, peer);
        while !state::begin_session(mine, SESSION_BUDGET, peer) {
            if started.elapsed() > SESSION_SLOT_WAIT {
                return Err(SessionError::Busy);
            }
            Timer::after(Duration::from_millis(200)).await;
        }
        let result = self.exchange(channel, first).await;
        status::count_session();
        let side = state::session_side();
        let theirs = state::end_session(peer);
        state::log_session(peer, side, started.elapsed().as_millis());
        result.and_then(|()| theirs.ok_or(SessionError::NoHello))
    }

    async fn exchange(
        &self,
        channel: &mut L2capChannel<'s, DefaultPacketPool>,
        first: Option<Sync>,
    ) -> Result<(), SessionError> {
        let mut out = [0u8; wire::MAX_FRAME_LEN];
        if let Some(msg) = first {
            state::session_in(msg).map_err(SessionError::Protocol)?;
        }
        loop {
            while let Some(msg) = state::session_out() {
                let frame = wire::encode(&msg, &mut out).map_err(|_| SessionError::Encode)?;
                channel
                    .send(self.stack, frame)
                    .await
                    .map_err(|_| SessionError::Channel)?;
            }
            if state::session_complete() {
                return Ok(());
            }
            let msg = with_timeout(SESSION_IDLE, receive_frame(self.stack, channel))
                .await
                .map_err(|_| SessionError::Timeout)??;
            state::session_in(msg).map_err(SessionError::Protocol)?;
        }
    }

    /// Handles the link's connection events until it drops. Drops a peer outside the fleet.
    async fn events(&self, conn: &Connection<'s, DefaultPacketPool>, slot: &Slot) {
        loop {
            let event = conn.next().await;
            if crate::outsider(&event, conn, self.fleet) {
                warn!("peer outside the fleet: dropping it");
                status::foreign();
                conn.disconnect();
                continue;
            }
            match event {
                ConnectionEvent::Disconnected { reason } => {
                    crate::disconnected(reason);
                    return;
                }
                ConnectionEvent::Encrypted { security_level, .. } => crate::log_encrypted(security_level, self.fleet),
                ConnectionEvent::RequestConnectionParams(req) => {
                    if let Err(e) = req.accept(None, self.stack).await {
                        warn!("connection parameters: {:?}", e);
                    }
                }
                ConnectionEvent::ConnectionParamsUpdated {
                    conn_interval,
                    peripheral_latency,
                    ..
                } => {
                    info!(
                        "connection parameters: interval {=u64} ms, latency {=u16}",
                        conn_interval.as_millis(),
                        peripheral_latency
                    );
                    slot.params.signal(true);
                }
                ConnectionEvent::ConnectionParamsUpdateFailed { .. } => slot.params.signal(false),
                ConnectionEvent::DataLengthUpdated {
                    max_tx_octets,
                    max_tx_time,
                    max_rx_octets,
                    max_rx_time,
                } => {
                    info!(
                        "data length: tx {} B {} us, rx {} B {} us",
                        max_tx_octets, max_tx_time, max_rx_octets, max_rx_time
                    )
                }
                _ => {}
            }
        }
    }

    /// Takes the channels the peer opens on the link and gives each to its owner: updates to
    /// `update`, telemetry to the sink's UART, another hello to a reply.
    async fn channels(&self, conn: &Connection<'s, DefaultPacketPool>) {
        loop {
            let pending = match L2capChannel::listen(self.stack, conn).next().await {
                Ok(pending) => pending,
                Err(_) => return core::future::pending().await,
            };
            let spsm = pending.spsm();
            let config = if spsm == UPDATE_SPSM {
                update::RECEIVING
            } else {
                L2capChannelConfig::default()
            };
            let channel = match pending.accept(self.stack, &config).await {
                Ok(channel) => channel,
                Err(e) => {
                    warn!("channel {=u16:#06x}: {:?}", spsm, e);
                    continue;
                }
            };
            match spsm {
                // Mesh links run full-length packets throughout (2M, 251 octets).
                UPDATE_SPSM => update::take(self.stack, conn, channel, self.rram, false).await,
                #[cfg(feature = "board-dk")]
                TELEMETRY_SPSM => match self.uart {
                    Some(uart) => report::forward(self.stack, uart, channel, conn.handle().raw()).await,
                    None => info!("telemetry channel on a node that isn't a sink: closing it"),
                },
                #[cfg(not(feature = "board-dk"))]
                TELEMETRY_SPSM => info!("telemetry channel on a node that isn't a sink: closing it"),
                SYNC_SPSM => info!("a second sync channel from the peer: closing it"),
                _ => info!("channel {=u16:#06x} opened by the peer: closing it", spsm),
            }
        }
    }

    /// Carries the outbox over the link while its peer is a sink, opening the telemetry channel
    /// again after a failure.
    async fn uplink(&self, conn: &Connection<'s, DefaultPacketPool>) {
        loop {
            match L2capChannel::create(self.stack, conn, TELEMETRY_SPSM, &Default::default()).await {
                Ok(mut channel) => {
                    info!("uplink: channel open");
                    report::run(self.stack, &mut channel).await;
                }
                Err(e) => warn!("uplink: channel: {:?}", e),
            }
            Timer::after(UPLINK_RETRY).await;
        }
    }

    /// Offers the update this node carries, if any, once the link is up and every few minutes
    /// after (DEF-39), and on a sink as soon as it starts carrying another (taken from the host).
    #[cfg(feature = "board-dk")]
    async fn offers(&self, conn: &Connection<'s, DefaultPacketPool>) {
        let Some(mut carrying) = update::CARRYING.receiver() else {
            warn!("no receiver left for carried updates");
            return core::future::pending().await;
        };
        let _ = carrying.try_get();
        loop {
            update::offer(self.stack, conn, self.rram).await;
            select(carrying.changed(), Timer::after(update::OFFER_EVERY)).await;
        }
    }

    /// Offers the update this node carries, if any, once the link is up and every few minutes
    /// after (DEF-39).
    #[cfg(not(feature = "board-dk"))]
    async fn offers(&self, conn: &Connection<'s, DefaultPacketPool>) {
        loop {
            update::offer(self.stack, conn, self.rram).await;
            Timer::after(update::OFFER_EVERY).await;
        }
    }

    /// One burst on a link this node is central on: to the fast parameters if the link idles
    /// slow, the procedures, the Link record, back to the slow parameters.
    async fn burst(&self, conn: &Connection<'s, DefaultPacketPool>, slot: &Slot, peer: NodeId) {
        let (switches, procedures, slow, fast) = self.with_policy(|p| {
            let s = *p.settings();
            (
                p.switches_for_burst(peer),
                p.burst_length(peer),
                p.connect_params(self.with_sink_peer(peer)),
                (s.fast_interval_1250us, 0u16),
            )
        });
        if switches && !self.set_params(conn, slot, fast).await {
            self.with_policy(|p| p.burst_done(peer, Self::now_ms()));
            return;
        }
        let started = Instant::now();
        match cs::burst(
            self.stack,
            conn.handle(),
            self.node,
            peer,
            status::antennas(),
            procedures,
        )
        .await
        {
            Ok(burst) => {
                let (ok, failed) = (burst.estimates(), burst.failures);
                status::count_burst(burst.procedures());
                match burst.finish() {
                    Some(link) => {
                        info!(
                            "burst with {=u64:012x}: {=u32} mm ± {=u32}, quality {=u8}, {=usize} of {=u16} procedures in {=u64} ms",
                            peer.0,
                            link.distance_mm,
                            link.sigma_mm,
                            link.quality,
                            ok,
                            link.procedures,
                            started.elapsed().as_millis()
                        );
                        state::write(Body::Link(link));
                        if self.role != Role::Anchor {
                            let beacon = self.with_policy(|p| {
                                p.heard_list()
                                    .iter()
                                    .find(|h| h.id == peer)
                                    .and_then(|h| h.beacon.position)
                            });
                            fix::ranged(
                                self.node,
                                peer,
                                link.distance_mm as f32 / 1000.0,
                                link.sigma_mm as f32 / 1000.0,
                                beacon,
                            );
                        }
                    }
                    None => warn!("burst with {=u64:012x}: nothing from {=u16} procedures", peer.0, failed),
                }
            }
            Err(e) => warn!("burst with {=u64:012x}: {}", peer.0, e),
        }
        if switches {
            // An anchor's links outnumber the controller's CS configurations: free the slot.
            cs::remove_config(self.stack, conn.handle()).await;
            self.set_params(conn, slot, slow).await;
        }
        self.with_policy(|p| p.burst_done(peer, Self::now_ms()));
    }

    fn with_sink_peer(&self, peer: NodeId) -> bool {
        self.with_policy(|p| p.link(peer).is_some_and(|l| l.peer_sink))
    }

    /// Asks the controller for new connection parameters and waits for the update.
    async fn set_params(
        &self,
        conn: &Connection<'s, DefaultPacketPool>,
        slot: &Slot,
        (interval_1250, latency): (u16, u16),
    ) -> bool {
        let interval = Duration::from_micros(u64::from(interval_1250) * 1250);
        let timeout_ms = (u64::from(interval_1250) * 1250 / 1000 * u64::from(latency + 1) * 6).clamp(4_000, 32_000);
        let params = RequestedConnParams {
            min_connection_interval: interval,
            max_connection_interval: interval,
            max_latency: latency,
            supervision_timeout: Duration::from_millis(timeout_ms),
            ..Default::default()
        };
        slot.params.reset();
        if let Err(e) = conn.update_connection_params(self.stack, &params).await {
            warn!("connection parameters: {:?}", e);
            return false;
        }
        match with_timeout(PARAMS_WITHIN, slot.params.wait()).await {
            Ok(ok) => ok,
            Err(_) => {
                warn!("connection parameters: no update within {} s", PARAMS_WITHIN.as_secs());
                false
            }
        }
    }
}

/// Why a session ended early.
#[derive(Clone, Copy, Debug, defmt::Format)]
enum SessionError {
    /// The node's session slot stayed taken.
    Busy,
    /// The peer sent nothing for [`SESSION_IDLE`].
    Timeout,
    /// The channel failed.
    Channel,
    Encode,
    Decode,
    /// The peer broke the protocol.
    Protocol(navgrid_mesh::Protocol),
    /// The session ended without the peer's hello.
    NoHello,
}

/// One frame from the sync channel, decoded.
async fn receive_frame(
    stack: &Host<'_, '_>,
    channel: &mut L2capChannel<'_, DefaultPacketPool>,
) -> Result<Sync, SessionError> {
    let mut buf = [0u8; 255];
    let len = channel
        .receive(stack, &mut buf)
        .await
        .map_err(|_| SessionError::Channel)?;
    // The frame's terminator may or may not be in the SDU.
    let end = buf[..len].iter().position(|&b| b == 0).unwrap_or(len);
    wire::decode(&mut buf[..end]).map_err(|_| SessionError::Decode)
}

/// Waits for a link that was told to disconnect to actually go.
async fn drain(conn: &Connection<'_, DefaultPacketPool>) {
    let _ = with_timeout(Duration::from_secs(5), async {
        loop {
            if let ConnectionEvent::Disconnected { .. } = conn.next().await {
                return;
            }
        }
    })
    .await;
}

#[allow(dead_code)]
fn _role_is_peripheral(conn: &Connection<'_, DefaultPacketPool>) -> bool {
    conn.role() == LeConnRole::Peripheral
}
