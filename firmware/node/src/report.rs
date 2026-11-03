//! Telemetry toward a sink (D-6), first cut. The initiator keeps a second link, to one sink, and
//! sends each on-device range, and its health once a minute, over an L2CAP channel as
//! `navgrid-proto` frames. The sink (a DK)
//! writes every frame it receives to its UART, which the DK's debugger exposes as VCOM1, for
//! `navgrid monitor`. The sink comes from the initiator's provisioning. It also takes updates from
//! the host on the same port and offers the ones it carries to the node on its link (`update`).

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::warn;
#[cfg(feature = "board-dk")]
use embassy_futures::join::join;
#[cfg(feature = "board-dk")]
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use navgrid_proto::{Firmware, Health, MAX_FRAME_LEN, Message, NodeId, Range, RangeMethod};
use navgrid_ranging::{PhaseSlope, range_quality, range_sigma_m};
use nrf_sdc::SoftdeviceController;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use trouble_host::prelude::*;

use crate::status;

type Stack<'a, 'd> = trouble_host::Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

/// Messages waiting for the uplink, or on the sink for its UART. Full means the sink can't keep
/// up; new ones are dropped.
static OUTBOX: Channel<CriticalSectionRawMutex, Message, 16> = Channel::new();
/// Ranges, the bulk of the traffic, stop queuing at this depth so a boot report, a log frame or
/// raw tones still find a slot.
const RANGE_QUEUE_LIMIT: usize = 12;

/// Queues one of this device's own reports (its boot report, say) toward the host.
pub fn queue(msg: Message) {
    let _ = OUTBOX.try_send(msg);
}

/// Sends the frames the logger keeps for the host (`logger`), as `Log` records.
#[embassy_executor::task]
pub async fn logs(node: NodeId) -> ! {
    loop {
        crate::logger::QUEUED_FRAME.wait().await;
        while let Some(frame) = crate::logger::take() {
            queue(Message::Log(navgrid_proto::Log {
                node,
                firmware: crate::FIRMWARE,
                board: crate::board::BOARD,
                frame,
            }));
        }
    }
}

/// On the sink: writes its own reports to the UART, as they're queued.
#[cfg(feature = "board-dk")]
pub async fn own_reports(uart: &SinkUart<'_>) -> ! {
    loop {
        let msg = OUTBOX.receive().await;
        send(uart, &msg).await;
    }
}

const HEALTH_PERIOD: Duration = Duration::from_secs(60);

/// The CS link the ranges belong to, once the uplink is up: (initiator, reflector).
static LINK: Mutex<CriticalSectionRawMutex, Cell<Option<(NodeId, NodeId)>>> = Mutex::new(Cell::new(None));
/// Whether an uplink is up to carry Health: a reflector's too, which has no CS link to report.
static UPLINK: AtomicBool = AtomicBool::new(false);

/// Starts reporting ranges of the CS link between `initiator` and `reflector`.
pub fn set_link(initiator: NodeId, reflector: NodeId) {
    LINK.lock(|l| l.set(Some((initiator, reflector))));
}

/// Queues one on-device range for the sink. A no-op until [`set_link`].
pub fn push(est: &PhaseSlope, paths: usize) {
    let Some((initiator, reflector)) = LINK.lock(Cell::get) else {
        return;
    };
    if OUTBOX.len() >= RANGE_QUEUE_LIMIT {
        return;
    }
    let range = Range {
        initiator,
        reflector,
        uptime_ms: Instant::now().as_millis() as u32,
        distance_mm: (est.distance_m.max(0.0) * 1000.0) as u32,
        sigma_mm: (range_sigma_m(est) * 1000.0) as u32,
        quality: range_quality(est),
        method: RangeMethod::PhaseSlope,
        steps: est.pairs,
        antenna_paths: paths as u8,
    };
    let _ = OUTBOX.try_send(Message::Range(range));
}

/// Added to every round-trip-time range before it's reported, so that the negative means the
/// reflector's turnaround bias gives survive the unsigned field; the host subtracts it.
pub const RTT_REPORT_OFFSET_M: f32 = 100.0;

/// Queues one on-device round-trip-time range (`cs::Ranging::rtt`), `steps` mode-1 steps averaged,
/// offset by [`RTT_REPORT_OFFSET_M`]. A no-op until [`set_link`].
pub fn push_rtt(distance_m: f32, steps: u8) {
    let Some((initiator, reflector)) = LINK.lock(Cell::get) else {
        return;
    };
    if OUTBOX.len() >= RANGE_QUEUE_LIMIT {
        return;
    }
    let range = Range {
        initiator,
        reflector,
        uptime_ms: Instant::now().as_millis() as u32,
        distance_mm: ((distance_m + RTT_REPORT_OFFSET_M).max(0.0) * 1000.0) as u32,
        sigma_mm: 1000,
        quality: 128,
        method: RangeMethod::Rtt,
        steps,
        antenna_paths: 1,
    };
    let _ = OUTBOX.try_send(Message::Range(range));
}

/// Queues one chunk of raw tones (`cs::send_tones`), as the initiator of the link. A no-op until
/// [`set_link`].
pub fn push_tones(procedure: u16, path: u8, paths: u8, first: u8, pct: heapless::Vec<u8, { navgrid_proto::TONES_PCT_LEN }>) {
    let Some((node, _)) = LINK.lock(Cell::get) else {
        return;
    };
    let _ = OUTBOX.try_send(Message::Tones(navgrid_proto::Tones {
        node,
        procedure,
        path,
        paths,
        first,
        pct,
    }));
}

/// Queues this node's health once a minute while the uplink is up (FR-S2).
pub async fn health(mpsl: &MultiprotocolServiceLayer<'_>, node: NodeId, firmware: Firmware) -> ! {
    loop {
        Timer::after(HEALTH_PERIOD).await;
        if !UPLINK.load(Ordering::Relaxed) {
            continue;
        }
        let t = mpsl.get_temperature();
        let health = Health {
            node,
            uptime_s: Instant::now().as_secs() as u32,
            battery_mv: status::battery_mv().unwrap_or(0),
            temperature_cdeg: (t.raw() * 25) as i16,
            firmware,
            queued: OUTBOX.len() as u16,
        };
        let _ = OUTBOX.try_send(Message::Health(health));
    }
}

/// Sends queued messages over `channel` until it fails.
pub async fn run(stack: &Stack<'_, '_>, channel: &mut L2capChannel<'_, DefaultPacketPool>) {
    UPLINK.store(true, Ordering::Relaxed);
    let mut buf = [0u8; MAX_FRAME_LEN];
    loop {
        let msg = OUTBOX.receive().await;
        let frame = match navgrid_proto::encode(&msg, &mut buf) {
            Ok(frame) => frame,
            Err(_) => {
                warn!("uplink: message does not fit a frame");
                continue;
            }
        };
        if let Err(e) = channel.send(stack, frame).await {
            warn!("uplink: {:?}", e);
            LINK.lock(|l| l.set(None));
            UPLINK.store(false, Ordering::Relaxed);
            return;
        }
        // TrouBLE's per-link send lock goes to whichever future asks first. With a backlog this
        // one would take it again at once, and the update channel on the same link (its credits,
        // its accept) would never send (DEF-41).
        embassy_futures::yield_now().await;
    }
}

/// The sink's UART: the frames it forwards and its own, among them its answers to the host.
#[cfg(feature = "board-dk")]
pub type SinkUart<'d> =
    embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::NoopRawMutex, embassy_nrf::uarte::UarteTx<'d>>;

/// Writes one of the sink's own frames.
#[cfg(feature = "board-dk")]
pub async fn send(uart: &SinkUart<'_>, msg: &Message) {
    let mut buf = [0u8; MAX_FRAME_LEN];
    if let Ok(frame) = navgrid_proto::encode(msg, &mut buf)
        && uart.lock().await.write(frame).await.is_err()
    {
        warn!("sink: UART write failed");
    }
}

/// How many nodes the sink serves at once.
#[cfg(feature = "board-dk")]
const SINK_LINKS: usize = 4;

/// The sink: writes a `Hello` frame, then advertises and serves up to [`SINK_LINKS`] fleet nodes at
/// once ([`serve_node`]), advertising again whenever a link is free.
#[cfg(feature = "board-dk")]
pub async fn serve_sink(
    stack: &Stack<'_, '_>,
    fleet: &crate::keys::Fleet,
    uart: &SinkUart<'_>,
    node: NodeId,
    beacons: &crate::beacon::Beacons,
    params: &AdvertisementParameters,
    rram: &'static crate::firmware::Rram,
) -> ! {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::signal::Signal;

    let hello = Message::Hello {
        node,
        role: navgrid_proto::Role::Sink,
        protocol: navgrid_proto::PROTOCOL_VERSION,
        firmware: crate::FIRMWARE,
        antennas: 1,
    };
    send(uart, &hello).await;
    let accepted: Channel<NoopRawMutex, Connection<'_, DefaultPacketPool>, 1> = Channel::new();
    let busy = Cell::new(0usize);
    let freed: Signal<NoopRawMutex, ()> = Signal::new();
    let advertising = async {
        let mut peripheral = stack.peripheral();
        loop {
            while busy.get() == SINK_LINKS {
                freed.wait().await;
            }
            defmt::info!(
                "sink: advertising, {=usize} of {=usize} links up",
                busy.get(),
                SINK_LINKS
            );
            let conn = beacons.advertise(&mut peripheral, params).await;
            busy.set(busy.get() + 1);
            accepted.send(conn).await;
        }
    };
    let slot = || async {
        loop {
            let conn = accepted.receive().await;
            serve_node(stack, fleet, uart, &conn, rram).await;
            busy.set(busy.get() - 1);
            freed.signal(());
        }
    };
    join(
        advertising,
        embassy_futures::join::join_array([slot(), slot(), slot(), slot()]),
    )
    .await;
    unreachable!()
}

/// One node's link to the sink, until it drops: forwards every SDU of its telemetry channel to
/// `uart` unchanged (each SDU is one frame). Once that channel is open, it offers the node the
/// update the sink carries, and again whenever it starts carrying another.
#[cfg(feature = "board-dk")]
async fn serve_node(
    stack: &Stack<'_, '_>,
    fleet: &crate::keys::Fleet,
    uart: &SinkUart<'_>,
    conn: &Connection<'_, DefaultPacketPool>,
    rram: &'static crate::firmware::Rram,
) {
    defmt::info!("sink: connected: handle {}", conn.handle().raw());
    fleet.trust(stack, conn);
    // Only fleet members get the channel.
    if !crate::encrypted(stack, conn, fleet).await {
        return;
    }
    let _link = status::Link::up(false);
    // A link can drop before or during the channel; never wait on a dead one.
    let disconnected = async {
        loop {
            if let ConnectionEvent::Disconnected { reason } = conn.next().await {
                return reason;
            }
        }
    };
    let forward = async {
        let mut channel = match L2capChannel::listen(stack, conn)
            .accept(&L2capChannelConfig::default())
            .await
        {
            Ok(channel) => channel,
            Err(e) => {
                warn!("sink: channel: {:?}", e);
                return;
            }
        };
        let frames = async {
            let mut sdu = [0u8; 255];
            let mut frames = 0u32;
            loop {
                match channel.receive(stack, &mut sdu).await {
                    Ok(len) => {
                        if uart.lock().await.write(&sdu[..len]).await.is_err() {
                            warn!("sink: UART write failed");
                        }
                        frames += 1;
                        if frames.is_multiple_of(100) {
                            defmt::info!("sink: handle {}: {=u32} frames", conn.handle().raw(), frames);
                        }
                    }
                    Err(e) => {
                        defmt::info!("sink: channel closed after {=u32} frames: {:?}", frames, e);
                        return;
                    }
                }
            }
        };
        // The node listens for updates once its telemetry channel is up.
        let offering = async {
            let Some(mut carrying) = crate::update::CARRYING.receiver() else {
                warn!("sink: no receiver left for carried updates");
                return core::future::pending().await;
            };
            // Whatever was carried before is offered just below.
            let _ = carrying.try_get();
            loop {
                // Its nodes are of another board: only what it carries for them is any use.
                if crate::firmware::carried().is_some() {
                    crate::update::offer(stack, conn, rram).await;
                }
                carrying.changed().await;
            }
        };
        select(frames, offering).await;
    };
    if let Either::Second(reason) = select(forward, disconnected).await {
        defmt::info!("sink: disconnected: {:?}", reason);
    }
}
