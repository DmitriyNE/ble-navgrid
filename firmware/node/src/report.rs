//! Telemetry toward a sink (D-6). A node queues its ranges, raw tones, health, boot and log
//! reports in an outbox and sends them as `navgrid-proto` frames over an L2CAP channel on any
//! mesh link whose peer is a sink (`mesh`). The sink (a DK) writes every frame it receives to its
//! UART, which the DK's debugger exposes as VCOM1, for `navgrid monitor`, and takes updates from
//! the host on the same port (`update`).

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::warn;
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

/// Whether a link to a sink carries the outbox, so Health is worth queuing.
static UPLINK: AtomicBool = AtomicBool::new(false);

/// Queues one on-device range of the link between `initiator` (this node) and `reflector`.
pub fn push(initiator: NodeId, reflector: NodeId, est: &PhaseSlope, paths: usize) {
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
/// offset by [`RTT_REPORT_OFFSET_M`].
pub fn push_rtt(initiator: NodeId, reflector: NodeId, distance_m: f32, steps: u8) {
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

/// Beacons heard, reported to the host once per node per [`HEARD_EVERY`]: a sink's nearby list
/// for the field tool (FR-O6). Other nodes keep what they hear to themselves.
const HEARD_EVERY: Duration = Duration::from_secs(10);
const HEARD_NODES: usize = 24;
static HEARD_LAST: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<heapless::Vec<(NodeId, Instant), HEARD_NODES>>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(heapless::Vec::new()));

/// A beacon `node` heard from `of`.
pub fn push_heard(node: NodeId, of: NodeId, rssi_dbm: i8, beacon: &navgrid_proto::Beacon) {
    let now = Instant::now();
    let due = HEARD_LAST.lock(|l| {
        let mut l = l.borrow_mut();
        match l.iter_mut().find(|(id, _)| *id == of) {
            Some((_, last)) if now - *last < HEARD_EVERY => false,
            Some((_, last)) => {
                *last = now;
                true
            }
            None => {
                if l.is_full() {
                    let (i, _) = l.iter().enumerate().min_by_key(|(_, (_, t))| *t).expect("not empty");
                    l.swap_remove(i);
                }
                let _ = l.push((of, now));
                true
            }
        }
    });
    if !due || OUTBOX.len() >= RANGE_QUEUE_LIMIT {
        return;
    }
    let _ = OUTBOX.try_send(Message::Heard(navgrid_proto::Heard {
        node,
        of,
        rssi_dbm,
        role: beacon.role,
        sink: beacon.sink,
        attention: beacon.attention,
        install: beacon.install,
        hops: beacon.hops,
        free_links: beacon.free_links,
        battery_mv: beacon.battery_mv,
        number: beacon.number,
    }));
}

/// Queues one chunk of raw tones (`cs::send_tones`), as the initiator of the link.
pub fn push_tones(
    node: NodeId,
    procedure: u16,
    path: u8,
    paths: u8,
    first: u8,
    pct: heapless::Vec<u8, { navgrid_proto::TONES_PCT_LEN }>,
) {
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

/// On the sink: forwards every SDU of a node's telemetry channel to `uart` unchanged (each SDU is
/// one frame) until the channel closes.
#[cfg(feature = "board-dk")]
pub async fn forward(stack: &Stack<'_, '_>, uart: &SinkUart<'_>, mut channel: L2capChannel<'_, DefaultPacketPool>, handle: u16) {
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
                    defmt::info!("sink: handle {}: {=u32} frames", handle, frames);
                }
            }
            Err(e) => {
                defmt::info!("sink: channel closed after {=u32} frames: {:?}", frames, e);
                return;
            }
        }
    }
}
