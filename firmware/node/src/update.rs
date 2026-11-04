//! Firmware updates between neighbours (architecture §13, D-36), single hop.
//!
//! - **Sending:** a node offers what it can pass on to a peer over a fleet link ([`offer`]): the
//!   update it was installed from, or on a sink the one it carries for the nodes' board.
//! - **Receiving:** a peer of the same board on another build takes it ([`listen`]). It writes
//!   the image into its update slot and the header into its staging page, then restarts into the
//!   check at boot (`firmware::apply_staged`), which hands a good one to the bootloader.
//! - **From the host:** a sink takes an update over its serial port ([`from_host`]). One for its
//!   own board it installs the same way. One for another board it checks against the signing key
//!   and keeps in its update slot, then offers it to the nodes on its links.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, with_timeout};
use navgrid_proto::layout::{ACTIVE, ACTIVE_LEN, DFU, STAGED_UPDATE, STAGED_UPDATE_LEN};
use navgrid_proto::{Firmware, UPDATE_CHUNK, UPDATE_HEADER_LEN, UPDATE_SPSM, UpdateHeader, UpdateMsg};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

use crate::board::BOARD;
use crate::firmware::{self, Rram};
#[cfg(feature = "board-dk")]
pub use sink::{CARRYING, from_host};

type Stack<'a, 'd> = trouble_host::Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

/// The receiver writes the update slot this much at a time: RRAM needs no erase, so this is
/// only the write granularity (and a resume's), and the buffer's size in RAM.
const PAGE: usize = 1024;
/// How long a peer may take to answer an offer, and to store the end of the image.
const ANSWER_WITHIN: Duration = Duration::from_secs(5);
/// Progress is logged every this many bytes.
const LOG_EVERY: usize = 64 * 1024;
/// A write gets its timeslots within this, or fails: one waiting longer would hold the transfer
/// up without a word.
const WRITE_WITHIN: Duration = Duration::from_secs(10);
/// Writes slower than this are logged.
const SLOW_WRITE_MS: u64 = 500;
/// A chunk that takes longer than this to send or arrive is logged; the transfer goes on waiting.
const STALL_AFTER: Duration = Duration::from_secs(5);
/// A chunk that still hasn't come or gone this long after that ends the transfer: a link that
/// died under a transfer left the receive pending for good on the pre-mesh build, and the node
/// then declined every offer as busy (log 2026-11-04).
const GIVE_UP_AFTER: Duration = Duration::from_secs(60);
/// A node offers what it carries on each link this often, not only once the link is up: a peer
/// that declined while its own image wasn't confirmed yet, or whose transfer broke off, gets
/// another offer without a reconnect (DEF-39).
pub const OFFER_EVERY: Duration = Duration::from_secs(3 * 60);

/// A transfer that broke off: its build, and how many whole pages of it the update slot holds.
/// The next offer of the same build is taken from there (DEF-39). Kept in RAM: a restart starts
/// over.
static PARTIAL: Mutex<CriticalSectionRawMutex, Cell<Option<(UpdateHeader, usize)>>> = Mutex::new(Cell::new(None));

/// The build the controller wants this node on, once a desired-firmware record has named one.
/// Offers of other builds are then declined, so a rollout reaches the nodes the controller named
/// and no others.
static WANTED: Mutex<CriticalSectionRawMutex, Cell<Option<Firmware>>> = Mutex::new(Cell::new(None));

/// The controller wants `firmware` here. Whether that's news.
pub fn want(firmware: Firmware) -> bool {
    WANTED.lock(|w| w.replace(Some(firmware)) != Some(firmware))
}

/// The build the controller wants here, if it has said.
pub fn wanted() -> Option<Firmware> {
    WANTED.lock(|w| w.get())
}

/// Offers what this node can pass on, if anything, to the peer on `conn`, and streams it if the
/// peer wants it. An image the node runs goes out only once it's confirmed here: a bad one would
/// otherwise spread before the bootloader reverts it.
/// The connection parameters a transfer asks for, and the link's own to go back to if the
/// transfer fails (after one that completes, the peer restarts into the image).
#[derive(Clone, Copy)]
pub struct TransferParams {
    pub fast: (u16, u16),
    pub back: (u16, u16),
}

async fn request_params(
    stack: &Stack<'_, '_>,
    conn: &Connection<'_, DefaultPacketPool>,
    (interval_1250, latency): (u16, u16),
) {
    let interval = Duration::from_micros(u64::from(interval_1250) * 1250);
    let timeout_ms = (u64::from(interval_1250) * 1250 / 1000 * u64::from(latency + 1) * 6).clamp(4_000, 32_000);
    let params = RequestedConnParams {
        min_connection_interval: interval,
        max_connection_interval: interval,
        max_latency: latency,
        supervision_timeout: Duration::from_millis(timeout_ms),
        ..Default::default()
    };
    if let Err(e) = conn.update_connection_params(stack, &params).await {
        warn!("update: connection parameters: {:?}", e);
    }
}

pub async fn offer(
    stack: &Stack<'_, '_>,
    conn: &Connection<'_, DefaultPacketPool>,
    rram: &'static Rram,
    params: Option<TransferParams>,
) {
    let Some((header, at)) = firmware::servable() else {
        return;
    };
    if at == ACTIVE && !firmware::confirmed(rram).await {
        info!("update: not offering {}: not confirmed here yet", header.firmware);
        return;
    }
    info!("update: offering {}", header.firmware);
    let mut channel = match L2capChannel::create(stack, conn, UPDATE_SPSM, &Default::default()).await {
        Ok(channel) => channel,
        Err(e) => {
            warn!("update: channel: {:?}", e);
            return;
        }
    };
    let mut buf = [0u8; UPDATE_CHUNK + 5];
    if channel
        .send(stack, UpdateMsg::Offer(header).encode(&mut buf))
        .await
        .is_err()
    {
        return;
    }
    let mut answer = [0u8; 16];
    let from = match with_timeout(ANSWER_WITHIN, channel.receive(stack, &mut answer)).await {
        Ok(Ok(n)) => match UpdateMsg::decode(&answer[..n]) {
            Some(UpdateMsg::Want { from }) => from as usize,
            Some(UpdateMsg::Decline) => {
                info!("update: the peer declined {}", header.firmware);
                return;
            }
            _ => return,
        },
        _ => return,
    };
    if from > header.len as usize {
        warn!(
            "update: the peer wants {} from byte {=usize}, past its end",
            header.firmware, from
        );
        return;
    }
    // A fast link for the transfer: an anchor link idles at 1 s, which would take half an hour.
    // Not on a link the policy doesn't know (a peer on another build): the request ended one
    // such link with an LL response timeout (log 2026-11-04).
    if let Some(params) = params {
        request_params(stack, conn, params.fast).await;
    }
    let done = stream(stack, &mut channel, &mut buf, &mut answer, header, at, from).await;
    if let (false, Some(params)) = (done, params) {
        request_params(stack, conn, params.back).await;
    }
}

/// Streams the image from byte `from` and waits for the peer's `Done`: `true` once it has it.
async fn stream(
    stack: &Stack<'_, '_>,
    channel: &mut L2capChannel<'_, DefaultPacketPool>,
    buf: &mut [u8; UPDATE_CHUNK + 5],
    answer: &mut [u8; 16],
    header: UpdateHeader,
    at: u32,
    from: usize,
) -> bool {
    info!("update: sending {} from byte {=usize}", header.firmware, from);
    // SAFETY: memory-mapped RRAM: the running image, which nothing writes while it runs, or the
    // update slot, checked below before each chunk.
    let image = unsafe { core::slice::from_raw_parts(at as *const u8, header.len as usize) };
    let t = Instant::now();
    let mut offset = from;
    while offset < image.len() {
        // A sink's update slot changes when the host sends another update.
        if at == DFU && firmware::carried() != Some(header) {
            warn!("update: no longer carrying {}, stopping", header.firmware);
            return false;
        }
        let end = (offset + UPDATE_CHUNK).min(image.len());
        let data = UpdateMsg::Data {
            offset: offset as u32,
            bytes: &image[offset..end],
        };
        let mut send = core::pin::pin!(channel.send(stack, data.encode(&mut buf[..])));
        let sent = match with_timeout(STALL_AFTER, send.as_mut()).await {
            Ok(sent) => sent,
            Err(_) => {
                warn!("update: sending at byte {=usize}: stalled, still waiting", offset);
                match with_timeout(GIVE_UP_AFTER, send.as_mut()).await {
                    Ok(sent) => sent,
                    Err(_) => {
                        warn!("update: sending at byte {=usize}: given up", offset);
                        return false;
                    }
                }
            }
        };
        if let Err(e) = sent {
            warn!("update: sending at byte {=usize}: {:?}", offset, e);
            return false;
        }
        if end / LOG_EVERY != offset / LOG_EVERY {
            info!("update: sent {=usize} KiB", end / 1024);
        }
        offset = end;
    }
    if channel.send(stack, UpdateMsg::Done.encode(&mut buf[..])).await.is_err() {
        return false;
    }
    match with_timeout(ANSWER_WITHIN, channel.receive(stack, &mut answer[..])).await {
        Ok(Ok(n)) if UpdateMsg::decode(&answer[..n]) == Some(UpdateMsg::Done) => {
            info!("update: sent {} in {=u64} ms", header.firmware, t.elapsed().as_millis());
            true
        }
        _ => {
            warn!("update: the peer didn't confirm {}", header.firmware);
            false
        }
    }
}

/// Takes an update channel a peer opened on `conn`, accepted by the link's channel dispatcher
/// (`mesh`). With `widen`, the link switches to full-length packets for the transfer and back
/// after: on a slow link one full-length exchange fills an event, which would cap the telemetry
/// (DEF-42). Returns when the transfer fails or the channel wasn't an update after all; a
/// transfer that succeeds restarts the node into the update.
pub async fn take(
    stack: &Stack<'_, '_>,
    conn: &Connection<'_, DefaultPacketPool>,
    mut channel: L2capChannel<'_, DefaultPacketPool>,
    rram: &'static Rram,
    widen: bool,
) {
    info!("update: channel {=u16:#06x} opened by the peer", channel.spsm());
    if channel.spsm() != UPDATE_SPSM {
        return;
    }
    let Some((header, from)) = answer(stack, &mut channel, rram).await else {
        return;
    };
    if widen {
        packet_length(stack, conn, FULL_LENGTH).await;
    }
    // Returns only if the transfer failed; otherwise the node restarts into the update.
    receive(stack, &mut channel, rram, header, from).await;
    TAKING.store(false, Ordering::Release);
    if widen {
        packet_length(stack, conn, SHORT_LENGTH).await;
    }
}

/// Set while this node takes an update: a node listens on several links (a reflector on its CS
/// link and its sink link), and two transfers would write the one update slot.
static TAKING: AtomicBool = AtomicBool::new(false);

/// Credits go back four chunks at a time rather than after each (TrouBLE's default), out of the
/// 8 the sender starts with. On the sink link the node sends one packet per event during a
/// transfer, and a credit packet per chunk would take every one of them from the telemetry.
pub(crate) const RECEIVING: L2capChannelConfig = L2capChannelConfig {
    mtu: None,
    mps: None,
    flow_policy: CreditFlowPolicy::Every(4),
    initial_credits: None,
};

/// Full-length packets for a transfer: 251 octets, 2120 µs.
const FULL_LENGTH: (u16, u16) = (251, 2120);
/// The short ones otherwise: 27 octets, 328 µs.
const SHORT_LENGTH: (u16, u16) = (27, 328);

async fn packet_length(stack: &Stack<'_, '_>, conn: &Connection<'_, DefaultPacketPool>, (octets, us): (u16, u16)) {
    if let Err(e) = conn.update_data_length(stack, octets, us).await {
        warn!("update: data length {=u16}: {:?}", octets, e);
    }
}

/// Reads the peer's offer and answers it: the header and the byte asked for from (the end of a
/// transfer of the same build that broke off, else 0), or `None` once declined.
async fn answer(
    stack: &Stack<'_, '_>,
    channel: &mut L2capChannel<'_, DefaultPacketPool>,
    rram: &'static Rram,
) -> Option<(UpdateHeader, usize)> {
    let mut rx = [0u8; UPDATE_HEADER_LEN + 1];
    let mut tx = [0u8; 16];
    let n = channel.receive(stack, &mut rx).await.ok()?;
    let Some(UpdateMsg::Offer(header)) = UpdateMsg::decode(&rx[..n]) else {
        return None;
    };
    let wanted = header.board == BOARD
        && header.firmware != crate::FIRMWARE
        && !reverted(&header)
        && by_controller(&header)
        && takes(&header, rram).await;
    let busy = wanted && TAKING.swap(true, Ordering::Acquire);
    if busy {
        info!(
            "update: declining {}: taking an update on another link",
            header.firmware
        );
    }
    if !wanted || busy {
        let _ = channel.send(stack, UpdateMsg::Decline.encode(&mut tx)).await;
        return None;
    }
    let from = PARTIAL
        .lock(|p| p.get())
        .filter(|(h, _)| *h == header)
        .map_or(0, |(_, done)| done);
    info!(
        "update: receiving {} ({=u32} bytes) from byte {=usize}",
        header.firmware, header.len, from
    );
    channel
        .send(stack, UpdateMsg::Want { from: from as u32 }.encode(&mut tx))
        .await
        .ok()?;
    Some((header, from))
}

/// Whether the controller, if it has named the build for this node, named `header`'s, saying
/// so otherwise.
fn by_controller(header: &UpdateHeader) -> bool {
    match wanted() {
        Some(firmware) if firmware != header.firmware => {
            info!(
                "update: declining {}: the controller wants {} here",
                header.firmware, firmware
            );
            false
        }
        _ => true,
    }
}

/// Takes the image after [`answer`], from byte `from`, and restarts into the check at boot once
/// it's stored. A transfer that breaks off is remembered, to go on from its last whole page at
/// the next offer.
async fn receive(
    stack: &Stack<'_, '_>,
    channel: &mut L2capChannel<'_, DefaultPacketPool>,
    rram: &'static Rram,
    header: UpdateHeader,
    from: usize,
) {
    let mut page = LINK_PAGE.lock().await;
    let mut writer = Writer::from(from, &mut page);
    if !store(stack, channel, rram, &header, &mut writer).await {
        let done = writer.next - writer.next % PAGE;
        PARTIAL.lock(|p| p.set((done > 0).then_some((header, done))));
        warn!(
            "update: {} broke off at byte {=usize}; {=usize} kept",
            header.firmware, writer.next, done
        );
    }
}

/// Takes the rest of the image into `writer`, stores its end and restarts; `false` if the
/// transfer broke off.
async fn store(
    stack: &Stack<'_, '_>,
    channel: &mut L2capChannel<'_, DefaultPacketPool>,
    rram: &'static Rram,
    header: &UpdateHeader,
    writer: &mut Writer<'_>,
) -> bool {
    let mut rx = [0u8; UPDATE_CHUNK + 5];
    let mut tx = [0u8; 16];
    let len = header.len as usize;
    let t = Instant::now();
    loop {
        let received = {
            let mut receiving = core::pin::pin!(channel.receive(stack, &mut rx));
            match with_timeout(STALL_AFTER, receiving.as_mut()).await {
                Ok(received) => received,
                Err(_) => {
                    warn!(
                        "update: receiving at byte {=usize}: stalled, still waiting",
                        writer.next
                    );
                    match with_timeout(GIVE_UP_AFTER, receiving.as_mut()).await {
                        Ok(received) => received,
                        Err(_) => {
                            warn!("update: receiving at byte {=usize}: given up", writer.next);
                            return false;
                        }
                    }
                }
            }
        };
        let n = match received {
            Ok(n) => n,
            Err(e) => {
                warn!("update: receiving at byte {=usize}: {:?}", writer.next, e);
                return false;
            }
        };
        match UpdateMsg::decode(&rx[..n]) {
            Some(UpdateMsg::Data { offset, bytes })
                if offset as usize == writer.next && writer.next + bytes.len() <= len =>
            {
                if !writer.put(rram, bytes).await {
                    return false;
                }
            }
            Some(UpdateMsg::Done) if writer.next == len => {
                info!("update: all {=usize} bytes in, storing the end", len);
                break;
            }
            _ => {
                warn!("update: unexpected message at byte {=usize}", writer.next);
                return false;
            }
        }
    }
    if !writer.flush(rram).await || !stage(rram, header).await {
        return false;
    }
    let _ = channel.send(stack, UpdateMsg::Done.encode(&mut tx)).await;
    info!(
        "update: received {} in {=u64} ms, restarting to check it",
        header.firmware,
        t.elapsed().as_millis()
    );
    // The sender closes the channel once it has the `Done`. On a slow link with a backlog, that
    // can take a while; restarting first would lose it.
    let _ = with_timeout(ANSWER_WITHIN, channel.receive(stack, &mut rx)).await;
    cortex_m::peripheral::SCB::sys_reset();
}

/// Whether this node reverted from `header`'s build, saying so.
fn reverted(header: &UpdateHeader) -> bool {
    let reverted = firmware::reverted_from() == Some(header.firmware);
    if reverted {
        info!("update: declining {}: this node reverted from it", header.firmware);
    }
    reverted
}

/// Whether this node may write its update slot for `header`, saying why not.
async fn takes(header: &UpdateHeader, rram: &'static Rram) -> bool {
    if header.len > ACTIVE_LEN {
        info!(
            "update: declining {}: {=u32} bytes is too long",
            header.firmware, header.len
        );
        false
    } else if !firmware::confirmed(rram).await {
        info!("update: declining {}: this image isn't confirmed yet", header.firmware);
        false
    } else {
        true
    }
}

/// The page a transfer over a link collects into: one, as one transfer runs at a time
/// (`TAKING`); in a static rather than in each link's future.
static LINK_PAGE: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, [u8; PAGE]> =
    embassy_sync::mutex::Mutex::new([0; PAGE]);

/// Collects an image into the update slot a page at a time.
struct Writer<'p> {
    page: &'p mut [u8; PAGE],
    /// Bytes of the image taken so far.
    next: usize,
}

impl<'p> Writer<'p> {
    /// Goes on from byte `done`, a whole number of pages already in the slot.
    fn from(done: usize, page: &'p mut [u8; PAGE]) -> Self {
        let mut writer = Self { page, next: 0 };
        writer.reset(done);
        writer
    }

    fn reset(&mut self, done: usize) {
        debug_assert!(done.is_multiple_of(PAGE));
        self.page.fill(0xff);
        self.next = done;
    }

    async fn put(&mut self, rram: &Rram, mut bytes: &[u8]) -> bool {
        while !bytes.is_empty() {
            let at = self.next % PAGE;
            let take = (PAGE - at).min(bytes.len());
            self.page[at..at + take].copy_from_slice(&bytes[..take]);
            bytes = &bytes[take..];
            self.next += take;
            if self.next.is_multiple_of(PAGE) {
                if !write(rram, DFU + (self.next - PAGE) as u32, self.page).await {
                    return false;
                }
                self.page.fill(0xff);
            }
            if self.next.is_multiple_of(LOG_EVERY) {
                info!("update: {=usize} KiB", self.next / 1024);
            }
        }
        true
    }

    /// Writes the last, partial page in whole 16-byte lines.
    async fn flush(&mut self, rram: &Rram) -> bool {
        let tail = self.next % PAGE;
        tail == 0
            || write(
                rram,
                DFU + (self.next - tail) as u32,
                &self.page[..tail.next_multiple_of(16)],
            )
            .await
    }
}

/// Writes `bytes` to RRAM, in MPSL timeslots.
async fn write(rram: &Rram, at: u32, bytes: &[u8]) -> bool {
    let t = Instant::now();
    let written = with_timeout(WRITE_WITHIN, async { rram.lock().await.write(at, bytes).await }).await;
    let ms = t.elapsed().as_millis();
    match written {
        Ok(Ok(())) => {
            if ms > SLOW_WRITE_MS {
                info!(
                    "update: writing {=usize} bytes at {=u32:#x} took {=u64} ms",
                    bytes.len(),
                    at,
                    ms
                );
            }
            true
        }
        Ok(Err(e)) => {
            warn!("update: writing at {=u32:#x}: {:?}", at, e);
            false
        }
        Err(_) => {
            warn!("update: writing at {=u32:#x}: not done within {=u64} ms", at, ms);
            false
        }
    }
}

/// Puts the header in the staging page, for the check at boot.
async fn stage(rram: &Rram, header: &UpdateHeader) -> bool {
    let mut line = [0xffu8; 96];
    line[..UPDATE_HEADER_LEN].copy_from_slice(&header.encode());
    let staged = with_timeout(WRITE_WITHIN, async {
        let mut rram = rram.lock().await;
        rram.erase(STAGED_UPDATE, STAGED_UPDATE + STAGED_UPDATE_LEN).await?;
        rram.write(STAGED_UPDATE, &line).await
    })
    .await;
    match staged {
        Ok(Ok(())) => true,
        Ok(Err(e)) => {
            warn!("update: staging the header: {:?}", e);
            false
        }
        Err(_) => {
            warn!(
                "update: staging the header: not done within {=u64} s",
                WRITE_WITHIN.as_secs()
            );
            false
        }
    }
}

/// The sink's side: updates from the host.
#[cfg(feature = "board-dk")]
mod sink {
    use embassy_nrf::uarte::UarteRxWithIdle;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::watch::Watch;
    use embassy_time::Timer;
    use navgrid_proto::{DOWNLINK_CHUNK, MAX_DOWNLINK_FRAME, Message, UpdateAnswer};

    use super::*;
    use crate::report::{self, SinkUart};

    /// Changes when the sink starts carrying an update, so every link it serves offers it at once.
    /// One receiver per link slot, and as many again for links being torn down while new
    /// ones start; a slot without a receiver never offers (log 2026-11-04).
    pub static CARRYING: Watch<CriticalSectionRawMutex, (), { 2 * navgrid_link::MAX_LINKS }> = Watch::new();

    /// An update arriving from the host.
    struct Transfer {
        header: UpdateHeader,
        stored: bool,
        started: Instant,
    }

    /// The page the sink collects the host's uploads into.
    static HOST_PAGE: static_cell::StaticCell<[u8; PAGE]> = static_cell::StaticCell::new();

    /// Takes updates the host sends over the serial port (`navgrid update --sink`), answering each
    /// message on `tx`. One for this board restarts the sink into the check at boot; one for another
    /// board is kept, and offered to the nodes on the sink's links from then on.
    pub async fn from_host(
        rx: &mut UarteRxWithIdle<'_>,
        tx: &SinkUart<'_>,
        rram: &'static Rram,
        signing_key: Option<[u8; 32]>,
    ) -> ! {
        let mut frame = [0u8; MAX_DOWNLINK_FRAME];
        let mut filled = 0;
        let mut transfer = None;
        let mut writer = Writer::from(0, HOST_PAGE.init([0xff; PAGE]));
        loop {
            // Frames come one at a time, each after the answer to the last, so a read that ends at an
            // idle line has the rest of a frame or nothing after it.
            let n = match rx.read_until_idle(&mut frame[filled..]).await {
                Ok(n) => n,
                Err(e) => {
                    warn!("update: serial: {:?}", e);
                    filled = 0;
                    continue;
                }
            };
            let start = filled;
            filled += n;
            let Some(end) = frame[start..filled].iter().position(|&b| b == 0).map(|i| start + i) else {
                if filled == frame.len() {
                    filled = 0;
                }
                continue;
            };
            filled = 0;
            // A damaged frame gets no answer: the host sends it again.
            let Some(msg) = navgrid_proto::decode_downlink(&mut frame[..end]) else {
                continue;
            };
            let Some(answer) = take(msg, &mut transfer, &mut writer, rram, signing_key).await else {
                continue;
            };
            report::send(tx, &Message::Update(answer)).await;
            if let (UpdateAnswer::Done, Some(t)) = (answer, &transfer)
                && t.header.board == BOARD
            {
                info!("update: restarting to check {}", t.header.firmware);
                Timer::after_millis(100).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }

    /// One message from the host, and the answer to it, if any.
    async fn take(
        msg: UpdateMsg<'_>,
        transfer: &mut Option<Transfer>,
        writer: &mut Writer<'_>,
        rram: &'static Rram,
        signing_key: Option<[u8; 32]>,
    ) -> Option<UpdateAnswer> {
        match msg {
            UpdateMsg::Record(bytes) => {
                crate::state::inject(bytes);
                None
            }
            UpdateMsg::Offer(header) => {
                *transfer = None;
                let new = if header.board == BOARD {
                    header.firmware != crate::FIRMWARE && !reverted(&header)
                } else {
                    firmware::carried() != Some(header)
                };
                if !new {
                    info!("update: declining {}: already running or carrying it", header.firmware);
                    return Some(UpdateAnswer::Decline);
                }
                // The slot is about to change: stop passing on what it held, and forget a
                // transfer into it that broke off.
                if !takes(&header, rram).await || !unstage(rram).await {
                    return Some(UpdateAnswer::Decline);
                }
                PARTIAL.lock(|p| p.set(None));
                info!(
                    "update: receiving {} for the {} board from the host ({=u32} bytes)",
                    header.firmware,
                    header.board.name(),
                    header.len
                );
                writer.reset(0);
                *transfer = Some(Transfer {
                    header,
                    stored: false,
                    started: Instant::now(),
                });
                Some(UpdateAnswer::Want { from: 0 })
            }
            UpdateMsg::Data { offset, bytes } => {
                let Some(t) = transfer.as_mut() else {
                    return Some(UpdateAnswer::Decline);
                };
                let next = writer.next;
                let expected = (t.header.len as usize - next).min(DOWNLINK_CHUNK);
                let offset = offset as usize;
                if offset == next && bytes.len() == expected && expected > 0 && !t.stored {
                    if !writer.put(rram, bytes).await {
                        *transfer = None;
                        return Some(UpdateAnswer::Decline);
                    }
                } else if offset >= next {
                    // Damaged on the way: the host sends it again.
                    return None;
                }
                // Stored, or stored before and its answer lost.
                Some(UpdateAnswer::Want {
                    from: writer.next as u32,
                })
            }
            UpdateMsg::Done => {
                let Some(t) = transfer.as_mut() else {
                    return Some(UpdateAnswer::Decline);
                };
                if writer.next != t.header.len as usize {
                    return Some(UpdateAnswer::Decline);
                }
                if !t.stored {
                    // The whole image first: the last page is still in the writer.
                    let stored = writer.flush(rram).await
                        && verify(&t.header, signing_key).await
                        && stage(rram, &t.header).await;
                    if !stored {
                        *transfer = None;
                        return Some(UpdateAnswer::Decline);
                    }
                    t.stored = true;
                    info!(
                        "update: {} from the host stored and checked in {=u64} ms",
                        t.header.firmware,
                        t.started.elapsed().as_millis()
                    );
                    if t.header.board != BOARD {
                        CARRYING.sender().send(());
                    }
                }
                Some(UpdateAnswer::Done)
            }
            UpdateMsg::Want { .. } | UpdateMsg::Decline => None,
        }
    }

    /// Checks the image in the update slot against `header`'s signature, as the check at boot does:
    /// Ed25519 over its SHA-512 digest.
    async fn verify(header: &UpdateHeader, signing_key: Option<[u8; 32]>) -> bool {
        use ed25519_dalek::{Digest, Sha512, Signature, Verifier, VerifyingKey};

        let Some(key) = signing_key.and_then(|k| VerifyingKey::from_bytes(&k).ok()) else {
            warn!("update: no signing key provisioned, declining {}", header.firmware);
            return false;
        };
        let t = Instant::now();
        // SAFETY: memory-mapped RRAM, just written; nothing else writes the slot meanwhile.
        let image = unsafe { core::slice::from_raw_parts(DFU as *const u8, header.len as usize) };
        let mut digest = Sha512::new();
        for page in image.chunks(PAGE) {
            digest.update(page);
            // The links' tasks run between pages.
            embassy_futures::yield_now().await;
        }
        let good = key
            .verify(&digest.finalize(), &Signature::from_bytes(&header.signature))
            .is_ok();
        if good {
            info!(
                "update: {} verified in {=u64} ms",
                header.firmware,
                t.elapsed().as_millis()
            );
        } else {
            warn!("update: {} doesn't match its signature, declining it", header.firmware);
        }
        good
    }

    /// Clears the staging page, before the update slot is written again.
    async fn unstage(rram: &Rram) -> bool {
        let cleared = with_timeout(WRITE_WITHIN, async {
            rram.lock()
                .await
                .erase(STAGED_UPDATE, STAGED_UPDATE + STAGED_UPDATE_LEN)
                .await
        })
        .await;
        match cleared {
            Ok(Ok(())) => true,
            Ok(Err(e)) => {
                warn!("update: clearing the staging page: {:?}", e);
                false
            }
            Err(_) => {
                warn!(
                    "update: clearing the staging page: not done within {=u64} s",
                    WRITE_WITHIN.as_secs()
                );
                false
            }
        }
    }
}
