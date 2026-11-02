//! Firmware updates between neighbours (architecture §13, D-36), single hop.
//!
//! - **Sending:** a node that runs an installed update offers it to a peer over a fleet link
//!   ([`offer`]).
//! - **Receiving:** a peer that runs another build takes it ([`listen`]). It writes the image into
//!   its update slot and the header into its staging page, then restarts into the check at boot
//!   (`firmware::apply_staged`), which hands a good one to the bootloader.

use defmt::{info, warn};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use navgrid_proto::layout::{ACTIVE, ACTIVE_LEN, DFU, STAGED_UPDATE, STAGED_UPDATE_LEN};
use navgrid_proto::{UPDATE_CHUNK, UPDATE_HEADER_LEN, UPDATE_SPSM, UpdateHeader, UpdateMsg};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

use crate::firmware::{self, Rram};

type Stack<'a, 'd> = trouble_host::Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

/// The receiver writes the update slot a page at a time.
const PAGE: usize = 4096;
/// How long a peer may take to answer an offer, and to store the end of the image.
const ANSWER_WITHIN: Duration = Duration::from_secs(5);
/// Progress is logged every this many bytes.
const LOG_EVERY: usize = 64 * 1024;

/// Offers this node's installed update, if it has one, to the peer on `conn`, and streams it if
/// the peer wants it.
pub async fn offer(stack: &Stack<'_, '_>, conn: &Connection<'_, DefaultPacketPool>) {
    let Some(header) = firmware::servable() else {
        return;
    };
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
    info!("update: sending {} from byte {=usize}", header.firmware, from);
    // SAFETY: memory-mapped RRAM, the running image, which nothing writes while it runs.
    let image = unsafe { core::slice::from_raw_parts(ACTIVE as *const u8, header.len as usize) };
    let t = Instant::now();
    let mut offset = from;
    while offset < image.len() {
        let end = (offset + UPDATE_CHUNK).min(image.len());
        let data = UpdateMsg::Data {
            offset: offset as u32,
            bytes: &image[offset..end],
        };
        if let Err(e) = channel.send(stack, data.encode(&mut buf)).await {
            warn!("update: sending at byte {=usize}: {:?}", offset, e);
            return;
        }
        offset = end;
    }
    if channel.send(stack, UpdateMsg::Done.encode(&mut buf)).await.is_err() {
        return;
    }
    match with_timeout(ANSWER_WITHIN, channel.receive(stack, &mut answer)).await {
        Ok(Ok(n)) if UpdateMsg::decode(&answer[..n]) == Some(UpdateMsg::Done) => {
            info!("update: sent {} in {=u64} ms", header.firmware, t.elapsed().as_millis())
        }
        _ => warn!("update: the peer didn't confirm {}", header.firmware),
    }
}

/// Takes the update channels a peer opens on `conn`. Never returns: the link's own task ends when
/// it drops.
pub async fn listen(stack: &Stack<'_, '_>, conn: &Connection<'_, DefaultPacketPool>, rram: &'static Rram) -> ! {
    loop {
        let mut channel = match L2capChannel::listen(stack, conn).accept(&Default::default()).await {
            Ok(channel) => channel,
            Err(_) => core::future::pending().await,
        };
        if channel.spsm() == UPDATE_SPSM {
            receive(stack, &mut channel, rram).await;
        }
    }
}

async fn receive(stack: &Stack<'_, '_>, channel: &mut L2capChannel<'_, DefaultPacketPool>, rram: &'static Rram) {
    let mut rx = [0u8; UPDATE_CHUNK + 5];
    let mut tx = [0u8; 16];
    let Ok(n) = channel.receive(stack, &mut rx).await else {
        return;
    };
    let Some(UpdateMsg::Offer(header)) = UpdateMsg::decode(&rx[..n]) else {
        return;
    };
    let unconfirmed = !firmware::confirmed(rram).await;
    if header.firmware == crate::FIRMWARE || header.len > ACTIVE_LEN || unconfirmed {
        if unconfirmed {
            info!("update: declining {}: this image isn't confirmed yet", header.firmware);
        }
        let _ = channel.send(stack, UpdateMsg::Decline.encode(&mut tx)).await;
        return;
    }
    info!("update: receiving {} ({=u32} bytes)", header.firmware, header.len);
    if channel
        .send(stack, UpdateMsg::Want { from: 0 }.encode(&mut tx))
        .await
        .is_err()
    {
        return;
    }
    let len = header.len as usize;
    let mut page = [0xffu8; PAGE];
    let mut next = 0usize;
    let t = Instant::now();
    loop {
        let n = match channel.receive(stack, &mut rx).await {
            Ok(n) => n,
            Err(e) => {
                warn!("update: receiving at byte {=usize}: {:?}", next, e);
                return;
            }
        };
        match UpdateMsg::decode(&rx[..n]) {
            Some(UpdateMsg::Data { offset, bytes }) if offset as usize == next && next + bytes.len() <= len => {
                let mut src = bytes;
                while !src.is_empty() {
                    let at = next % PAGE;
                    let take = (PAGE - at).min(src.len());
                    page[at..at + take].copy_from_slice(&src[..take]);
                    src = &src[take..];
                    next += take;
                    if next.is_multiple_of(PAGE) {
                        if !write(rram, DFU + (next - PAGE) as u32, &page).await {
                            return;
                        }
                        page.fill(0xff);
                    }
                    if next.is_multiple_of(LOG_EVERY) {
                        info!("update: {=usize} KiB", next / 1024);
                    }
                }
            }
            Some(UpdateMsg::Done) if next == len => break,
            _ => {
                warn!("update: unexpected message at byte {=usize}", next);
                return;
            }
        }
    }
    // The last, partial page, in whole 16-byte lines.
    let tail = next % PAGE;
    if tail != 0 && !write(rram, DFU + (next - tail) as u32, &page[..tail.next_multiple_of(16)]).await {
        return;
    }
    if !stage(rram, &header).await {
        return;
    }
    let _ = channel.send(stack, UpdateMsg::Done.encode(&mut tx)).await;
    info!(
        "update: received {} in {=u64} ms, restarting to check it",
        header.firmware,
        t.elapsed().as_millis()
    );
    Timer::after_millis(100).await;
    cortex_m::peripheral::SCB::sys_reset();
}

/// Writes `bytes` to RRAM, in MPSL timeslots.
async fn write(rram: &Rram, at: u32, bytes: &[u8]) -> bool {
    match rram.lock().await.write(at, bytes).await {
        Ok(()) => true,
        Err(e) => {
            warn!("update: writing at {=u32:#x}: {:?}", at, e);
            false
        }
    }
}

/// Puts the header in the staging page, for the check at boot.
async fn stage(rram: &Rram, header: &UpdateHeader) -> bool {
    let mut line = [0xffu8; 96];
    line[..UPDATE_HEADER_LEN].copy_from_slice(&header.encode());
    let mut rram = rram.lock().await;
    let staged = match rram.erase(STAGED_UPDATE, STAGED_UPDATE + STAGED_UPDATE_LEN).await {
        Ok(()) => rram.write(STAGED_UPDATE, &line).await,
        Err(e) => Err(e),
    };
    if let Err(e) = staged {
        warn!("update: staging the header: {:?}", e);
    }
    staged.is_ok()
}
