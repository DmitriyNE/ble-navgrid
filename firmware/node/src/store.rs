//! What survives a reboot, in the two RRAM pages `memory.x` keeps out of the image:
//!
//! - the **provisioning** page at [`PROVISIONING_ADDRESS`], which `navgrid provision` writes
//!   through the probe and the firmware only reads;
//! - the **boot epoch** log, which numbers this node's boots for its mesh record versions. Each
//!   boot writes the next 16-byte RRAM line, so any one line is rewritten once every 256 boots.

use embassy_nrf::rramc::{self, Nvmc};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use navgrid_proto::{MAX_PROVISIONING_LEN, PROVISIONING_ADDRESS, Provisioning, ProvisioningError};

const EPOCH_ADDRESS: u32 = PROVISIONING_ADDRESS + rramc::PAGE_SIZE as u32;
/// One RRAM write line per slot.
const SLOT_LEN: u32 = 16;
const SLOTS: u32 = rramc::PAGE_SIZE as u32 / SLOT_LEN;

pub fn provisioning() -> Result<Provisioning, ProvisioningError> {
    // SAFETY: memory-mapped RRAM that this firmware never writes.
    let page = unsafe { core::slice::from_raw_parts(PROVISIONING_ADDRESS as *const u8, MAX_PROVISIONING_LEN) };
    navgrid_proto::decode_provisioning(page)
}

/// Starts a new boot epoch: one more than the highest in the log, recorded in the slot after it.
/// A slot holds the epoch and its complement, so a blank or torn slot doesn't count.
pub fn next_epoch(rram: &mut Nvmc<'_>) -> Result<u32, rramc::Error> {
    let mut last: Option<(u32, u32)> = None;
    for slot in 0..SLOTS {
        let mut b = [0u8; 8];
        rram.read(EPOCH_ADDRESS + slot * SLOT_LEN, &mut b)?;
        let epoch = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        let check = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
        if epoch != u32::MAX && check == !epoch && last.is_none_or(|(e, _)| epoch > e) {
            last = Some((epoch, slot));
        }
    }
    let (epoch, slot) = match last {
        Some((e, s)) => (e + 1, (s + 1) % SLOTS),
        None => (1, 0),
    };
    let mut line = [0xffu8; SLOT_LEN as usize];
    line[..4].copy_from_slice(&epoch.to_le_bytes());
    line[4..8].copy_from_slice(&(!epoch).to_le_bytes());
    rram.write(EPOCH_ADDRESS + slot * SLOT_LEN, &line)?;
    Ok(epoch)
}
