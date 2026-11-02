//! Keys derived from the fleet key (D-7, NFR-7). Fleet members encrypt their links with keys
//! both ends can derive, so they never pair.

use aes::Aes128;
use cmac::{Cmac, KeyInit, Mac};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

fn cmac(key: &[u8; 16], parts: &[&[u8]]) -> [u8; 16] {
    let mut mac = <Cmac<Aes128> as KeyInit>::new_from_slice(key).expect("AES-128 takes 16-byte keys");
    for part in parts {
        mac.update(part);
    }
    mac.finalize().into_bytes().into()
}

/// The long-term key of a link between two fleet devices: AES-CMAC under the fleet key of a
/// label and both device addresses, the numerically lower first, so both ends get the same key.
pub fn link_key(fleet: &[u8; 16], a: [u8; 6], b: [u8; 6]) -> u128 {
    let value = |x: [u8; 6]| {
        let mut v = [0u8; 8];
        v[..6].copy_from_slice(&x);
        u64::from_le_bytes(v)
    };
    let (lo, hi) = if value(a) <= value(b) { (a, b) } else { (b, a) };
    u128::from_le_bytes(cmac(fleet, &[b"navgrid link key", &lo, &hi]))
}

/// This node's fleet membership: its address, and the fleet key once provisioned.
pub struct Fleet {
    own: [u8; 6],
    key: Option<[u8; 16]>,
}

impl Fleet {
    pub fn new(own: [u8; 6], key: Option<[u8; 16]>) -> Self {
        Self { own, key }
    }

    /// Unprovisioned nodes pair Just Works instead, for bring-up.
    pub fn provisioned(&self) -> bool {
        self.key.is_some()
    }

    /// Lets a link to `peer` encrypt under the fleet's link key without pairing. A central calls
    /// it before connecting, a peripheral as soon as it accepts. TrouBLE forgets the key when the
    /// link drops, as it isn't bonded.
    pub fn trust_address(&self, stack: &Stack<'_, SoftdeviceController<'_>, DefaultPacketPool>, peer: [u8; 6]) {
        let Some(key) = &self.key else {
            return;
        };
        let identity = Identity {
            addr: Address::random(peer),
            irk: None,
        };
        let ltk = LongTermKey::new(link_key(key, self.own, peer));
        let bond = BondInformation::new(identity, ltk, SecurityLevel::Encrypted, false);
        if let Err(e) = stack.add_bond_information(bond) {
            defmt::warn!("fleet key for {=[u8]:02x}: {:?}", peer, e);
        }
    }

    /// [`trust_address`](Self::trust_address) for the peer of an accepted connection.
    pub fn trust(
        &self,
        stack: &Stack<'_, SoftdeviceController<'_>, DefaultPacketPool>,
        conn: &Connection<'_, DefaultPacketPool>,
    ) {
        if let Ok(peer) = conn.peer_address().addr.raw().try_into() {
            self.trust_address(stack, peer);
        }
    }
}
