//! Crypto drivers for TrouBLE's security manager. Pairing (LE Secure Connections) is needed
//! because Channel Sounding only starts on an encrypted link.
//!
//! AES-128 and P-256 come from the RustCrypto software drivers. The RNG driver is a ChaCha20
//! CSPRNG seeded from CRACEN at boot: CRACEN itself belongs to the SoftDevice Controller once the
//! controller is built.

use core::cell::RefCell;

use defmt::unwrap;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use rand_chacha::ChaCha20Rng;
use rand_chacha::rand_core::{RngCore, SeedableRng};
// Linking the crate registers its AES-128 ECB/CMAC and P-256 ECDH drivers.
use embassy_crypto_rustcrypto as _;

static RNG: CriticalSectionMutex<RefCell<Option<ChaCha20Rng>>> = CriticalSectionMutex::new(RefCell::new(None));

/// Seeds the RNG driver. Call once at boot with 32 bytes from CRACEN, before pairing can happen.
pub fn seed(seed: [u8; 32]) {
    RNG.lock(|rng| *rng.borrow_mut() = Some(ChaCha20Rng::from_seed(seed)));
}

struct Rng;

impl embassy_crypto::driver::Rng for Rng {
    fn fill_bytes(buf: &mut [u8]) {
        RNG.lock(|rng| unwrap!(rng.borrow_mut().as_mut(), "crypto RNG used before seeding").fill_bytes(buf));
    }
}

embassy_crypto::rng_impl!(Rng);
