//! Board support: regulator, crystal load capacitors and the Channel Sounding antenna switch.
//!
//! Exactly one `board-*` feature selects the board. Values come from the boards' devicetrees in
//! NCS v3.4.1 (`boards/nordic/nrf54l15dk`, `boards/nordic/nrf54l15tag`).

use embassy_nrf::pac;

#[cfg(all(feature = "board-dk", feature = "board-tag"))]
compile_error!("select exactly one board feature");
#[cfg(not(any(feature = "board-dk", feature = "board-tag")))]
compile_error!("select a board: --features board-dk or --features board-tag");

#[cfg(feature = "board-dk")]
mod consts {
    pub const NAME: &str = "nRF54L15 DK";
    pub const LFXO_LOAD_FF: u32 = 17_000;
    pub const HFXO_LOAD_FF: u32 = 15_000;
    pub const ANTENNAS: u8 = 1;
    pub const ANTENNA_SWITCH: Option<extern "C" fn(u8)> = None;
}

#[cfg(feature = "board-tag")]
mod consts {
    pub const NAME: &str = "nRF54L15 TAG";
    pub const LFXO_LOAD_FF: u32 = 9_000;
    pub const HFXO_LOAD_FF: u32 = 8_000;
    pub const ANTENNAS: u8 = 2;
    pub const ANTENNA_SWITCH: Option<extern "C" fn(u8)> = Some(super::antenna::switch);
}

pub use consts::*;

/// Turns on DC/DC and sets the crystal load capacitors. Must run before `embassy_nrf::init`
/// starts the clocks. embassy-nrf has no nRF54L option for either yet (architecture §6, P-5), so
/// this ports the equivalent part of Zephyr's `soc/nordic/nrf54l/soc.c` (ncs-v3.4.1).
pub fn init_power_and_clocks() {
    // Both boards fit the DC/DC inductor (devicetree: vregmain regulator-initial-mode = DCDC).
    pac::REGULATORS.vregmain().dcdcen().write(|w| w.set_val(true));
    pac::OSCILLATORS
        .xosc32ki()
        .intcap()
        .write(|w| w.set_val(lfxo_intcap(LFXO_LOAD_FF)));
    pac::OSCILLATORS
        .xosc32m()
        .config()
        .intcap()
        .write(|w| w.set_val(hfxo_intcap(HFXO_LOAD_FF)));
}

/// Sign-extends the 9-bit two's-complement SLOPE field of the FICR crystal trims.
fn slope9(raw: u16) -> i32 {
    (i32::from(raw) << 23) >> 23
}

/// PS: `CAPVALUE = round((2C − 12)·(SLOPE + 0.765625·2⁹)/2⁹ + OFFSET/2⁶)`, C in pF.
/// Zephyr's integer form, with C in fF.
fn lfxo_intcap(load_ff: u32) -> u8 {
    let trim = pac::FICR.xosc32ktrim().read();
    let slope = (slope9(trim.slope()) + 392) as u32;
    let offset = u32::from(trim.offset());
    let mid = (2 * load_ff - 12_000) * slope + (offset << 3) * 1000;
    (mid / 512_000 + u32::from(mid % 512_000 >= 256_000)) as u8
}

/// PS: `CAPVALUE = ((C − 5.5)·(SLOPE + 791) + OFFSET·4) / 2⁸`, C in pF.
/// Zephyr's integer form, with C in fF.
fn hfxo_intcap(load_ff: u32) -> u8 {
    let trim = pac::FICR.xosc32mtrim().read();
    let slope = (slope9(trim.slope()) + 791) as u32;
    let offset = u32::from(trim.offset());
    let mid = ((load_ff - 5_500) * slope + (offset << 2) * 1000) >> 8;
    (mid / 1000 + u32::from(mid % 1000 >= 500)) as u8
}

/// TAG antenna switch: a SKY13348 driven by P1.09 (V1, antenna 0) and P1.10 (V2, antenna 1),
/// one control line per antenna, active high (NCS `multiplexing-mode = 0`).
#[cfg(feature = "board-tag")]
pub mod antenna {
    use embassy_nrf::Peri;
    use embassy_nrf::gpio::{Level, Output, OutputDrive, Pin};
    use embassy_nrf::pac;

    const PINS: [usize; 2] = [9, 10];

    /// Configures the control lines and selects antenna 0 for ordinary BLE traffic. The pins stay
    /// configured after this returns; [`switch`] drives them through the PAC.
    pub fn init(v1: Peri<'static, impl Pin>, v2: Peri<'static, impl Pin>) {
        Output::new(v1, Level::High, OutputDrive::Standard).persist();
        Output::new(v2, Level::Low, OutputDrive::Standard).persist();
    }

    /// Called by the SDC in interrupt context between CS antenna paths; the switch must settle
    /// within 6 µs. Break-before-make, like NCS `cs_antenna_switch_func`.
    pub extern "C" fn switch(antenna: u8) {
        let port = pac::P1;
        for (i, &pin) in PINS.iter().enumerate() {
            if i != usize::from(antenna) {
                port.outclr().write(|w| w.set_pin(pin, true));
            }
        }
        if let Some(&pin) = PINS.get(usize::from(antenna)) {
            port.outset().write(|w| w.set_pin(pin, true));
        }
    }
}
