//! navgrid node firmware.
//!
//! Bring-up skeleton (phase P0). It sets up the board (DC/DC, crystals, antenna switch), starts
//! MPSL and the SoftDevice Controller with every Channel Sounding role and the board's antennas
//! enabled, logs the controller's CS capabilities, then advertises as a connectable navgrid node
//! through TrouBLE. Ranging comes next (architecture §6, P-1..P-3).

#![no_std]
#![no_main]

mod board;

use bt_hci::cmd::SyncCmd;
use bt_hci::cmd::le::LeCsReadLocalSupportedCapabilities;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::mode::Blocking;
use embassy_nrf::{bind_interrupts, config, cracen, pac};
use embassy_time::Duration;
use navgrid_proto::{NodeConfig, NodeId};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

/// Per-node link budget (architecture §5); P0 validates it on hardware.
const CENTRAL_LINKS: u8 = 6;
const PERIPHERAL_LINKS: u8 = 8;
const MAX_LINKS: usize = (CENTRAL_LINKS + PERIPHERAL_LINKS) as usize;
/// Signal + ATT per link.
const L2CAP_CHANNELS: usize = 2 * MAX_LINKS;
const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;
/// Links that can hold a CS configuration at once. Each costs ~4–7 KB of SDC memory depending
/// on antenna paths (`SDC_MEM_CS` in sdc.h).
const CS_LINKS: u8 = 6;
/// SDC memory pool. If this is off, the SDC logs the exact requirement at boot.
const SDC_MEM: usize = 112 * 1024;

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn build_sdc<'d, const N: usize>(
    p: sdc::Peripherals<'d>,
    rng: &'d mut cracen::Cracen<'static, Blocking>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
    antennas: u8,
) -> Result<sdc::SoftdeviceController<'d>, sdc::Error> {
    // With diversity, a link has up to local × remote antennas paths (4 between two TAGs).
    // Without it, one path, whatever the peer has (NodeConfig::diversity).
    let builder = sdc::Builder::new()?
        .cs_count(CS_LINKS)?
        .cs_cfg(if antennas > 1 { 4 } else { 1 }, antennas)?;
    let builder = match board::ANTENNA_SWITCH {
        Some(switch) if antennas > 1 => builder.cs_antenna_switch_callback(switch),
        _ => builder,
    };
    builder
        .support_adv()
        .support_ext_adv()
        .support_peripheral()
        .support_central()
        .support_scan()
        .support_le_2m_phy()
        .support_channel_sounding_initiator_role_central()
        .support_channel_sounding_initiator_role_peripheral()
        .support_channel_sounding_reflector_role_central()
        .support_channel_sounding_reflector_role_peripheral()
        .support_channel_sounding_mode3()
        .central_count(CENTRAL_LINKS)?
        .peripheral_count(PERIPHERAL_LINKS)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

/// Static random address derived from FICR. It is the node's identity both on air and as [`NodeId`].
fn static_address() -> [u8; 6] {
    let ficr = pac::FICR;
    let raw = (u64::from(ficr.deviceaddr(1).read()) << 32) | u64::from(ficr.deviceaddr(0).read());
    let mut addr = [0u8; 6];
    addr.copy_from_slice(&raw.to_le_bytes()[..6]);
    addr[5] |= 0xc0; // static random addresses have both MSBs set
    addr
}

async fn log_cs_capabilities(sdc: &sdc::SoftdeviceController<'_>) {
    let caps = match LeCsReadLocalSupportedCapabilities::new().exec(sdc).await {
        Ok(caps) => caps,
        Err(e) => {
            warn!("CS capabilities: {:?}", e);
            return;
        }
    };
    info!(
        "CS: configs={} antennas={} paths={} roles={=u8:#04x} modes={=u8:#04x} rtt={=u8:#04x} sync_phys={=u8:#04x}",
        caps.num_config_supported,
        caps.num_antennas_supported,
        caps.max_antenna_paths_supported,
        caps.roles_supported,
        caps.optional_modes_supported,
        caps.rtt_capability,
        caps.cs_sync_phys_supported,
    );
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    board::init_power_and_clocks();
    let mut config = config::Config::default();
    config.clock_speed = config::ClockSpeed::CK128;
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    // Until provisioning (FR-O1) stores one in flash.
    let node_cfg = NodeConfig::default();
    let antennas = if node_cfg.diversity { board::ANTENNAS } else { 1 };
    #[cfg(feature = "board-tag")]
    board::antenna::init(p.P1_09, p.P1_10);

    let mpsl_p = mpsl::Peripherals::new(
        p.GRTC_CH7,
        p.GRTC_CH8,
        p.GRTC_CH9,
        p.GRTC_CH10,
        p.GRTC_CH11,
        p.TIMER10,
        p.TIMER20,
        p.TEMP,
        p.PPI10_CH0,
        p.PPI20_CH1,
        p.PPIB11_CH0,
        p.PPIB21_CH0,
    );
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 50,
        skip_wait_lfclk_started: false,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    let sdc_p = sdc::Peripherals::new(
        p.PPI00_CH1,
        p.PPI00_CH3,
        p.PPI10_CH1,
        p.PPI10_CH2,
        p.PPI10_CH3,
        p.PPI10_CH4,
        p.PPI10_CH5,
        p.PPI10_CH6,
        p.PPI10_CH7,
        p.PPI10_CH8,
        p.PPI10_CH9,
        p.PPI10_CH10,
        p.PPI10_CH11,
        p.PPIB00_CH1,
        p.PPIB00_CH2,
        p.PPIB00_CH3,
        p.PPIB10_CH1,
        p.PPIB10_CH2,
        p.PPIB10_CH3,
    );
    let mut rng = cracen::Cracen::new_blocking(p.CRACEN);
    let mut sdc_mem = sdc::Mem::<SDC_MEM>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem, antennas));

    let addr = static_address();
    let mut id = [0u8; 8];
    id[..6].copy_from_slice(&addr);
    let node = NodeId(u64::from_le_bytes(id));
    info!("navgrid node {=u64:012x} on {}, {}", node.0, board::NAME, node_cfg);
    log_cs_capabilities(&sdc).await;

    let mut resources: HostResources<DefaultPacketPool, MAX_LINKS, L2CAP_CHANNELS> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(addr))
        .build();
    let mut peripheral = stack.peripheral();
    let mut runner = stack.runner();

    let mut adv_data = [0; 31];
    let adv_len = unwrap!(AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"navgrid"),
        ],
        &mut adv_data[..],
    ));

    // CR2032: advertise slowly; NodeConfig trades target acquisition time for idle current.
    let adv_interval = Duration::from_millis(u64::from(node_cfg.adv_interval_ms));
    let adv_params = AdvertisementParameters {
        interval_min: adv_interval,
        interval_max: adv_interval,
        ..Default::default()
    };
    let _ = join(runner.run(), async {
        loop {
            let advertiser = unwrap!(
                peripheral
                    .advertise(
                        &adv_params,
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &adv_data[..adv_len],
                            scan_data: &[]
                        },
                    )
                    .await
            );
            info!("advertising");
            let conn = unwrap!(advertiser.accept().await);
            info!("connected: handle {}", conn.handle().raw());
            loop {
                if let ConnectionEvent::Disconnected { reason } = conn.next().await {
                    info!("disconnected: {:?}", reason);
                    break;
                }
            }
        }
    })
    .await;
}
