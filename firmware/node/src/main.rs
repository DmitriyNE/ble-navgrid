//! navgrid node firmware.
//!
//! Bring-up firmware (phase P0). It sets up the board (crystals, antenna switch), starts
//! MPSL and the SoftDevice Controller with every Channel Sounding role and the board's antennas
//! enabled, and logs the controller's CS capabilities. Then, depending on the build-time role
//! (see `cs`), it either advertises and reflects, or connects to a reflector, pairs, and runs CS
//! procedures. Both sides log raw CS step data over RTT.

#![no_std]
#![no_main]

mod board;
mod crypto;
mod cs;

use bt_hci::cmd::le::LeCsReadLocalSupportedCapabilities;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::mode::Blocking;
use embassy_nrf::{bind_interrupts, config, cracen, pac};
use embassy_time::Duration;
use navgrid_proto::{NodeConfig, NodeId};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, SoftdeviceController, mpsl};
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
/// SDC memory pool. The TAG figure (2 antennas, 4 paths) is what its SDC reported; one antenna
/// saves 1 KiB per CS link for each of the 3 extra paths. If this is off, the SDC logs the exact
/// requirement at boot.
const SDC_MEM: usize = if board::ANTENNAS > 1 { 93_840 } else { 75_408 };

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
    // Order matters (found on hardware): the SDC sizes CS memory only once CS support is on, and
    // rejects more than one antenna (EINVAL) until the switch callback is set.
    let builder = sdc::Builder::new()?
        .support_adv()
        .support_ext_adv()
        .support_peripheral()
        .support_central()
        .support_scan()
        .support_ext_scan()
        .support_ext_central()
        .support_le_2m_phy()
        .support_channel_sounding_initiator_role_central()
        .support_channel_sounding_initiator_role_peripheral()
        .support_channel_sounding_reflector_role_central()
        .support_channel_sounding_reflector_role_peripheral()
        .support_channel_sounding_mode3();
    let builder = match board::ANTENNA_SWITCH {
        Some(switch) if antennas > 1 => builder.cs_antenna_switch_callback(switch),
        _ => builder,
    };
    // With diversity, a link has up to local × remote antennas paths (4 between two TAGs).
    // Without it, one path, whatever the peer has (NodeConfig::diversity).
    builder
        .cs_count(CS_LINKS)?
        .cs_cfg(if antennas > 1 { 4 } else { 1 }, antennas)?
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

/// Runs once the host is up: the controller disallows CS commands until the host has announced
/// Channel Sounding support, which the runner does while initialising (`stack.command` waits).
async fn log_cs_capabilities(stack: &Host<'_, '_>) {
    let caps = match stack.command(LeCsReadLocalSupportedCapabilities::new()).await {
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
    board::early_init();
    let mut config = config::Config::default();
    config.clock_speed = config::ClockSpeed::CK128;
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    // Priority 0 belongs to MPSL's radio scheduling (mpsl.h: MPSL_HIGH_IRQ_PRIORITY).
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
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
    // The SDC keeps CRACEN; pairing gets a CSPRNG seeded from it first.
    let mut seed = [0u8; 32];
    rng.blocking_fill_bytes(&mut seed);
    crypto::seed(seed);
    let mut sdc_mem = sdc::Mem::<SDC_MEM>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem, antennas));

    let addr = static_address();
    let mut id = [0u8; 8];
    id[..6].copy_from_slice(&addr);
    let node = NodeId(u64::from_le_bytes(id));
    info!("navgrid node {=u64:012x} on {}, {}", node.0, board::NAME, node_cfg);

    let mut resources: HostResources<DefaultPacketPool, MAX_LINKS, L2CAP_CHANNELS> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(addr))
        .build();
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
    let peer = cs::is_initiator().then(|| {
        unwrap!(
            cs::peer(),
            "NAVGRID_PEER must be the reflector's node ID (12 hex digits)"
        )
    });
    let role = async {
        log_cs_capabilities(&stack).await;
        match peer {
            Some(peer) => initiator(&stack, peer).await,
            None => reflector(&stack, &adv_data[..adv_len], &adv_params).await,
        }
    };
    let _ = join(runner.run_with_handler(&cs::Events), role).await;
}

type Host<'a, 'd> = Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

/// Advertises, accepts a connection, enables the CS reflector role on it, and serves it until it
/// drops. Repeats forever.
async fn reflector(stack: &Host<'_, '_>, adv_data: &[u8], params: &AdvertisementParameters) {
    let mut peripheral = stack.peripheral();
    loop {
        let advertiser = unwrap!(
            peripheral
                .advertise(
                    params,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data,
                        scan_data: &[]
                    },
                )
                .await
        );
        info!("advertising");
        let conn = unwrap!(advertiser.accept().await);
        info!("connected: handle {}", conn.handle().raw());
        if let Err(e) = cs::reflect(stack, conn.handle()).await {
            warn!("CS: {}", e);
        }
        serve(stack, &conn).await;
    }
}

/// Connects to `peer`, pairs, runs the CS initiator sequence, and serves the link until it drops.
/// Repeats forever.
async fn initiator(stack: &Host<'_, '_>, peer: [u8; 6]) {
    let mut central = stack.central();
    let target = Address::random(peer);
    // Nordic's sample connects at 20 ms with a 4 s supervision timeout.
    let config = ConnectConfig {
        connect_params: RequestedConnParams {
            min_connection_interval: Duration::from_millis(20),
            max_connection_interval: Duration::from_millis(20),
            supervision_timeout: Duration::from_secs(4),
            ..Default::default()
        },
        scan_config: ScanConfig {
            filter_accept_list: &[target],
            ..Default::default()
        },
    };
    loop {
        info!("connecting to {=[u8]:02x}", peer);
        // Extended Create Connection: the legacy command trips SDC assert 50:701 on the nRF54L15 as
        // soon as the peer's advertisement is heard (DEF-17).
        let conn = match central.connect_ext(&config).await {
            Ok(conn) => conn,
            Err(e) => {
                warn!("connect: {:?}", e);
                continue;
            }
        };
        info!("connected: handle {}", conn.handle().raw());
        if let Err(e) = conn.request_security() {
            warn!("request security: {:?}", e);
        }
        if !encrypted(stack, &conn).await {
            continue;
        }
        match cs::initiate(stack, conn.handle()).await {
            Ok(()) => info!("CS running"),
            Err(e) => warn!("CS: {}", e),
        }
        serve(stack, &conn).await;
    }
}

/// Waits until the link is encrypted. Returns `false` if pairing fails or the link drops.
async fn encrypted(stack: &Host<'_, '_>, conn: &Connection<'_, DefaultPacketPool>) -> bool {
    loop {
        match conn.next().await {
            ConnectionEvent::PairingComplete { security_level, .. } => {
                info!("paired: {:?}", security_level);
                return true;
            }
            ConnectionEvent::Encrypted { security_level, .. } => {
                info!("encrypted: {:?}", security_level);
                return true;
            }
            ConnectionEvent::PairingFailed(e) => {
                warn!("pairing failed: {:?}", e);
                conn.disconnect();
            }
            ConnectionEvent::Disconnected { reason } => {
                info!("disconnected: {:?}", reason);
                return false;
            }
            ConnectionEvent::RequestConnectionParams(req) => {
                if let Err(e) = req.accept(None, stack).await {
                    warn!("connection parameters: {:?}", e);
                }
            }
            _ => {}
        }
    }
}

/// Handles connection events until the link drops.
async fn serve(stack: &Host<'_, '_>, conn: &Connection<'_, DefaultPacketPool>) {
    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("disconnected: {:?}", reason);
                return;
            }
            ConnectionEvent::PairingComplete { security_level, .. } => info!("paired: {:?}", security_level),
            ConnectionEvent::RequestConnectionParams(req) => {
                if let Err(e) = req.accept(None, stack).await {
                    warn!("connection parameters: {:?}", e);
                }
            }
            _ => {}
        }
    }
}
