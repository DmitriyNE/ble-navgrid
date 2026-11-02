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
mod report;

use core::cell::Cell;

use bt_hci::cmd::le::LeCsReadLocalSupportedCapabilities;
use defmt::{error, info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::mode::Blocking;
use embassy_nrf::{bind_interrupts, config, cracen, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, with_timeout};
use navgrid_proto::{NodeConfig, NodeId};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, SoftdeviceController, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL20 => embassy_nrf::uarte::InterruptHandler<embassy_nrf::peripherals::SERIAL20>;
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
const SDC_MEM: usize = if board::ANTENNAS > 1 { 99_072 } else { 80_640 };

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
        // As NCS enables them; power control lets CS run on a different PHY than the ACL.
        .support_phy_update_central()
        .support_phy_update_peripheral()
        .support_dle_central()
        .support_dle_peripheral()
        .support_le_power_control_central()
        .support_le_power_control_peripheral()
        // Inline PCT transfer needs the extended feature set, as NCS's IPT samples enable it.
        .support_extended_feature_set_central()
        .support_extended_feature_set_peripheral()
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
    unwrap!(cs::set_event_length(), "CS event length");
    unwrap!(cs::set_tone_length(), "CS tone length");

    let addr = static_address();
    let mut id = [0u8; 8];
    id[..6].copy_from_slice(&addr);
    let node = NodeId(u64::from_le_bytes(id));
    info!("navgrid node {=u64:012x} on {}, {}", node.0, board::NAME, node_cfg);

    let mut resources: HostResources<DefaultPacketPool, MAX_LINKS, L2CAP_CHANNELS> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(addr))
        .register_l2cap_spsm(navgrid_proto::TELEMETRY_SPSM)
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
    // The sink writes frames to UART20 (P1.04, 115200 baud), which the DK's debugger exposes as
    // VCOM1.
    #[cfg(feature = "board-dk")]
    let uart = embassy_nrf::uarte::UarteTx::new(p.SERIAL20, Irqs, p.P1_04, Default::default());
    let role = async {
        log_cs_capabilities(&stack).await;
        #[cfg(feature = "board-dk")]
        if cs::is_sink() {
            // USB-powered: advertise fast so nodes find it quickly.
            let sink_params = AdvertisementParameters {
                interval_min: Duration::from_millis(100),
                interval_max: Duration::from_millis(100),
                ..Default::default()
            };
            report::serve_sink(&stack, uart, node, &adv_data[..adv_len], &sink_params).await;
        }
        match peer {
            Some(peer) => initiator(&stack, peer, antennas, node).await,
            None => reflector(&stack, &adv_data[..adv_len], &adv_params).await,
        }
    };
    // The runner returns only on an error, e.g. an HCI event bt-hci can't parse; everything stops
    // then, so say why.
    let host = async {
        if let Err(e) = runner.run_with_handler(&cs::Events).await {
            error!("host stopped: {:?}", e);
        }
    };
    join(host, role).await;
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
        cs::exchange_features(stack, conn.handle()).await;
        if let Err(e) = cs::reflect(stack, conn.handle()).await {
            warn!("CS: {}", e);
        }
        serve(stack, &conn).await;
    }
}

/// Ranges with `peer` forever and, with `NAVGRID_SINK` set, keeps a telemetry uplink to the sink
/// alongside. The two share the central role, taking it only while connecting.
async fn initiator(stack: &Host<'_, '_>, peer: [u8; 6], antennas: u8, node: NodeId) {
    // Short ACL events leave room in each 30 ms CS-link interval for the sink link's event after
    // the CS subevent (see `Links`). Applies to connections made from now on.
    if let Err(e) = stack
        .command(sdc::vendor::NordicEventLengthSet::new(ACL_EVENT_LENGTH_US))
        .await
    {
        warn!("ACL event length: {:?}", e);
    }
    let central = Mutex::<CriticalSectionRawMutex, _>::new(stack.central());
    let links = Links::default();
    let report = async {
        let Some(sink) = report::sink() else {
            return;
        };
        loop {
            let uplink = {
                let mut central = central.lock().await;
                links.space_before(stack, Link::Sink).await;
                uplink(stack, &mut central, sink).await
            };
            if let Some((_conn, mut channel)) = uplink {
                links.sink.set(true);
                report::set_link(node, node_id(peer));
                report::run(stack, &mut channel).await;
                links.sink.set(false);
            }
            Timer::after(Duration::from_secs(5)).await;
        }
    };
    join(ranging(stack, &central, &links, peer, antennas), report).await;
}

/// ACL event length on the initiator: room for a packet pair each way, and short enough that the
/// CS subevent ends with time to spare in a 30 ms interval.
const ACL_EVENT_LENGTH_US: u32 = 2_500;
/// From a CS-link anchor to the end of its longest CS subevent: the ACL event, the CS offset past
/// it with 0.5 ms of set-up, 22 ms of subevent, and 1 ms of margin.
const CS_FOOTPRINT_US: u32 = 26_000;
/// The sink link's interval: a multiple of the CS link's 30 ms keeps its events at a fixed offset
/// from the CS link's, so they never drift onto a CS subevent.
const SINK_INTERVAL_MS: u64 = 120;

#[derive(Clone, Copy)]
enum Link {
    Cs,
    Sink,
}

/// Which of the initiator's two central links are up. The SDC places a new central link the
/// configured spacing after the running one, so the spacing is set just before each connection:
/// the sink link goes after the CS link's footprint, the CS link right after the sink link's short
/// event, whichever is second (DEF-26). Fixed offsets need the common 30 ms factor.
#[derive(Default)]
struct Links {
    cs: Cell<bool>,
    sink: Cell<bool>,
}

impl Links {
    async fn space_before(&self, stack: &Host<'_, '_>, new: Link) {
        let spacing = match new {
            Link::Sink if self.cs.get() => CS_FOOTPRINT_US,
            Link::Cs if self.sink.get() => ACL_EVENT_LENGTH_US,
            // First link: nothing to space from.
            _ => return,
        };
        if let Err(e) = stack
            .command(sdc::vendor::NordicCentralAclEventSpacingSet::new(spacing))
            .await
        {
            warn!("central ACL event spacing: {:?}", e);
        }
    }
}

/// The node ID of a device address: the address in the low six bytes.
fn node_id(addr: [u8; 6]) -> NodeId {
    let mut id = [0u8; 8];
    id[..6].copy_from_slice(&addr);
    NodeId(u64::from_le_bytes(id))
}

/// Connects to the sink and opens the telemetry channel. Gives up after 10 s, so a missing sink
/// doesn't hold up ranging. The caller spaces it from the CS link first (see `Links`).
async fn uplink<'a>(
    stack: &'a Host<'_, '_>,
    central: &mut Central<'a, SoftdeviceController<'_>, DefaultPacketPool>,
    sink: [u8; 6],
) -> Option<(Connection<'a, DefaultPacketPool>, L2capChannel<'a, DefaultPacketPool>)> {
    let target = Address::random(sink);
    let config = ConnectConfig {
        connect_params: RequestedConnParams {
            min_connection_interval: Duration::from_millis(SINK_INTERVAL_MS),
            max_connection_interval: Duration::from_millis(SINK_INTERVAL_MS),
            supervision_timeout: Duration::from_secs(4),
            ..Default::default()
        },
        scan_config: ScanConfig {
            filter_accept_list: &[target],
            ..Default::default()
        },
    };
    info!("uplink: connecting to sink {=[u8]:02x}", sink);
    let conn = match with_timeout(Duration::from_secs(10), central.connect_ext(&config)).await {
        Ok(Ok(conn)) => conn,
        Ok(Err(e)) => {
            warn!("uplink: connect: {:?}", e);
            return None;
        }
        Err(_) => {
            warn!("uplink: no sink within 10 s, ranging without it");
            return None;
        }
    };
    match L2capChannel::create(stack, &conn, navgrid_proto::TELEMETRY_SPSM, &Default::default()).await {
        Ok(channel) => {
            info!("uplink: channel open");
            Some((conn, channel))
        }
        Err(e) => {
            warn!("uplink: channel: {:?}", e);
            None
        }
    }
}

/// Connects to `peer`, pairs, runs the CS initiator sequence, and serves the link until it drops.
/// Repeats forever.
async fn ranging(
    stack: &Host<'_, '_>,
    central: &Mutex<CriticalSectionRawMutex, Central<'_, SoftdeviceController<'_>, DefaultPacketPool>>,
    links: &Links,
    peer: [u8; 6],
    antennas: u8,
) {
    let target = Address::random(peer);
    // A 4 s supervision timeout, as Nordic's sample uses.
    let config = ConnectConfig {
        connect_params: RequestedConnParams {
            min_connection_interval: Duration::from_millis(cs::ACL_INTERVAL_MS),
            max_connection_interval: Duration::from_millis(cs::ACL_INTERVAL_MS),
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
        let connected = {
            let mut central = central.lock().await;
            links.space_before(stack, Link::Cs).await;
            central.connect_ext(&config).await
        };
        let conn = match connected {
            Ok(conn) => conn,
            Err(e) => {
                warn!("connect: {:?}", e);
                continue;
            }
        };
        links.cs.set(true);
        info!("connected: handle {}", conn.handle().raw());
        cs::exchange_features(stack, conn.handle()).await;
        // Zephyr's central moves the ACL to 2M on connect (BT_AUTO_PHY_UPDATE); CS runs on 2M too.
        if let Err(e) = conn.set_phy(stack, PhyKind::Le2M).await {
            warn!("set PHY: {:?}", e);
        }
        // And to the largest data length (BT_AUTO_DATA_LEN_UPDATE): 251 octets, 2120 us.
        if let Err(e) = conn.update_data_length(stack, 251, 2120).await {
            warn!("data length: {:?}", e);
        }
        if let Err(e) = conn.request_security() {
            warn!("request security: {:?}", e);
        }
        if !encrypted(stack, &conn).await {
            links.cs.set(false);
            continue;
        }
        match cs::initiate(stack, conn.handle(), antennas).await {
            Ok(()) => info!("CS running"),
            Err(e) => warn!("CS: {}", e),
        }
        serve(stack, &conn).await;
        links.cs.set(false);
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
            ConnectionEvent::DataLengthUpdated {
                max_tx_octets,
                max_tx_time,
                max_rx_octets,
                max_rx_time,
            } => {
                info!(
                    "data length: tx {} B {} us, rx {} B {} us",
                    max_tx_octets, max_tx_time, max_rx_octets, max_rx_time
                )
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
            ConnectionEvent::DataLengthUpdated {
                max_tx_octets,
                max_tx_time,
                max_rx_octets,
                max_rx_time,
            } => {
                info!(
                    "data length: tx {} B {} us, rx {} B {} us",
                    max_tx_octets, max_tx_time, max_rx_octets, max_rx_time
                )
            }
            _ => {}
        }
    }
}
