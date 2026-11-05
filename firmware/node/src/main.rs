//! navgrid node firmware.
//!
//! It sets up the board (crystals, antenna switch), reads its provisioning and starts a new boot
//! epoch (see `store`), starts MPSL and the SoftDevice Controller with every Channel Sounding
//! role and the board's antennas enabled, and logs the controller's CS capabilities. Then it runs
//! the mesh (`mesh`): beacons, links to the neighbours the policy picks, ranging bursts on them,
//! telemetry toward a sink, updates over any link; a sink also serves the host over its UART.
//! Links between provisioned devices encrypt under the fleet key without pairing (`keys`). The
//! TAG's LED and button show status on demand (`ui`), and every node measures its battery
//! (`battery`).

#![no_std]
#![no_main]

mod baro;
mod battery;
mod beacon;
mod board;
mod crash;
mod crypto;
mod cs;
mod firmware;
mod fix;
mod keys;
mod logger;
mod mesh;
mod report;
mod state;
mod status;
mod store;
#[cfg(feature = "board-tag")]
mod ui;
mod update;

use bt_hci::cmd::le::LeCsReadLocalSupportedCapabilities;
use bt_hci::param::LeConnRole;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::mode::Blocking;
use embassy_nrf::rramc::Nvmc;
use embassy_nrf::{bind_interrupts, config, cracen, pac};
use embassy_time::{Duration, Timer, with_timeout};
use navgrid_proto::{Firmware, NodeConfig, NodeId, ProvisioningError};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, SoftdeviceController, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    SERIAL20 => embassy_nrf::uarte::InterruptHandler<embassy_nrf::peripherals::SERIAL20>;
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

/// Per-node link budget (architecture §5), as the link policy counts it; S5 validates it on
/// hardware.
use navgrid_link::{CENTRAL_LINKS, MAX_LINKS, PERIPHERAL_LINKS};
/// Sync, telemetry and update channels per link.
const L2CAP_CHANNELS: usize = 3 * MAX_LINKS;
const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;
use cs::CS_LINKS;
/// SDC memory pool. The TAG figure (2 antennas, 4 paths) is what its SDC reported; one antenna
/// saves 1 KiB per CS link for each of the 3 extra paths. If this is off, the SDC logs the exact
/// requirement at boot.
const SDC_MEM: usize = if board::ANTENNAS > 1 { 92_728 } else { 74_296 };

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

/// The node's identity, both on air (a static random address) and as its [`NodeId`], from FICR.
fn node_id_from_ficr() -> NodeId {
    let ficr = pac::FICR;
    NodeId::from_ficr([ficr.deviceaddr(0).read(), ficr.deviceaddr(1).read()])
}

/// This build (`build.rs` stamps the commit), for the boot log, Health reports and the sink's Hello.
const FIRMWARE: Firmware = Firmware::new(
    parse(env!("CARGO_PKG_VERSION_MAJOR"), 10) as u8,
    parse(env!("CARGO_PKG_VERSION_MINOR"), 10) as u8,
    parse(env!("CARGO_PKG_VERSION_PATCH"), 10) as u8,
    parse(env!("NAVGRID_COMMIT"), 16),
    !env!("NAVGRID_DIRTY").is_empty(),
);

/// The same as text in the image, where `navgrid flash` finds it: `navgrid-firmware 0.1.0+1950d45d`.
#[used]
static FIRMWARE_TAG: &[u8] = concat!(
    "navgrid-firmware ",
    env!("CARGO_PKG_VERSION"),
    "+",
    env!("NAVGRID_COMMIT"),
    env!("NAVGRID_DIRTY"),
    "\0"
)
.as_bytes();

/// The board, where `navgrid sign` finds it: `navgrid-board tag`.
#[used]
#[cfg(feature = "board-dk")]
static BOARD_TAG: &[u8] = b"navgrid-board dk\0";
#[used]
#[cfg(feature = "board-tag")]
static BOARD_TAG: &[u8] = b"navgrid-board tag\0";

const fn parse(s: &str, radix: u32) -> u32 {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v = 0;
    while i < b.len() {
        let digit = match b[i] {
            c @ b'0'..=b'9' => c - b'0',
            c @ b'a'..=b'f' => c - b'a' + 10,
            _ => 0,
        };
        v = v * radix + digit as u32;
        i += 1;
    }
    v
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
    info!("boot: main");
    // First, before anything can clear the reset reasons or land in the crash record.
    let (reset_reason, crash) = crash::take();
    info!(
        "boot: reset reasons {=u32:#010x}, crash record {}",
        reset_reason,
        crash.is_some()
    );
    board::early_init();
    info!("boot: board init done");
    let mut config = config::Config::default();
    config.clock_speed = config::ClockSpeed::CK128;
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    // Priority 0 belongs to MPSL's radio scheduling (mpsl.h: MPSL_HIGH_IRQ_PRIORITY).
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    let mut p = embassy_nrf::init(config);
    info!("boot: hal up");

    // RRAM writes happen here, before MPSL starts, so none lands in radio activity; MPSL's timeslot
    // driver takes the RRAM after.
    let mut rram = Nvmc::new(p.RRAMC.reborrow());
    #[cfg(feature = "board-tag")]
    let (mut led, mut button) = (ui::Led::new(p.P2_08, p.P2_10, p.P2_09), ui::button(p.P0_00));
    #[cfg(feature = "board-tag")]
    ui::factory_reset_if_held(&mut button, &mut led, &mut rram).await;
    let epoch = match store::next_epoch(&mut rram) {
        Ok(epoch) => epoch,
        Err(e) => {
            warn!("boot epoch: {:?}", e);
            0
        }
    };
    let (provisioning, corrupt) = match store::provisioning() {
        Ok(p) => (Some(p), false),
        Err(ProvisioningError::Blank) => (None, false),
        Err(e) => {
            warn!("provisioning page: {:?}; running unprovisioned", e);
            (None, true)
        }
    };
    // Before MPSL too: it writes RRAM, and may restart into an update.
    firmware::apply_staged(&mut rram, provisioning.and_then(|p| p.signing_key));
    info!("boot: image checked");
    logger::set_level(provisioning.map_or_else(Default::default, |p| p.report_level));
    let node_cfg = provisioning.map_or_else(NodeConfig::default, |p| p.config);
    let number = provisioning.and_then(|p| p.number);
    status::set_identity(provisioning.is_some(), node_cfg.sink);
    #[cfg(feature = "board-tag")]
    {
        let boot = match (provisioning.is_some(), corrupt) {
            (true, _) => ui::Colour::Green,
            (false, true) => ui::Colour::Red,
            (false, false) => ui::Colour::Blue,
        };
        spawner.spawn(unwrap!(ui::run(led, button, boot)));
    }
    #[cfg(not(feature = "board-tag"))]
    let _ = corrupt;
    spawner.spawn(unwrap!(battery::run(p.SAADC)));
    #[cfg(feature = "board-tag")]
    spawner.spawn(unwrap!(baro::run(p.SERIAL21, p.P1_12, p.P1_11)));
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
        accuracy_ppm: board::LFXO_ACCURACY_PPM,
        skip_wait_lfclk_started: false,
    };
    // One timeslot session: the RRAM driver's.
    static SESSIONS: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl: &'static MultiprotocolServiceLayer = MPSL.init(unwrap!(MultiprotocolServiceLayer::with_timeslots(
        mpsl_p,
        Irqs,
        lfclk_cfg,
        SESSIONS.init(mpsl::SessionMem::new())
    )));
    spawner.spawn(unwrap!(mpsl_task(mpsl)));
    info!("boot: mpsl up");
    let rram = firmware::share_rram(mpsl::Flash::take(mpsl, p.RRAMC));
    if let Some(crash) = &crash {
        warn!("the last run crashed: {}", crash);
    }
    let boot = navgrid_proto::Boot {
        node: node_id_from_ficr(),
        firmware: FIRMWARE,
        board: board::BOARD,
        epoch,
        reset_reason,
        // `firmware::run` reads it from the bootloader.
        image: navgrid_proto::ImageState::Confirmed,
        crash,
    };
    spawner.spawn(unwrap!(firmware::run(p.WDT0, rram, boot)));
    spawner.spawn(unwrap!(report::logs(node_id_from_ficr())));
    info!("boot: watchdog and logs tasks spawned");
    #[cfg(feature = "crash-test")]
    spawner.spawn(unwrap!(crash::test(epoch)));

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
    // Everything the mesh and the BLE host borrow lives in statics: the mesh's tasks need
    // `'static`, and a large value built on the stack and moved into place is what overflowed
    // the stack at boot (log 2026-11-04).
    static RNG: StaticCell<cracen::Cracen<'static, Blocking>> = StaticCell::new();
    let rng = RNG.init(cracen::Cracen::new_blocking(p.CRACEN));
    // The SDC keeps CRACEN; pairing gets a CSPRNG seeded from it first.
    let mut seed = [0u8; 32];
    rng.blocking_fill_bytes(&mut seed);
    crypto::seed(seed);
    static SDC_MEM_STORE: StaticCell<sdc::Mem<SDC_MEM>> = StaticCell::new();
    // SAFETY: `Mem` is uninitialised memory by definition, so nothing is constructed or copied.
    let sdc_mem = unsafe { SDC_MEM_STORE.uninit().assume_init_mut() };
    let sdc = unwrap!(build_sdc(sdc_p, rng, mpsl, sdc_mem, antennas));
    info!("boot: sdc up");
    unwrap!(cs::set_event_length(), "CS event length");
    unwrap!(cs::set_tone_length(), "CS tone length");

    let node = node_id_from_ficr();
    let addr = node.address();
    {
        // The node record: what the controller sees of this node's configuration is a hash.
        let mut cfg_bytes = [0u8; 64];
        let config = postcard::to_slice(&node_cfg, &mut cfg_bytes).map_or(0, |b| navgrid_proto::crc32(b));
        state::init(
            node,
            node_cfg.sink,
            epoch,
            navgrid_mesh::NodeInfo {
                role: node_cfg.role,
                number,
                firmware: FIRMWARE,
                config,
                hops: 0,
                frame: 0,
                pinned: node_cfg.pinned,
                neighbours: Default::default(),
            },
        );
    }
    match number {
        Some(n) => info!(
            "navgrid node N{=u16:03} {=u64:012x} on {}, firmware {}, boot {=u32}, {}",
            n,
            node.0,
            board::NAME,
            FIRMWARE,
            epoch,
            node_cfg
        ),
        None => info!(
            "navgrid node {=u64:012x} on {}, firmware {}, boot {=u32}, {}",
            node.0,
            board::NAME,
            FIRMWARE,
            epoch,
            node_cfg
        ),
    }
    let (used, available) = stack_use();
    info!(
        "ram: {=usize} B between the statics and the top of the stack, {=usize} B used so far",
        available, used
    );
    if let Some(p) = provisioning
        && (p.peer.is_some() || p.sink.is_some())
    {
        info!("provisioning names a bring-up peer or sink: the mesh picks its own links now");
    }
    if provisioning.is_none() {
        warn!("not provisioned: pairing Just Works");
    }
    static FLEET: StaticCell<keys::Fleet> = StaticCell::new();
    let fleet: &'static keys::Fleet = FLEET.init(keys::Fleet::new(addr, provisioning.map(|p| p.fleet_key)));
    state::set_controller_key(fleet.controller_key());

    // FR-M4: the beacon in the advertisement, the name in the scan response.
    static BEACONS: StaticCell<beacon::Beacons> = StaticCell::new();
    let beacons: &'static beacon::Beacons = BEACONS.init(beacon::Beacons::new(node_cfg, number));

    // CR2032: advertise slowly; NodeConfig trades target acquisition time for idle current. A
    // sink is USB-powered: fast, so nodes find it quickly. Both are settings from here on.
    status::set_adv_interval_ms(if node_cfg.sink { 100 } else { node_cfg.adv_interval_ms });
    status::set_antennas(antennas);
    // The sink writes frames to UART20 (TX P1.04, 115200 baud), which the DK's debugger exposes as
    // VCOM1, and takes the host's on its RX (P1.05). Frames from the host end at an idle line,
    // which TIMER21 times.
    #[cfg(feature = "board-dk")]
    let (uart, mut from_host) = embassy_nrf::uarte::Uarte::new(p.SERIAL20, p.P1_05, p.P1_04, Irqs, Default::default())
        .split_with_idle(p.TIMER21, p.PPI20_CH2, p.PPI20_CH3);
    #[cfg(feature = "board-dk")]
    static UART: StaticCell<report::SinkUart<'static>> = StaticCell::new();
    #[cfg(feature = "board-dk")]
    let uart: &'static report::SinkUart<'static> = UART.init(embassy_sync::mutex::Mutex::new(uart));
    #[cfg(feature = "board-dk")]
    let sink_uart = node_cfg.sink.then_some(uart);
    #[cfg(not(feature = "board-dk"))]
    let sink_uart: Option<&'static ()> = None;
    // One bond per link: the fleet keys of the links up now.
    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, MAX_LINKS, L2CAP_CHANNELS, 1, MAX_LINKS>> =
        StaticCell::new();
    // SAFETY: every field of `HostResources` is `MaybeUninit`, and `trouble_host::new` fills them.
    let resources = unsafe { RESOURCES.uninit().assume_init_mut() };
    info!("boot: state up");
    static STACK: StaticCell<Host<'static, 'static>> = StaticCell::new();
    let stack: &'static Host<'static, 'static> = STACK.init(
        trouble_host::new(sdc, resources)
            .set_random_address(Address::random(addr))
            .register_l2cap_spsm(navgrid_proto::TELEMETRY_SPSM)
            .register_l2cap_spsm(navgrid_proto::UPDATE_SPSM)
            .register_l2cap_spsm(navgrid_mesh::wire::SYNC_SPSM)
            .build(),
    );
    let mut runner = stack.runner();

    static MESH: StaticCell<mesh::Mesh<'static, 'static, 'static>> = StaticCell::new();
    let mesh: &'static mesh::Mesh<'static, 'static, 'static> = MESH.init(mesh::Mesh::new(
        stack,
        fleet,
        beacons,
        rram,
        node,
        node_cfg.role,
        node_cfg.sink,
        sink_uart,
    ));
    mesh.spawn(spawner);
    let role = async {
        log_cs_capabilities(stack).await;
        // Short ACL events leave room in each 30 ms burst interval for the other links' events
        // after the CS subevent (architecture §3). Applies to connections made from now on.
        if let Err(e) = stack
            .command(sdc::vendor::NordicEventLengthSet::new(ACL_EVENT_LENGTH_US))
            .await
        {
            warn!("ACL event length: {:?}", e);
        }
        // A sink keeps the crystal running: on the internal oscillator between radio events, the
        // UART's clock is off by enough for framing errors on the host's frames.
        let _hfxo = if node_cfg.sink {
            match mpsl.request_hfclk().await {
                Ok(hfxo) => Some(hfxo),
                Err(e) => {
                    warn!("sink: HFXO request: {:?}", e);
                    None
                }
            }
        } else {
            None
        };
        #[cfg(feature = "board-dk")]
        let sink = async {
            if node_cfg.sink {
                join(
                    update::from_host(&mut from_host, uart, rram, provisioning.and_then(|p| p.signing_key)),
                    report::own_reports(uart),
                )
                .await;
            }
        };
        #[cfg(not(feature = "board-dk"))]
        let sink = async {};
        embassy_futures::join::join3(sink, report::health(mpsl, node, FIRMWARE), state::run(mpsl)).await;
    };
    // The runner returns only on an error. The node can't receive anything after that, while its
    // controller keeps advertising and the mesh believes its links up (DEF-71), so it is fatal:
    // a crash record and a restart, which the host sees at the next boot.
    let host = async {
        if let Err(e) = runner.run_with_handler(&cs::Events).await {
            core::panic!("host stopped: {:?}", e);
        }
    };
    join(host, role).await;
}

pub(crate) type Host<'a, 'd> = Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

/// ACL event length: room for a packet pair each way, and short enough that a CS subevent ends
/// with time to spare in a 30 ms interval.
const ACL_EVENT_LENGTH_US: u32 = 2_500;

/// Whether `event` shows that the peer isn't in the fleet, on a provisioned node.
///
/// - A peripheral's fleet links encrypt through the controller's key request, never by pairing,
///   so a pairing means an outsider.
/// - A central's encryption under the fleet key fails, with `BondLost`, when the peer lacks the
///   key, and TrouBLE would fall back to pairing. TrouBLE also reports a central's encryption
///   under a stored key as `PairingComplete`, so that means nothing there.
pub(crate) fn outsider(event: &ConnectionEvent, conn: &Connection<'_, DefaultPacketPool>, fleet: &keys::Fleet) -> bool {
    fleet.provisioned()
        && match event {
            ConnectionEvent::BondLost => true,
            ConnectionEvent::PairingComplete { .. } => conn.role() == LeConnRole::Peripheral,
            _ => false,
        }
}

/// Logs a disconnection; a MIC failure means the peer encrypted under another fleet key.
pub(crate) fn disconnected(reason: bt_hci::param::Status) {
    info!("disconnected: {:?}", reason);
    if reason == bt_hci::param::Status::CONN_TERMINATED_DUE_TO_MIC_FAILURE {
        status::foreign();
    }
}

pub(crate) fn log_encrypted(security_level: SecurityLevel, fleet: &keys::Fleet) {
    let how = if fleet.provisioned() { "fleet key" } else { "Just Works" };
    info!("encrypted: {:?}, {=str}", security_level, how);
}

/// Starts encrypting a central's link. TrouBLE runs one security procedure at a time, so this
/// waits out another link's for up to 5 s. Drops the link if it can't start.
pub(crate) async fn secure(conn: &Connection<'_, DefaultPacketPool>) -> bool {
    for _ in 0..50 {
        match conn.request_security() {
            Ok(()) => return true,
            Err(trouble_host::Error::InvalidState) => Timer::after(Duration::from_millis(100)).await,
            Err(e) => {
                warn!("request security: {:?}", e);
                break;
            }
        }
    }
    warn!("request security: gave up");
    conn.disconnect();
    false
}

/// Waits up to 10 s for the link to be encrypted. Returns `false`, dropping the link, if it isn't
/// by then, the peer isn't in the fleet ([`outsider`]), or the link drops.
pub(crate) async fn encrypted(
    stack: &Host<'_, '_>,
    conn: &Connection<'_, DefaultPacketPool>,
    fleet: &keys::Fleet,
) -> bool {
    match with_timeout(Duration::from_secs(10), wait_encrypted(stack, conn, fleet)).await {
        Ok(encrypted) => encrypted,
        Err(_) => {
            warn!("not encrypted within 10 s: dropping the link");
            conn.disconnect();
            false
        }
    }
}

async fn wait_encrypted(stack: &Host<'_, '_>, conn: &Connection<'_, DefaultPacketPool>, fleet: &keys::Fleet) -> bool {
    loop {
        let event = conn.next().await;
        if outsider(&event, conn, fleet) {
            warn!("peer outside the fleet: dropping it");
            status::foreign();
            conn.disconnect();
            continue;
        }
        match event {
            ConnectionEvent::Encrypted { security_level, .. } => {
                log_encrypted(security_level, fleet);
                return true;
            }
            ConnectionEvent::PairingFailed(e) => {
                warn!("pairing failed: {:?}", e);
                conn.disconnect();
            }
            ConnectionEvent::Disconnected { reason } => {
                disconnected(reason);
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

unsafe extern "C" {
    static _stack_start: u32;
    static _stack_end: u32;
}

/// What the stack area is painted with before anything runs, so its high-water mark can be
/// read ([`stack_use`]).
const STACK_PAINT: u32 = 0xdead_beef;
/// The stack limit sits this far above the end of the statics, so the fault handler has room to
/// run when the limit is hit.
const STACK_GUARD: usize = 512;

// Before RAM is initialised, on the fresh stack: paints the stack area, and sets the stack limit
// register so an overflow faults at once (DEF-68). Without it the overflow silently overwrote the
// radio stack's statics just below the stack, and the node crashed through a corrupted callback
// pointer at boot (log 2026-11-04).
core::arch::global_asm!(
    ".section .text.__pre_init, \"ax\"",
    ".global __pre_init",
    ".type __pre_init, %function",
    ".thumb_func",
    "__pre_init:",
    "    ldr r0, ={end}",
    "    mov r1, sp",
    "    sub r1, r1, #256",
    "    ldr r2, ={paint}",
    "1:",
    "    cmp r0, r1",
    "    bhs 2f",
    "    str r2, [r0], #4",
    "    b 1b",
    "2:",
    "    ldr r0, ={end}",
    "    add r0, r0, #{guard}",
    "    msr msplim, r0",
    "    bx lr",
    ".ltorg",
    end = sym _stack_end,
    paint = const STACK_PAINT,
    guard = const STACK_GUARD,
);

/// Bytes of stack used at most since boot, and bytes available: from the end of the statics to
/// the top of RAM. The statics take most of the part (spike 2026-11-04-firmware-ram-budget), so
/// this is watched and logged.
pub fn stack_use() -> (usize, usize) {
    let end = core::ptr::addr_of!(_stack_end) as usize;
    let top = core::ptr::addr_of!(_stack_start) as usize;
    let mut p = end;
    // SAFETY: reading the stack area as plain words.
    while p < top && unsafe { (p as *const u32).read_volatile() } == STACK_PAINT {
        p += 4;
    }
    (top - p, top - end)
}
