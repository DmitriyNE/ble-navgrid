//! navgrid node firmware.
//!
//! Bring-up firmware (phase P0). It sets up the board (crystals, antenna switch), reads its
//! provisioning and starts a new boot epoch (see `store`), starts MPSL and the SoftDevice
//! Controller with every Channel Sounding role and the board's antennas enabled, and logs the
//! controller's CS capabilities. Then, as provisioned, it advertises and reflects, connects to a
//! reflector and runs CS procedures, or serves as the sink. Links between provisioned devices
//! encrypt under the fleet key without pairing (`keys`). The TAG's LED and button show status on
//! demand (`ui`), and every node measures its battery (`battery`).

#![no_std]
#![no_main]

mod battery;
mod beacon;
mod board;
mod crash;
mod crypto;
mod cs;
mod firmware;
mod keys;
mod logger;
mod report;
mod status;
mod store;
#[cfg(feature = "board-tag")]
mod ui;
mod update;

use core::cell::Cell;

use bt_hci::cmd::le::LeCsReadLocalSupportedCapabilities;
use bt_hci::param::LeConnRole;
use defmt::{error, info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_nrf::mode::Blocking;
use embassy_nrf::rramc::Nvmc;
use embassy_nrf::{bind_interrupts, config, cracen, pac};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use navgrid_proto::{Firmware, NodeConfig, NodeId, ProvisioningError, Role};
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
    // First, before anything can clear the reset reasons or land in the crash record.
    let (reset_reason, crash) = crash::take();
    board::early_init();
    let mut config = config::Config::default();
    config.clock_speed = config::ClockSpeed::CK128;
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    // Priority 0 belongs to MPSL's radio scheduling (mpsl.h: MPSL_HIGH_IRQ_PRIORITY).
    config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    let mut p = embassy_nrf::init(config);

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
    logger::set_level(provisioning.map_or_else(Default::default, |p| p.report_level));
    let node_cfg = provisioning.map_or_else(NodeConfig::default, |p| p.config);
    let number = provisioning.and_then(|p| p.number);
    status::set_identity(provisioning.is_some(), node_cfg.role == Role::Sink);
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
    let mut rng = cracen::Cracen::new_blocking(p.CRACEN);
    // The SDC keeps CRACEN; pairing gets a CSPRNG seeded from it first.
    let mut seed = [0u8; 32];
    rng.blocking_fill_bytes(&mut seed);
    crypto::seed(seed);
    let mut sdc_mem = sdc::Mem::<SDC_MEM>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem, antennas));
    unwrap!(cs::set_event_length(), "CS event length");
    unwrap!(cs::set_tone_length(), "CS tone length");

    let node = node_id_from_ficr();
    let addr = node.address();
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
    let peer = provisioning.and_then(|p| p.peer);
    let sink = provisioning.and_then(|p| p.sink);
    match provisioning {
        Some(_) => info!("provisioned: peer {}, sink {}", peer, sink),
        None => warn!("not provisioned: reflecting, and pairing Just Works"),
    }
    let fleet = keys::Fleet::new(addr, provisioning.map(|p| p.fleet_key));

    // One bond per link: the fleet keys of the links up now.
    let mut resources: HostResources<DefaultPacketPool, MAX_LINKS, L2CAP_CHANNELS, 1, MAX_LINKS> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(addr))
        .register_l2cap_spsm(navgrid_proto::TELEMETRY_SPSM)
        .register_l2cap_spsm(navgrid_proto::UPDATE_SPSM)
        .build();
    let mut runner = stack.runner();

    // FR-M4: the beacon in the advertisement, the name in the scan response.
    let beacons = beacon::Beacons::new(node_cfg, number, MAX_LINKS as u8);

    // CR2032: advertise slowly; NodeConfig trades target acquisition time for idle current.
    let adv_interval = Duration::from_millis(u64::from(node_cfg.adv_interval_ms));
    let adv_params = AdvertisementParameters {
        interval_min: adv_interval,
        interval_max: adv_interval,
        ..Default::default()
    };
    if peer.is_some() {
        cs::set_initiator();
    }
    // The sink writes frames to UART20 (TX P1.04, 115200 baud), which the DK's debugger exposes as
    // VCOM1, and takes the host's on its RX (P1.05). Frames from the host end at an idle line,
    // which TIMER21 times.
    #[cfg(feature = "board-dk")]
    let (uart, mut from_host) = embassy_nrf::uarte::Uarte::new(p.SERIAL20, p.P1_05, p.P1_04, Irqs, Default::default())
        .split_with_idle(p.TIMER21, p.PPI20_CH2, p.PPI20_CH3);
    #[cfg(feature = "board-dk")]
    let uart = Mutex::new(uart);
    let role = async {
        log_cs_capabilities(&stack).await;
        #[cfg(feature = "board-dk")]
        if node_cfg.role == Role::Sink {
            // USB-powered: advertise fast so nodes find it quickly.
            let sink_params = AdvertisementParameters {
                interval_min: Duration::from_millis(100),
                interval_max: Duration::from_millis(100),
                ..Default::default()
            };
            // And keep the crystal running: on the internal oscillator between radio events, the
            // UART's clock is off by enough for framing errors on the host's frames.
            let _hfxo = match mpsl.request_hfclk().await {
                Ok(hfxo) => Some(hfxo),
                Err(e) => {
                    warn!("sink: HFXO request: {:?}", e);
                    None
                }
            };
            embassy_futures::join::join3(
                report::serve_sink(&stack, &fleet, &uart, node, &beacons, &sink_params, rram),
                update::from_host(&mut from_host, &uart, rram, provisioning.and_then(|p| p.signing_key)),
                report::own_reports(&uart),
            )
            .await;
        }
        match peer {
            Some(peer) => {
                let ranging = initiator(
                    &stack,
                    &fleet,
                    peer.address(),
                    sink.map(NodeId::address),
                    antennas,
                    node,
                    rram,
                );
                join(ranging, report::health(mpsl, node, FIRMWARE)).await;
            }
            None => reflector(&stack, &fleet, &beacons, &adv_params, rram).await,
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
async fn reflector(
    stack: &Host<'_, '_>,
    fleet: &keys::Fleet,
    beacons: &beacon::Beacons,
    params: &AdvertisementParameters,
    rram: &'static firmware::Rram,
) {
    let mut peripheral = stack.peripheral();
    loop {
        info!("advertising");
        let conn = beacons.advertise(&mut peripheral, params).await;
        info!("connected: handle {}", conn.handle().raw());
        fleet.trust(stack, &conn);
        cs::exchange_features(stack, conn.handle()).await;
        if let Err(e) = cs::reflect(stack, conn.handle()).await {
            warn!("CS: {}", e);
        }
        select(serve(stack, &conn, fleet), update::listen(stack, &conn, rram, false)).await;
    }
}

/// Ranges with `peer` forever and, given a sink, keeps a telemetry uplink to it alongside. The two
/// share the central role, taking it only while connecting.
async fn initiator(
    stack: &Host<'_, '_>,
    fleet: &keys::Fleet,
    peer: [u8; 6],
    sink: Option<[u8; 6]>,
    antennas: u8,
    node: NodeId,
    rram: &'static firmware::Rram,
) {
    // Short ACL events leave room in each 30 ms CS-link interval for the sink link's event after
    // the CS subevent (see `Links`). Applies to connections made from now on.
    if let Err(e) = stack
        .command(sdc::vendor::NordicEventLengthSet::new(ACL_EVENT_LENGTH_US))
        .await
    {
        warn!("ACL event length: {:?}", e);
    }
    let mut central = stack.central();
    listen(&mut central).await;
    let central = Mutex::<CriticalSectionRawMutex, _>::new(central);
    let links = Links::default();
    let report = async {
        let Some(sink) = sink else {
            return;
        };
        loop {
            if let Some((conn, mut channel)) = uplink(stack, fleet, &central, &links, sink).await {
                let _link = status::Link::up(true);
                report::set_link(node, node_id(peer));
                // The sink offers the update it carries for this board.
                select(
                    report::run(stack, &mut channel),
                    update::listen(stack, &conn, rram, true),
                )
                .await;
            }
            links.sink.set(false);
            Timer::after(Duration::from_secs(5)).await;
        }
    };
    join(ranging(stack, fleet, &central, &links, peer, antennas, rram), report).await;
}

/// Scans for a few seconds; the event handler logs the beacons it hears (FR-M4). Neighbour
/// discovery (FR-M1) will build on this.
async fn listen(central: &mut Central<'_, SoftdeviceController<'_>, DefaultPacketPool>) {
    let mut scanner = Scanner::new(central);
    match scanner.scan_ext(&ScanConfig::default()).await {
        Ok(session) => {
            Timer::after(Duration::from_secs(3)).await;
            session.stop().await;
        }
        Err(e) => warn!("scan: {:?}", e),
    }
}

/// How long the initiator tries to connect to its peer before letting the sink link have a go.
const PEER_CONNECT_WITHIN: Duration = Duration::from_secs(10);

/// ACL event length on the initiator: room for a packet pair each way, and short enough that the
/// CS subevent ends with time to spare in a 30 ms interval.
const ACL_EVENT_LENGTH_US: u32 = 2_500;
/// Where a sink link that connects second goes, after the CS link's anchor: as late as its short
/// event allows. It only stays there until the CS link reconnects after it (`Links`).
const CS_FOOTPRINT_US: u32 = 30_000 - ACL_EVENT_LENGTH_US;
/// The sink link's interval: a multiple of the CS link's 30 ms keeps its events at a fixed offset
/// from the CS link's, so they never drift onto a CS subevent.
const SINK_INTERVAL_MS: u64 = 120;

#[derive(Clone, Copy)]
enum Link {
    Cs,
    Sink,
}

/// Which of the initiator's two central links are up. The SDC places a new central link the
/// configured spacing after the running one, so the spacing is set just before each connection
/// (DEF-26). Fixed offsets need the common 30 ms factor.
///
/// Only one order keeps the sink link's events off the CS subevents: the sink link first and the
/// CS link 2.5 ms after it. Spacing a sink link that comes second can't, so when the sink link
/// connects under a CS link, the CS link reconnects after it (`respace`).
#[derive(Default)]
struct Links {
    cs: Cell<bool>,
    sink: Cell<bool>,
    respace: Signal<NoopRawMutex, ()>,
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

/// Connects to the sink, spaced from the CS link (see `Links`), encrypts the link and opens the
/// telemetry channel. Gives up after 10 s, so a missing sink doesn't hold up ranging; the central
/// role is held only while connecting.
async fn uplink<'a>(
    stack: &'a Host<'_, '_>,
    fleet: &keys::Fleet,
    central: &Mutex<CriticalSectionRawMutex, Central<'a, SoftdeviceController<'_>, DefaultPacketPool>>,
    links: &Links,
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
    fleet.trust_address(stack, sink);
    let connected = {
        let mut central = central.lock().await;
        links.space_before(stack, Link::Sink).await;
        with_timeout(Duration::from_secs(10), central.connect_ext(&config)).await
    };
    let conn = match connected {
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
    // Up from now: a CS link connecting next must be spaced from it, and one already up has to
    // reconnect after it (see `Links`).
    links.sink.set(true);
    if links.cs.get() {
        links.respace.signal(());
    }
    // 2M, as on the CS link. Packets stay short except during an update from the sink
    // (`update::listen`): a full-length exchange fills the link's 2.5 ms event (DEF-42).
    if let Err(e) = conn.set_phy(stack, PhyKind::Le2M).await {
        warn!("uplink: set PHY: {:?}", e);
    }
    if !secure(&conn).await || !encrypted(stack, &conn, fleet).await {
        return None;
    }
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

/// Connects to `peer`, encrypts the link, runs the CS initiator sequence, and serves the link until
/// it drops. Repeats forever.
async fn ranging(
    stack: &Host<'_, '_>,
    fleet: &keys::Fleet,
    central: &Mutex<CriticalSectionRawMutex, Central<'_, SoftdeviceController<'_>, DefaultPacketPool>>,
    links: &Links,
    peer: [u8; 6],
    antennas: u8,
    rram: &'static firmware::Rram,
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
        // TrouBLE forgets the key when a link drops.
        fleet.trust_address(stack, peer);
        // Bounded, as the uplink's is: while the peer is away the central role is free between
        // tries, and the sink link can connect and report.
        let connected = {
            let mut central = central.lock().await;
            links.space_before(stack, Link::Cs).await;
            with_timeout(PEER_CONNECT_WITHIN, central.connect_ext(&config)).await
        };
        let conn = match connected {
            Ok(Ok(conn)) => conn,
            Ok(Err(e)) => {
                warn!("connect: {:?}", e);
                continue;
            }
            Err(_) => {
                // The uplink, waiting in the same task, takes the central role before the next try.
                embassy_futures::yield_now().await;
                continue;
            }
        };
        links.respace.reset();
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
        if !secure(&conn).await || !encrypted(stack, &conn, fleet).await {
            links.cs.set(false);
            continue;
        }
        let _link = status::Link::up(false);
        match cs::initiate(stack, conn.handle(), antennas).await {
            Ok(()) => info!("CS running"),
            Err(e) => warn!("CS: {}", e),
        }
        // Also catches a sink link that connected while this one was being set up.
        let serving = async {
            if let Either::Second(()) = select(serve(stack, &conn, fleet), links.respace.wait()).await {
                info!("CS link: reconnecting after the sink link (DEF-26)");
                conn.disconnect();
                serve(stack, &conn, fleet).await;
            }
        };
        join(serving, update::offer(stack, &conn, rram)).await;
        links.cs.set(false);
    }
}

/// Whether `event` shows that the peer isn't in the fleet, on a provisioned node.
///
/// - A peripheral's fleet links encrypt through the controller's key request, never by pairing,
///   so a pairing means an outsider.
/// - A central's encryption under the fleet key fails, with `BondLost`, when the peer lacks the
///   key, and TrouBLE would fall back to pairing. TrouBLE also reports a central's encryption
///   under a stored key as `PairingComplete`, so that means nothing there.
fn outsider(event: &ConnectionEvent, conn: &Connection<'_, DefaultPacketPool>, fleet: &keys::Fleet) -> bool {
    fleet.provisioned()
        && match event {
            ConnectionEvent::BondLost => true,
            ConnectionEvent::PairingComplete { .. } => conn.role() == LeConnRole::Peripheral,
            _ => false,
        }
}

/// Logs a disconnection; a MIC failure means the peer encrypted under another fleet key.
fn disconnected(reason: bt_hci::param::Status) {
    info!("disconnected: {:?}", reason);
    if reason == bt_hci::param::Status::CONN_TERMINATED_DUE_TO_MIC_FAILURE {
        status::foreign();
    }
}

fn log_encrypted(security_level: SecurityLevel, fleet: &keys::Fleet) {
    let how = if fleet.provisioned() { "fleet key" } else { "Just Works" };
    info!("encrypted: {:?}, {=str}", security_level, how);
}

/// Starts encrypting a central's link. TrouBLE runs one security procedure at a time, so this
/// waits out another link's for up to 5 s. Drops the link if it can't start.
async fn secure(conn: &Connection<'_, DefaultPacketPool>) -> bool {
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

/// Handles connection events until the link drops, and drops a peer outside the fleet
/// ([`outsider`]).
async fn serve(stack: &Host<'_, '_>, conn: &Connection<'_, DefaultPacketPool>, fleet: &keys::Fleet) {
    let mut link = None;
    loop {
        let event = conn.next().await;
        if outsider(&event, conn, fleet) {
            warn!("peer outside the fleet: dropping it");
            status::foreign();
            conn.disconnect();
            continue;
        }
        match event {
            ConnectionEvent::Disconnected { reason } => {
                disconnected(reason);
                return;
            }
            // A peripheral's link counts once encrypted; a central counts its own after
            // `encrypted`.
            ConnectionEvent::Encrypted { security_level, .. } => {
                log_encrypted(security_level, fleet);
                if conn.role() == LeConnRole::Peripheral && link.is_none() {
                    link = Some(status::Link::up(false));
                }
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
