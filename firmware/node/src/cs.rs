//! Channel Sounding on the mesh's links.
//!
//! The central of a link runs its ranging bursts (`mesh`): on the first burst it creates the CS
//! configuration (capabilities, config, CS security), then sets the procedure parameters for the
//! burst's procedure count and enables procedures; the controller stops after that many. The
//! event handler collects the subevent results of the burst under way, by connection handle,
//! into a channel response per antenna path, and at each procedure's end estimates a distance
//! (the shortest coherent antenna path, `navgrid-ranging`) into the burst and the telemetry.
//! Results of other handles are the reflector side's and are dropped. With inline PCT transfer
//! the initiator ranges on its own; with `NAVGRID_LOG_STEPS` set at build time both sides also
//! log their raw step data over RTT, for the host to combine offline. The command order follows
//! Nordic's NCS v3.4.1 samples: mode 2 with three mode-0 steps and RTT steps among them, all 72
//! channels, 2M PHY, a 30 ms connection and 22 ms subevents (spike 2026-11-02).

use bt_hci::cmd::le::{
    LeCsCreateConfig, LeCsProcedureEnable, LeCsReadRemoteSupportedCapabilities, LeCsRemoveConfig,
    LeCsSecurityEnableCommand, LeCsSetDefaultSettings, LeCsSetProcedureParams, LeReadRemoteFeatures,
};
use bt_hci::event::le::{
    LeCsConfigComplete, LeCsProcedureEnableComplete, LeCsReadRemoteSupportedCapabilitiesComplete,
    LeCsSecurityEnableComplete, LeCsSubeventResult, LeCsSubeventResultContinue,
};
use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};

use bt_hci::param::{
    ConnHandle, ExtDuration, LeCsSubeventStepData, LeExtAdvReportsIter, PackedAbortReasons, PhyKind, Status,
};
use defmt::{Format, info, warn};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, with_timeout};
use navgrid_link::Burst;
use navgrid_proto::{BEACON_COMPANY, Beacon, NodeId, TONES_CHANNELS, TONES_PCT_LEN};
use navgrid_ranging::steps::{self, MAX_ANTENNA_PATHS, RawStep, Role, Step, ToneQuality, encode_pct};
use navgrid_ranging::{ChannelResponse, Combine, PhaseSlope, SPEED_OF_LIGHT, combine_paths, phase_slope};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

type Stack<'a, 'd> = trouble_host::Stack<'a, SoftdeviceController<'d>, DefaultPacketPool>;

const CONFIG_ID: u8 = 0;
/// Role bits for LE CS Set Default Settings.
const ROLE_INITIATOR: u8 = 1 << 0;
const ROLE_REFLECTOR: u8 = 1 << 1;
/// Let the controller pick the CS_SYNC antenna, repeating the antenna sequence.
const CS_SYNC_ANTENNA_REPETITIVE: u8 = 0xFE;
const MAX_TX_POWER_DBM: i8 = 20;
/// The connection interval bursts run at. 30 ms fits a 22 ms subevent, so four antenna paths
/// take two subevents per procedure rather than four (more subevents widened the spread).
pub const ACL_INTERVAL_MS: u64 = 30;
/// The same interval in 0.625 ms units, as procedure lengths are given.
const ACL_INTERVAL_0625: u16 = (ACL_INTERVAL_MS * 1000 / 625) as u16;
/// Time the SDC sets aside per CS event. With one subevent per event the SDC docs make it equal to
/// the subevent length, so this is the longest one [`procedure_timing`] asks for; NCS's default is
/// 5 ms.
const CS_EVENT_LENGTH_US: u32 = 22_000;
/// Raw step data over RTT, for `navgrid cs`. At the procedure rate below it outruns a 100 kHz SWD
/// link, so it is opt-in: build with `NAVGRID_LOG_STEPS` set.
const LOG_STEPS: bool = option_env!("NAVGRID_LOG_STEPS").is_some();
/// How long to wait for an asynchronous CS command to complete.
const COMPLETION_TIMEOUT: Duration = Duration::from_secs(5);
/// Inline PCT transfer (IPT, `CS_Enhancements_1` bit 0): the reflector returns each tone with the
/// phase it measured added, so the initiator's own PCTs carry the round trip and it can range
/// alone. Needs SDC v3.4.1 or later on the reflector: v3.4.0 applies the IPT phase wrongly when
/// several antenna paths are configured (DRGN-29459).
const INLINE_PCT: bool = true;
/// Fewest adjacent channel pairs for an on-device estimate.
const MIN_PAIRS: u8 = 20;
/// Links that can hold a CS configuration at once. Each costs ~4–7 KB of SDC memory depending
/// on antenna paths (`SDC_MEM_CS` in sdc.h); a configuration takes a slot on both ends of its
/// link, so an anchor's survey bursts remove theirs after (`mesh`).
pub const CS_LINKS: u8 = 6;
const CONFIGS: usize = CS_LINKS as usize + 2;

/// Completions of the asynchronous CS commands, with their connection handle, passed from the
/// host runner's event handler to the burst that waits for them.
#[derive(Format)]
enum Done {
    RemoteCapabilities(u16, Status),
    Config(u16, Status),
    Security(u16, Status),
    Procedure(u16, Status),
}

static DONE: Channel<CriticalSectionRawMutex, Done, 4> = Channel::new();

/// The links with a CS configuration, and the peer's antenna count from its capabilities.
static CONFIGURED: Mutex<CriticalSectionRawMutex, RefCell<heapless::Vec<(u16, u8), CONFIGS>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

/// Procedure interval and maximum procedure length (connection events) and subevent length (µs)
/// for an antenna path count. The SDC places CS events two connection events apart, so a
/// procedure takes at least two. All 72 channels fit one 11 ms subevent with one path, one 22 ms
/// subevent with two, and two with four. Measured at 30 ms: 16.7 procedures/s with one path,
/// 11.1 with four (spike 2026-11-02-cs-ranging-rate).
fn procedure_timing(paths: u8) -> (u16, u16, u32) {
    match paths {
        1 => (2, 1, 11_000),
        2 => (2, 1, 22_000),
        _ => (3, 2, 22_000),
    }
}

/// Tone antenna configuration index (LE CS Set Procedure Parameters) and preferred peer antenna
/// mask for the local and peer antenna counts, using up to two antennas each. Indices: 0 A1:B1,
/// 1 A2:B1, 4 A1:B2, 7 A2:B2 (A initiator, B reflector); antenna paths = A × B.
fn tone_antenna_config(local: u8, peer: u8) -> (u8, u8) {
    match (local >= 2, peer >= 2) {
        (true, true) => (7, 0b11),
        (true, false) => (1, 0b01),
        (false, true) => (4, 0b11),
        (false, false) => (0, 0b01),
    }
}

/// Antenna paths of the tone antenna configurations [`tone_antenna_config`] picks.
fn antenna_paths(config: u8) -> u8 {
    match config {
        7 => 4,
        1 | 4 => 2,
        _ => 1,
    }
}

#[derive(Format)]
pub enum CsError {
    /// The controller refused a command.
    Command(&'static str),
    /// A command's completion event reported a failure.
    Failed(&'static str, Status),
    /// No completion event arrived.
    Timeout(&'static str),
    /// The link dropped during the burst.
    Dropped,
}

/// Host event handler: completions go to the burst under way; subevent results of its handle are
/// collected, and the navgrid beacons heard while scanning go to the mesh policy (FR-M4).
pub struct Events;

impl EventHandler for Events {
    fn on_ext_adv_reports(&self, reports: LeExtAdvReportsIter) {
        for report in reports.flatten() {
            for ad in AdStructure::decode(report.data).flatten() {
                if let AdStructure::ManufacturerSpecificData {
                    company_identifier: BEACON_COMPANY,
                    payload,
                } = ad
                    && let Some(beacon) = Beacon::decode(payload)
                {
                    let id = crate::mesh::node_id(report.addr.raw());
                    if crate::mesh::HEARD.try_send((id, beacon, report.rssi)).is_err() {
                        warn!("beacon from {=u64:012x}: no room to keep it", id.0);
                    }
                    if crate::status::is_sink() {
                        crate::report::push_heard(crate::mesh::SELF.load(), id, report.rssi, &beacon);
                    }
                }
            }
        }
    }

    fn on_cs_read_remote_supported_capabilities_complete(&self, e: &LeCsReadRemoteSupportedCapabilitiesComplete) {
        info!(
            "CS peer on {}: antennas={} paths={} roles={=u8:#04x} modes={=u8:#04x}",
            e.handle.raw(),
            e.num_antennas_supported,
            e.max_antenna_paths_supported,
            e.roles_supported,
            e.optional_modes_supported
        );
        let antennas = e.num_antennas_supported;
        CONFIGURED.lock(|c| {
            let mut c = c.borrow_mut();
            match c.iter_mut().find(|(h, _)| *h == e.handle.raw()) {
                Some(entry) => entry.1 = antennas,
                None => {
                    let _ = c.push((e.handle.raw(), antennas));
                }
            }
        });
        let _ = DONE.try_send(Done::RemoteCapabilities(e.handle.raw(), e.status));
    }

    fn on_cs_config_complete(&self, e: &LeCsConfigComplete) {
        info!(
            "CS config {} on {} action {}: {}, mode {}/{}, steps {}-{}, mode-0 {}, map {=[u8]:02x}, enhancements {=u8:#04x}, T_IP1 {} T_IP2 {} T_FCS {} T_PM {} us",
            e.config_id,
            e.handle.raw(),
            e.action,
            e.status,
            e.main_mode_type,
            e.sub_mode_type,
            e.min_main_mode_steps,
            e.max_main_mode_steps,
            e.mode_0_steps,
            e.channel_map,
            e.reserved,
            e.t_ip1_time,
            e.t_ip2_time,
            e.t_fcs_time,
            e.t_pm_time
        );
        let _ = DONE.try_send(Done::Config(e.handle.raw(), e.status));
    }

    fn on_cs_security_enable_complete(&self, e: &LeCsSecurityEnableComplete) {
        info!("CS security on {}: {}", e.handle.raw(), e.status);
        let _ = DONE.try_send(Done::Security(e.handle.raw(), e.status));
    }

    fn on_cs_procedure_enable_complete(&self, e: &LeCsProcedureEnableComplete) {
        info!(
            "CS procedures on {} {}: every {} events (event interval {}), max length {} us, {} subevent(s) of {} us every {} us, tx {} dBm: {}",
            e.handle.raw(),
            if e.state { "on" } else { "off" },
            e.procedure_interval,
            e.event_interval,
            e.max_procedure_len.as_micros(),
            e.subevents_per_event,
            e.subevent_len.as_micros(),
            e.subevent_interval.as_micros(),
            e.selected_tx_power,
            e.status
        );
        let _ = DONE.try_send(Done::Procedure(e.handle.raw(), e.status));
    }

    fn on_cs_subevent_result(&self, e: &LeCsSubeventResult<'_>) {
        let handle = e.connection_handle.raw();
        if LOG_STEPS {
            log_steps(
                "CSR",
                handle,
                e.procedure_counter,
                e.frequency_compensation.as_raw(),
                e.reference_power_level,
                Done2(e.procedure_done_status as u8, e.subevent_done_status as u8),
                abort_byte(&e.abort_reason),
                e.num_antenna_paths,
                &e.steps,
            );
        }
        range(
            handle,
            Some(e.procedure_counter),
            e.procedure_done_status as u8,
            e.num_antenna_paths,
            &e.steps,
        );
    }

    fn on_cs_subevent_result_continue(&self, e: &LeCsSubeventResultContinue<'_>) {
        let handle = e.connection_handle.raw();
        // A continuation carries no procedure counter or frequency compensation of its own; the
        // host takes them from the preceding CSR line.
        if LOG_STEPS {
            log_steps(
                "CSC",
                handle,
                0,
                0,
                0,
                Done2(e.procedure_done_status as u8, e.subevent_done_status as u8),
                abort_byte(&e.abort_reason),
                e.num_antenna_paths,
                &e.steps,
            );
        }
        range(handle, None, e.procedure_done_status as u8, e.num_antenna_paths, &e.steps);
    }
}

/// The procedure being collected for an on-device estimate: one response per antenna path.
struct Ranging {
    procedure: u16,
    responses: [ChannelResponse; MAX_ANTENNA_PATHS as usize],
    /// The mode-1 steps' ToA−ToD so far, summed, in 0.5 ns, and how many.
    rtt_half_ns: i32,
    rtt_steps: u8,
}

impl Ranging {
    const EMPTY_RESPONSES: [ChannelResponse; MAX_ANTENNA_PATHS as usize] =
        [const { ChannelResponse::new() }; MAX_ANTENNA_PATHS as usize];

    const fn new() -> Self {
        Self {
            procedure: 0,
            responses: Self::EMPTY_RESPONSES,
            rtt_half_ns: 0,
            rtt_steps: 0,
        }
    }

    fn clear(&mut self) {
        self.responses = Self::EMPTY_RESPONSES;
        self.rtt_half_ns = 0;
        self.rtt_steps = 0;
    }

    /// Mean one-way distance from the mode-1 steps, with the reflector's turnaround taken as zero:
    /// a bias that doesn't depend on distance, which is what a referee for the phase slope's scale
    /// needs. Negative means are reported as zero.
    fn rtt(&self) -> Option<(f32, u8)> {
        (self.rtt_steps > 0).then(|| {
            let round_trip_s = self.rtt_half_ns as f32 / f32::from(self.rtt_steps) * 0.5e-9;
            (round_trip_s * SPEED_OF_LIGHT / 2.0, self.rtt_steps)
        })
    }
}

/// The burst under way on this node, as its initiator: one at a time (`navgrid-link`).
struct Active {
    handle: u16,
    node: NodeId,
    peer: NodeId,
    ranging: Ranging,
    burst: Burst,
    /// Procedures asked for, and procedures that ended (with a distance or not).
    wanted: u8,
    ended: u8,
}

static ACTIVE: Mutex<CriticalSectionRawMutex, RefCell<Option<Active>>> = Mutex::new(RefCell::new(None));
/// The active burst's procedures have all ended, or its link dropped.
static BURST_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn is_active(handle: u16) -> bool {
    ACTIVE.lock(|a| a.borrow().as_ref().is_some_and(|a| a.handle == handle))
}

/// The outcome of one procedure on the initiator.
enum Ranged {
    /// The per-path estimates combined, and how many paths gave one.
    Distance(PhaseSlope, usize),
    /// No path had enough channels; the most any path had.
    TooFewChannels(usize),
    Aborted,
}

/// On the initiator with inline PCT transfer, adds the subevent's mode-2 tones to the active
/// burst's current procedure, per antenna path, and at the procedure's end estimates a distance
/// into the burst and the telemetry. `procedure` is `None` for a continuation, which belongs to
/// the procedure of the preceding result. Results of any other handle are dropped.
fn range(handle: u16, procedure: Option<u16>, procedure_done: u8, paths: u8, steps: &LeCsSubeventStepData<'_>) {
    if !INLINE_PCT {
        return;
    }
    let done = ACTIVE.lock(|a| {
        let mut a = a.borrow_mut();
        let a = a.as_mut().filter(|a| a.handle == handle)?;
        let r = &mut a.ranging;
        if let Some(p) = procedure
            && p != r.procedure
        {
            r.procedure = p;
            r.clear();
        }
        for step in steps.iter() {
            let raw = RawStep {
                mode: step.step_mode,
                channel: step.step_channel,
                data: step.step_data,
            };
            match steps::decode(Role::Initiator, paths, raw) {
                Ok(Step::Mode2(tones)) => {
                    for path in 0..tones.paths() {
                        if let Some(tone) = tones.path(path)
                            && tone.quality != ToneQuality::Unavailable
                        {
                            r.responses[usize::from(path)].insert(raw.channel, tone.pct);
                        }
                    }
                }
                // A packet that wasn't received has no time (0x8000) and a failed access-address
                // check (quality 2 or more in the low nibble); either would swamp the mean.
                Ok(Step::Mode1(rtt)) if rtt.toa_tod != i16::MIN && rtt.quality & 0x0f < 2 => {
                    r.rtt_half_ns = r.rtt_half_ns.saturating_add(i32::from(rtt.toa_tod));
                    r.rtt_steps = r.rtt_steps.saturating_add(1);
                }
                _ => {}
            }
        }
        // 0: all results in; 1: more to come; 0xF: aborted.
        if procedure_done == 1 {
            return None;
        }
        let ranged = if procedure_done == 0 {
            let unused = PhaseSlope {
                distance_m: 0.0,
                coherence: 0.0,
                residual_rad: 0.0,
                pairs: 0,
            };
            let mut slopes = [unused; MAX_ANTENNA_PATHS as usize];
            let mut n = 0;
            for response in &r.responses {
                if let Some(slope) = phase_slope(response, MIN_PAIRS) {
                    slopes[n] = slope;
                    n += 1;
                }
            }
            match combine_paths(&slopes[..n], Combine::ShortestCoherent) {
                Some(combined) => Ranged::Distance(combined, n),
                None => Ranged::TooFewChannels(r.responses.iter().map(ChannelResponse::len).max().unwrap_or(0)),
            }
        } else {
            Ranged::Aborted
        };
        let rtt = r.rtt();
        if procedure_done == 0 {
            send_tones(a.node, r.procedure, &r.responses);
        }
        let p = r.procedure;
        r.clear();
        match &ranged {
            Ranged::Distance(d, _) => a.burst.add(d.distance_m, navgrid_ranging::range_quality(d)),
            _ => a.burst.fail(),
        }
        a.ended = a.ended.saturating_add(1);
        if a.ended >= a.wanted {
            BURST_DONE.signal(());
        }
        Some((a.node, a.peer, p, ranged, rtt))
    });
    let Some((node, peer, p, ranged, rtt)) = done else {
        return;
    };
    if let Some((distance_m, steps)) = rtt {
        crate::report::push_rtt(node, peer, distance_m, steps);
        info!("rtt {=u16}: {=f32} m from {=u8} step(s)", p, distance_m, steps);
    }
    match ranged {
        Ranged::Distance(d, n) => {
            crate::report::push(node, peer, &d, n);
            info!(
                "distance {=u16}: {=f32} m from {=usize} path(s), coherence {=f32}, residual {=f32} rad, {=u8} pairs",
                p, d.distance_m, n, d.coherence, d.residual_rad, d.pairs
            );
        }
        Ranged::TooFewChannels(channels) => {
            warn!("distance {=u16}: too few channels ({=usize})", p, channels)
        }
        Ranged::Aborted => warn!("distance {=u16}: procedure aborted", p),
    }
}

/// How often an initiator sends one antenna path's raw tones to the sink (`Message::Tones`).
const TONES_EVERY: Duration = Duration::from_secs(1);
/// One past the last CS channel (76).
const CS_CHANNELS_END: u8 = 77;
static TONES_LAST: Mutex<CriticalSectionRawMutex, Cell<Option<Instant>>> = Mutex::new(Cell::new(None));
static TONES_SENT: AtomicU32 = AtomicU32::new(0);

/// Queues one antenna path's tones of the procedure just completed, at most once a second, the
/// paths in turn: three chunks of [`TONES_CHANNELS`] channels from channel 2, zeros where a
/// channel had no usable tone.
fn send_tones(node: NodeId, procedure: u16, responses: &[ChannelResponse; MAX_ANTENNA_PATHS as usize]) {
    let now = Instant::now();
    let due = TONES_LAST.lock(|l| match l.get() {
        Some(t) if now - t < TONES_EVERY => false,
        _ => {
            l.set(Some(now));
            true
        }
    });
    if !due {
        return;
    }
    let paths = responses.iter().filter(|r| !r.is_empty()).count();
    if paths == 0 {
        return;
    }
    let path = TONES_SENT.fetch_add(1, Ordering::Relaxed) as usize % paths;
    let resp = &responses[path];
    let mut first = 2u8;
    while first < CS_CHANNELS_END {
        let end = (first + TONES_CHANNELS as u8).min(CS_CHANNELS_END);
        let mut pct = heapless::Vec::<u8, TONES_PCT_LEN>::new();
        for channel in first..end {
            let bytes = resp.get(channel).map_or([0; 3], encode_pct);
            let _ = pct.extend_from_slice(&bytes);
        }
        crate::report::push_tones(node, procedure, path as u8, paths as u8, first, pct);
        first = end;
    }
}

/// Procedure and subevent done status (0 complete, 1 partial, 0xF aborted).
struct Done2(u8, u8);

/// The abort reasons as the controller packs them: procedure in bits 0–3, subevent in bits 4–7.
fn abort_byte(reasons: &PackedAbortReasons) -> u8 {
    ((reasons.subevent_reason() as u8) << 4) | reasons.procedure_reason() as u8
}

/// Logs one subevent as a single line, for the host to parse:
/// `<tag> <role> <procedure counter> <frequency compensation> <reference power> <procedure done>
/// <subevent done> <abort reasons> <antenna paths> <steps> <step bytes>`. Done statuses and abort
/// reasons are the HCI values (abort: procedure in the low nibble, subevent in the high one). The
/// step bytes keep the HCI layout (mode, channel, length, data, per step), so the host parses them
/// exactly as it would a controller event.
#[allow(clippy::too_many_arguments)]
fn log_steps(
    tag: &str,
    handle: u16,
    procedure: u16,
    freq_comp: u16,
    ref_power: i8,
    done: Done2,
    abort: u8,
    paths: u8,
    steps: &LeCsSubeventStepData<'_>,
) {
    let mut buf = [0u8; 255];
    let mut len = 0;
    for step in steps.iter() {
        let end = len + 3 + step.step_data.len();
        if end > buf.len() {
            warn!("{=str}: step data does not fit the log buffer", tag);
            break;
        }
        buf[len..len + 3].copy_from_slice(&[step.step_mode, step.step_channel, step.step_data_length]);
        buf[len + 3..end].copy_from_slice(step.step_data);
        len = end;
    }
    info!(
        "{=str} {=str} {=u16} {=u16} {=i8} {=u8:x} {=u8:x} {=u8:02x} {=u8} {=usize} {=[u8]:02x}",
        tag,
        if is_active(handle) { "I" } else { "R" },
        procedure,
        freq_comp,
        ref_power,
        done.0,
        done.1,
        abort,
        paths,
        steps.len(),
        &buf[..len]
    );
}

/// Runs the LL feature exchange from the host, as Zephyr does on every connection. The SDC
/// (nrfxlib 3.4) never completes LE CS Read Remote Supported Capabilities when that command has to
/// start the feature exchange itself: neither side gets a completion event and the link idles. The
/// completion event of this command is masked; only the exchange matters.
pub async fn exchange_features(stack: &Stack<'_, '_>, handle: ConnHandle) {
    if let Err(e) = stack.async_command(LeReadRemoteFeatures::new(handle)).await {
        warn!("read remote features: {:?}", e);
    }
}

/// Sets the time the SDC sets aside for each CS event, at the point NCS does
/// (`CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT`), but to [`CS_EVENT_LENGTH_US`]. Call once, after the
/// SDC is built and before any CS configuration.
pub fn set_event_length() -> Result<(), Status> {
    use nrf_sdc::raw::{
        SDC_HCI_VS_CS_PARAM_TYPE_CS_EVENT_LENGTH_SET, sdc_hci_cmd_vs_cs_params_set, sdc_hci_cmd_vs_cs_params_set_t,
        sdc_hci_cmd_vs_cs_params_set_t__bindgen_ty_1, sdc_hci_vs_cs_event_length_params_t,
    };
    let params = sdc_hci_cmd_vs_cs_params_set_t {
        cs_param_type: SDC_HCI_VS_CS_PARAM_TYPE_CS_EVENT_LENGTH_SET as u8,
        cs_param_data: sdc_hci_cmd_vs_cs_params_set_t__bindgen_ty_1 {
            cs_event_length_params: sdc_hci_vs_cs_event_length_params_t {
                cs_event_length_us: CS_EVENT_LENGTH_US,
            },
        },
    };
    // SAFETY: a plain synchronous SDC call with a valid, fully initialised parameter struct.
    let status = Status::from(unsafe { sdc_hci_cmd_vs_cs_params_set(&params) });
    status.to_result().map_err(|_| status)
}

/// Sets the preferred CS tone length (T_PM) to 10 us, as NCS does after enabling the controller
/// (`CONFIG_BT_CTLR_SDC_CS_T_PM_LEN_DEFAULT`). Call once, before any CS configuration.
pub fn set_tone_length() -> Result<(), Status> {
    use nrf_sdc::raw::{
        SDC_HCI_VS_CS_PARAM_TYPE_CS_T_PM_SET, sdc_hci_cmd_vs_cs_params_set, sdc_hci_cmd_vs_cs_params_set_t,
        sdc_hci_cmd_vs_cs_params_set_t__bindgen_ty_1, sdc_hci_vs_cs_t_pm_params_t,
    };
    let params = sdc_hci_cmd_vs_cs_params_set_t {
        cs_param_type: SDC_HCI_VS_CS_PARAM_TYPE_CS_T_PM_SET as u8,
        cs_param_data: sdc_hci_cmd_vs_cs_params_set_t__bindgen_ty_1 {
            cs_t_pm_params: sdc_hci_vs_cs_t_pm_params_t { cs_t_pm_length_us: 10 },
        },
    };
    // SAFETY: a plain synchronous SDC call with a valid, fully initialised parameter struct.
    let status = Status::from(unsafe { sdc_hci_cmd_vs_cs_params_set(&params) });
    status.to_result().map_err(|_| status)
}

/// The reflector's part: allow the reflector role on this link.
pub async fn reflect(stack: &Stack<'_, '_>, handle: ConnHandle) -> Result<(), CsError> {
    stack
        .command(LeCsSetDefaultSettings::new(
            handle,
            ROLE_REFLECTOR,
            CS_SYNC_ANTENNA_REPETITIVE,
            MAX_TX_POWER_DBM,
        ))
        .await
        .map_err(|_| CsError::Command("set default settings"))?;
    Ok(())
}

/// A ranging burst on the link `handle` to `peer`, as its initiator: `procedures` procedures at
/// the link's current connection interval, one estimate each into the returned [`Burst`] and
/// the telemetry. Creates the CS configuration on the first burst of a link. Returns when the
/// controller has run the procedures, the link drops, or the burst takes too long; the burst
/// then holds what came in.
pub async fn burst(
    stack: &Stack<'_, '_>,
    handle: ConnHandle,
    node: NodeId,
    peer: NodeId,
    antennas: u8,
    procedures: u8,
) -> Result<Burst, CsError> {
    BURST_DONE.reset();
    ACTIVE.lock(|a| {
        *a.borrow_mut() = Some(Active {
            handle: handle.raw(),
            node,
            peer,
            ranging: Ranging::new(),
            burst: Burst::new(peer, 0),
            wanted: procedures.max(1),
            ended: 0,
        })
    });
    let result = initiate(stack, handle, antennas, procedures).await;
    let paths = match result {
        Ok(paths) => paths,
        Err(e) => {
            ACTIVE.lock(|a| *a.borrow_mut() = None);
            return Err(e);
        }
    };
    ACTIVE.lock(|a| {
        if let Some(a) = a.borrow_mut().as_mut() {
            a.burst.paths = paths;
        }
    });
    // Four paths take 90 ms a procedure; leave room for the setup and a few empty ones.
    let budget = Duration::from_millis(300 * u64::from(procedures) + 3_000);
    if with_timeout(budget, BURST_DONE.wait()).await.is_err() {
        warn!("CS burst on {}: {=u8} procedures didn't end in time", handle.raw(), procedures);
        if let Err(e) = stack.async_command(LeCsProcedureEnable::new(handle, CONFIG_ID, false)).await {
            warn!("CS procedure disable: {:?}", e);
        }
    }
    let active = ACTIVE.lock(|a| a.borrow_mut().take());
    match active {
        Some(a) => Ok(a.burst),
        None => Err(CsError::Dropped),
    }
}

/// Removes the link's CS configuration, freeing its slot on both controllers; the next burst
/// creates it again.
pub async fn remove_config(stack: &Stack<'_, '_>, handle: ConnHandle) {
    if !CONFIGURED.lock(|c| c.borrow().iter().any(|(h, _)| *h == handle.raw())) {
        return;
    }
    if let Err(e) = stack.async_command(LeCsRemoveConfig::new(handle, CONFIG_ID)).await {
        warn!("CS remove config: {:?}", e);
        return;
    }
    if let Err(e) = completed("remove config", handle.raw(), |d| match d {
        Done::Config(h, s) => Some((h, s)),
        _ => None,
    })
    .await
    {
        warn!("CS remove config: {}", e);
    }
    CONFIGURED.lock(|c| c.borrow_mut().retain(|(h, _)| *h != handle.raw()));
}

/// The link `handle` dropped: its configuration went with it, and a burst on it is over.
pub fn forget(handle: ConnHandle) {
    CONFIGURED.lock(|c| c.borrow_mut().retain(|(h, _)| *h != handle.raw()));
    if is_active(handle.raw()) {
        BURST_DONE.signal(());
    }
}

/// The initiator's sequence on an encrypted link: default settings and, the first time, peer
/// capabilities, config and CS security; then the procedure parameters for `procedures`
/// procedures and procedure enable. `antennas` is how many antennas this node may use for CS.
/// Returns the antenna path count.
async fn initiate(stack: &Stack<'_, '_>, handle: ConnHandle, antennas: u8, procedures: u8) -> Result<u8, CsError> {
    while DONE.try_receive().is_ok() {}
    let h = handle.raw();

    stack
        .command(LeCsSetDefaultSettings::new(
            handle,
            ROLE_INITIATOR,
            CS_SYNC_ANTENNA_REPETITIVE,
            MAX_TX_POWER_DBM,
        ))
        .await
        .map_err(|_| CsError::Command("set default settings"))?;

    let configured = CONFIGURED.lock(|c| c.borrow().iter().any(|(x, _)| *x == h));
    if !configured {
        stack
            .async_command(LeCsReadRemoteSupportedCapabilities::new(handle))
            .await
            .map_err(|_| CsError::Command("read remote capabilities"))?;
        completed("read remote capabilities", h, |d| match d {
            Done::RemoteCapabilities(x, s) => Some((x, s)),
            _ => None,
        })
        .await?;

        stack
            .async_command(LeCsCreateConfig::new(
                handle,
                CONFIG_ID,
                1,    // create context: local and remote
                2,    // main mode 2: phase-based ranging
                1,    // sub mode 1: RTT steps among the tone steps, a referee for the phase slope
                2,    // min main mode steps
                5,    // max main mode steps
                0,    // main mode repetition
                3,    // mode-0 steps
                0,    // role: initiator
                0,    // RTT type: access address only
                1,    // CS_SYNC PHY: 1M
                channel_map(),
                1,                    // channel map repetition
                0,                    // channel selection algorithm #3b
                0,                    // ch3c shape: hat (unused by #3b)
                2,                    // ch3c jump (unused by #3b)
                u8::from(INLINE_PCT), // cs_enhancements (Core 6.3): bit 0, inline PCT transfer
            ))
            .await
            .map_err(|_| CsError::Command("create config"))?;
        completed("create config", h, |d| match d {
            Done::Config(x, s) => Some((x, s)),
            _ => None,
        })
        .await?;

        stack
            .async_command(LeCsSecurityEnableCommand::new(handle))
            .await
            .map_err(|_| CsError::Command("security enable"))?;
        completed("security enable", h, |d| match d {
            Done::Security(x, s) => Some((x, s)),
            _ => None,
        })
        .await?;
    }

    let peer = CONFIGURED.lock(|c| c.borrow().iter().find(|(x, _)| *x == h).map_or(1, |(_, a)| *a));
    let (antenna_config, peer_antennas) = tone_antenna_config(antennas, peer);
    let paths = antenna_paths(antenna_config);
    let (interval, length, subevent_us) = procedure_timing(paths);
    info!(
        "CS burst on {}: {=u8} procedures, antennas {=u8} local, {=u8} peer, tone antenna config {=u8} ({=u8} paths)",
        h, procedures, antennas, peer, antenna_config, paths
    );
    stack
        .command(LeCsSetProcedureParams::new(
            handle,
            CONFIG_ID,
            bt_hci::param::Duration::from_u16(ACL_INTERVAL_0625 * length),
            interval, // min, in connection events
            interval, // max
            u16::from(procedures),
            ExtDuration::from_u32(subevent_us),
            ExtDuration::from_u32(subevent_us),
            antenna_config,
            PhyKind::Le2M,
            i8::MIN, // tx power delta: no recommendation
            peer_antennas,
            0xFF, // SNR control, initiator: not used
            0xFF, // SNR control, reflector: not used
        ))
        .await
        .map_err(|_| CsError::Command("set procedure parameters"))?;

    stack
        .async_command(LeCsProcedureEnable::new(handle, CONFIG_ID, true))
        .await
        .map_err(|_| CsError::Command("procedure enable"))?;
    completed("procedure enable", h, |d| match d {
        Done::Procedure(x, s) => Some((x, s)),
        _ => None,
    })
    .await?;
    Ok(paths)
}

/// Waits for the completion that `pick` accepts for `handle` and checks its status. Other
/// completions are dropped; they belong to no command in flight.
async fn completed(what: &'static str, handle: u16, pick: fn(Done) -> Option<(u16, Status)>) -> Result<(), CsError> {
    loop {
        let done = with_timeout(COMPLETION_TIMEOUT, DONE.receive())
            .await
            .map_err(|_| CsError::Timeout(what))?;
        if let Some((h, status)) = pick(done)
            && h == handle
        {
            return status.to_result().map_err(|_| CsError::Failed(what, status));
        }
    }
}

/// All channels CS may use: 2–22 and 26–76.
fn channel_map() -> [u8; 10] {
    let mut map = [0u8; 10];
    for channel in (2..=22).chain(26..=76) {
        map[channel / 8] |= 1 << (channel % 8);
    }
    map
}
