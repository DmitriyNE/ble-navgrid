//! Channel Sounding bring-up (phase P0).
//!
//! One board initiates and one reflects. Both log their raw step data over RTT, so the host can
//! combine the two sides offline; no RAS or inline PCT transfer is needed yet. The command order
//! and parameter values follow Nordic's NCS v3.4.1 `ras_initiator` sample: mode 2 with three
//! mode-0 steps, all 72 channels, a 1×1 antenna configuration, 2M PHY, 16 ms subevents, and a
//! procedure every 10 connection events. The connection interval differs (see
//! [`ACL_INTERVAL_MS`]). Like Zephyr's host, both sides run the feature exchange on connect and
//! the initiator moves the link to 2M.
//!
//! Build-time role (no provisioning yet): `NAVGRID_ROLE=initiator` with `NAVGRID_PEER` set to the
//! reflector's node ID as it logs it at boot (12 hex digits). Anything else builds a reflector.

use bt_hci::cmd::le::{
    LeCsCreateConfig, LeCsProcedureEnable, LeCsReadRemoteSupportedCapabilities, LeCsSecurityEnableCommand,
    LeCsSetDefaultSettings, LeCsSetProcedureParams, LeReadRemoteFeatures,
};
use bt_hci::event::le::{
    LeCsConfigComplete, LeCsProcedureEnableComplete, LeCsReadRemoteSupportedCapabilitiesComplete,
    LeCsSecurityEnableComplete, LeCsSubeventResult, LeCsSubeventResultContinue,
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

use bt_hci::param::{ConnHandle, ExtDuration, LeCsSubeventStepData, PackedAbortReasons, PhyKind, Status};
use defmt::{Format, info, warn};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, with_timeout};
use navgrid_ranging::steps::{self, MAX_ANTENNA_PATHS, RawStep, Role, Step, ToneQuality};
use navgrid_ranging::{ChannelResponse, Combine, PhaseSlope, combine_paths, phase_slope};
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
/// ACL connection interval the initiator asks for. At 20 ms (Nordic's sample) every CS subevent
/// aborted with a scheduling conflict; Nordic acknowledges an SDC bug of that kind (DEF-18).
pub const ACL_INTERVAL_MS: u64 = 50;
/// The same interval in 0.625 ms units, as procedure lengths are given.
const ACL_INTERVAL_0625: u16 = (ACL_INTERVAL_MS * 1000 / 625) as u16;
/// A CS procedure every this many connection events.
const PROCEDURE_INTERVAL: u16 = 10;
/// Time the SDC sets aside per CS event. With one subevent per event the SDC docs make it equal to
/// the subevent length; NCS's default is 5 ms.
const CS_EVENT_LENGTH_US: u32 = SUBEVENT_LENGTH_US;
/// Length of each CS subevent, as in Nordic's sample.
const SUBEVENT_LENGTH_US: u32 = 16_000;
/// How long to wait for an asynchronous CS command to complete.
const COMPLETION_TIMEOUT: Duration = Duration::from_secs(5);
/// Inline PCT transfer (IPT, `CS_Enhancements_1` bit 0): the reflector returns each tone with the
/// phase it measured added, so the initiator's own PCTs carry the round trip and it can range
/// alone. Needs SDC v3.4.1 or later on the reflector: v3.4.0 applies the IPT phase wrongly when
/// several antenna paths are configured (DRGN-29459).
const INLINE_PCT: bool = true;
/// Fewest adjacent channel pairs for an on-device estimate.
const MIN_PAIRS: u8 = 20;

/// Bring-up role, from `NAVGRID_ROLE` at build time.
pub fn is_initiator() -> bool {
    option_env!("NAVGRID_ROLE") == Some("initiator")
}

/// The reflector's address, from `NAVGRID_PEER` at build time: its node ID as logged at boot,
/// 12 hex digits, most significant first.
pub fn peer() -> Option<[u8; 6]> {
    let hex = option_env!("NAVGRID_PEER")?;
    let id = u64::from_str_radix(hex, 16).ok().filter(|_| hex.len() == 12)?;
    let mut addr = [0u8; 6];
    addr.copy_from_slice(&id.to_le_bytes()[..6]);
    Some(addr)
}

/// Completions of the asynchronous CS commands, passed from the host runner's event handler to
/// the task that runs the initiator sequence.
#[derive(Format)]
enum Done {
    RemoteCapabilities(Status),
    Config(Status),
    Security(Status),
    Procedure(Status),
}

static DONE: Channel<CriticalSectionRawMutex, Done, 4> = Channel::new();

/// The peer's antenna count from its CS capabilities.
static PEER_ANTENNAS: AtomicU8 = AtomicU8::new(1);

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

#[derive(Format)]
pub enum CsError {
    /// The controller refused a command.
    Command(&'static str),
    /// A command's completion event reported a failure.
    Failed(&'static str, Status),
    /// No completion event arrived.
    Timeout(&'static str),
}

/// Host event handler: completions go to the initiator sequence; subevent results are logged.
pub struct Events;

impl EventHandler for Events {
    fn on_cs_read_remote_supported_capabilities_complete(&self, e: &LeCsReadRemoteSupportedCapabilitiesComplete) {
        info!(
            "CS peer: antennas={} paths={} roles={=u8:#04x} modes={=u8:#04x}",
            e.num_antennas_supported, e.max_antenna_paths_supported, e.roles_supported, e.optional_modes_supported
        );
        PEER_ANTENNAS.store(e.num_antennas_supported, Ordering::Relaxed);
        let _ = DONE.try_send(Done::RemoteCapabilities(e.status));
    }

    fn on_cs_config_complete(&self, e: &LeCsConfigComplete) {
        info!(
            "CS config {} action {}: {}, mode {}/{}, steps {}-{}, mode-0 {}, map {=[u8]:02x}, enhancements {=u8:#04x}, T_IP1 {} T_IP2 {} T_FCS {} T_PM {} us",
            e.config_id,
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
        let _ = DONE.try_send(Done::Config(e.status));
    }

    fn on_cs_security_enable_complete(&self, e: &LeCsSecurityEnableComplete) {
        info!("CS security: {}", e.status);
        let _ = DONE.try_send(Done::Security(e.status));
    }

    fn on_cs_procedure_enable_complete(&self, e: &LeCsProcedureEnableComplete) {
        info!(
            "CS procedures {}: every {} events (event interval {}), max length {} us, {} subevent(s) of {} us every {} us, tx {} dBm: {}",
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
        let _ = DONE.try_send(Done::Procedure(e.status));
    }

    fn on_cs_subevent_result(&self, e: &LeCsSubeventResult<'_>) {
        log_steps(
            "CSR",
            e.procedure_counter,
            e.frequency_compensation.as_raw(),
            e.reference_power_level,
            Done2(e.procedure_done_status as u8, e.subevent_done_status as u8),
            abort_byte(&e.abort_reason),
            e.num_antenna_paths,
            &e.steps,
        );
        range(
            Some(e.procedure_counter),
            e.procedure_done_status as u8,
            e.num_antenna_paths,
            &e.steps,
        );
    }

    fn on_cs_subevent_result_continue(&self, e: &LeCsSubeventResultContinue<'_>) {
        // A continuation carries no procedure counter or frequency compensation of its own; the
        // host takes them from the preceding CSR line.
        log_steps(
            "CSC",
            0,
            0,
            0,
            Done2(e.procedure_done_status as u8, e.subevent_done_status as u8),
            abort_byte(&e.abort_reason),
            e.num_antenna_paths,
            &e.steps,
        );
        range(None, e.procedure_done_status as u8, e.num_antenna_paths, &e.steps);
    }
}

/// The procedure being collected for an on-device estimate: one response per antenna path.
struct Ranging {
    procedure: u16,
    responses: [ChannelResponse; MAX_ANTENNA_PATHS as usize],
}

static RANGING: Mutex<CriticalSectionRawMutex, RefCell<Ranging>> = Mutex::new(RefCell::new(Ranging {
    procedure: 0,
    responses: [const { ChannelResponse::new() }; MAX_ANTENNA_PATHS as usize],
}));

/// The outcome of one procedure on the initiator.
enum Ranged {
    /// The per-path estimates combined, and how many paths gave one.
    Distance(PhaseSlope, usize),
    /// No path had enough channels; the most any path had.
    TooFewChannels(usize),
    Aborted,
}

/// On the initiator with inline PCT transfer, adds the subevent's mode-2 tones to the current
/// procedure, per antenna path, and logs a distance once the procedure completes. `procedure` is
/// `None` for a continuation, which belongs to the procedure of the preceding result.
fn range(procedure: Option<u16>, procedure_done: u8, paths: u8, steps: &LeCsSubeventStepData<'_>) {
    if !(INLINE_PCT && is_initiator()) {
        return;
    }
    let done = RANGING.lock(|r| {
        let mut r = r.borrow_mut();
        if let Some(p) = procedure
            && p != r.procedure
        {
            r.procedure = p;
            r.responses = [const { ChannelResponse::new() }; MAX_ANTENNA_PATHS as usize];
        }
        for step in steps.iter() {
            let raw = RawStep {
                mode: step.step_mode,
                channel: step.step_channel,
                data: step.step_data,
            };
            let Ok(Step::Mode2(tones)) = steps::decode(Role::Initiator, paths, raw) else {
                continue;
            };
            for path in 0..tones.paths() {
                if let Some(tone) = tones.path(path)
                    && tone.quality != ToneQuality::Unavailable
                {
                    r.responses[usize::from(path)].insert(raw.channel, tone.pct);
                }
            }
        }
        // 0: all results in; 1: more to come; 0xF: aborted.
        (procedure_done != 1).then(|| {
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
                match combine_paths(&slopes[..n], Combine::ResidualWeighted) {
                    Some(combined) => Ranged::Distance(combined, n),
                    None => Ranged::TooFewChannels(r.responses.iter().map(ChannelResponse::len).max().unwrap_or(0)),
                }
            } else {
                Ranged::Aborted
            };
            r.responses = [const { ChannelResponse::new() }; MAX_ANTENNA_PATHS as usize];
            (r.procedure, ranged)
        })
    });
    match done {
        Some((p, Ranged::Distance(d, n))) => info!(
            "distance {=u16}: {=f32} m from {=usize} path(s), coherence {=f32}, residual {=f32} rad, {=u8} pairs",
            p, d.distance_m, n, d.coherence, d.residual_rad, d.pairs
        ),
        Some((p, Ranged::TooFewChannels(channels))) => {
            warn!("distance {=u16}: too few channels ({=usize})", p, channels)
        }
        Some((p, Ranged::Aborted)) => warn!("distance {=u16}: procedure aborted", p),
        None => {}
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
        if is_initiator() { "I" } else { "R" },
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

/// The initiator's sequence, on an encrypted link: default settings, peer capabilities, config,
/// CS security, procedure parameters, then procedures until the link drops. `antennas` is how
/// many antennas this node may use for CS.
pub async fn initiate(stack: &Stack<'_, '_>, handle: ConnHandle, antennas: u8) -> Result<(), CsError> {
    while DONE.try_receive().is_ok() {}

    stack
        .command(LeCsSetDefaultSettings::new(
            handle,
            ROLE_INITIATOR,
            CS_SYNC_ANTENNA_REPETITIVE,
            MAX_TX_POWER_DBM,
        ))
        .await
        .map_err(|_| CsError::Command("set default settings"))?;

    stack
        .async_command(LeCsReadRemoteSupportedCapabilities::new(handle))
        .await
        .map_err(|_| CsError::Command("read remote capabilities"))?;
    completed("read remote capabilities", |d| match d {
        Done::RemoteCapabilities(s) => Some(s),
        _ => None,
    })
    .await?;

    stack
        .async_command(LeCsCreateConfig::new(
            handle,
            CONFIG_ID,
            1,    // create context: local and remote
            2,    // main mode 2: phase-based ranging
            0xFF, // no sub mode
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
    completed("create config", |d| match d {
        Done::Config(s) => Some(s),
        _ => None,
    })
    .await?;

    stack
        .async_command(LeCsSecurityEnableCommand::new(handle))
        .await
        .map_err(|_| CsError::Command("security enable"))?;
    completed("security enable", |d| match d {
        Done::Security(s) => Some(s),
        _ => None,
    })
    .await?;

    let peer = PEER_ANTENNAS.load(Ordering::Relaxed);
    let (antenna_config, peer_antennas) = tone_antenna_config(antennas, peer);
    info!(
        "CS antennas: {=u8} local, {=u8} peer, tone antenna config {=u8}",
        antennas, peer, antenna_config
    );
    // A procedure every 10 connection events may last up to 9 intervals, as in Nordic's sample.
    stack
        .command(LeCsSetProcedureParams::new(
            handle,
            CONFIG_ID,
            bt_hci::param::Duration::from_u16(ACL_INTERVAL_0625 * (PROCEDURE_INTERVAL - 1)),
            PROCEDURE_INTERVAL, // min, in connection events
            PROCEDURE_INTERVAL, // max
            0,                  // procedure count: until disabled
            ExtDuration::from_u32(SUBEVENT_LENGTH_US),
            ExtDuration::from_u32(SUBEVENT_LENGTH_US),
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
    completed("procedure enable", |d| match d {
        Done::Procedure(s) => Some(s),
        _ => None,
    })
    .await
}

/// Waits for the completion that `pick` accepts and checks its status. Other completions are
/// dropped; they belong to no command in flight.
async fn completed(what: &'static str, pick: fn(Done) -> Option<Status>) -> Result<(), CsError> {
    loop {
        let done = with_timeout(COMPLETION_TIMEOUT, DONE.receive())
            .await
            .map_err(|_| CsError::Timeout(what))?;
        if let Some(status) = pick(done) {
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
