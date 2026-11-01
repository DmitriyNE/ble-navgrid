//! Channel Sounding bring-up (phase P0).
//!
//! One board initiates and one reflects. Both log their raw step data over RTT, so the host can
//! combine the two sides offline; no RAS or inline PCT transfer is needed yet. The command order
//! and parameter values follow Nordic's NCS v3.4.1 `ras_initiator` sample: mode 2 with three
//! mode-0 steps, all 72 channels, a 1×1 antenna configuration, 2M PHY, and a procedure every 10
//! connection events at a 20 ms interval.
//!
//! Build-time role (no provisioning yet): `NAVGRID_ROLE=initiator` with `NAVGRID_PEER` set to the
//! reflector's node ID as it logs it at boot (12 hex digits). Anything else builds a reflector.

use bt_hci::cmd::le::{
    LeCsCreateConfig, LeCsProcedureEnable, LeCsReadRemoteSupportedCapabilities, LeCsSecurityEnableCommand,
    LeCsSetDefaultSettings, LeCsSetProcedureParams,
};
use bt_hci::event::le::{
    LeCsConfigComplete, LeCsProcedureEnableComplete, LeCsReadRemoteSupportedCapabilitiesComplete,
    LeCsSecurityEnableComplete, LeCsSubeventResult, LeCsSubeventResultContinue,
};
use bt_hci::param::{ConnHandle, ExtDuration, LeCsSubeventStepData, PhyKind, Status};
use defmt::{Format, info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, with_timeout};
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
/// How long to wait for an asynchronous CS command to complete.
const COMPLETION_TIMEOUT: Duration = Duration::from_secs(5);

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
        let _ = DONE.try_send(Done::RemoteCapabilities(e.status));
    }

    fn on_cs_config_complete(&self, e: &LeCsConfigComplete) {
        info!("CS config {} action {}: {}", e.config_id, e.action, e.status);
        let _ = DONE.try_send(Done::Config(e.status));
    }

    fn on_cs_security_enable_complete(&self, e: &LeCsSecurityEnableComplete) {
        info!("CS security: {}", e.status);
        let _ = DONE.try_send(Done::Security(e.status));
    }

    fn on_cs_procedure_enable_complete(&self, e: &LeCsProcedureEnableComplete) {
        info!(
            "CS procedures {}: every {} events, {} subevent(s) per event, tx {} dBm: {}",
            if e.state { "on" } else { "off" },
            e.procedure_interval,
            e.subevents_per_event,
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
            e.num_antenna_paths,
            &e.steps,
        );
    }

    fn on_cs_subevent_result_continue(&self, e: &LeCsSubeventResultContinue<'_>) {
        // A continuation carries no procedure counter or frequency compensation of its own; the
        // host takes them from the preceding CSR line.
        log_steps("CSC", 0, 0, 0, e.num_antenna_paths, &e.steps);
    }
}

/// Logs one subevent as a single line, for the host to parse:
/// `<tag> <role> <procedure counter> <frequency compensation> <reference power> <antenna paths>
/// <steps> <step bytes>`. The step bytes keep the HCI layout (mode, channel, length, data, per
/// step), so the host parses them exactly as it would a controller event.
fn log_steps(tag: &str, procedure: u16, freq_comp: u16, ref_power: i8, paths: u8, steps: &LeCsSubeventStepData<'_>) {
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
        "{=str} {=str} {=u16} {=u16} {=i8} {=u8} {=usize} {=[u8]:02x}",
        tag,
        if is_initiator() { "I" } else { "R" },
        procedure,
        freq_comp,
        ref_power,
        paths,
        steps.len(),
        &buf[..len]
    );
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
/// CS security, procedure parameters, then procedures until the link drops.
pub async fn initiate(stack: &Stack<'_, '_>, handle: ConnHandle) -> Result<(), CsError> {
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
            1, // channel map repetition
            0, // channel selection algorithm #3b
            0, // ch3c shape: hat (unused by #3b)
            2, // ch3c jump (unused by #3b)
            0, // cs_enhancements (Core 6.3): no inline PCT transfer
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

    // At a 20 ms connection interval (32 × 0.625 ms) and a procedure every 10 events, a procedure
    // may last up to 9 intervals, as in Nordic's sample.
    stack
        .command(LeCsSetProcedureParams::new(
            handle,
            CONFIG_ID,
            bt_hci::param::Duration::from_u16(32 * 9),
            10, // min procedure interval, connection events
            10, // max procedure interval
            0,  // procedure count: until disabled
            ExtDuration::from_u32(16_000),
            ExtDuration::from_u32(16_000),
            0, // antenna configuration index 0: 1×1
            PhyKind::Le2M,
            i8::MIN, // tx power delta: no recommendation
            1,       // preferred peer antenna: first
            0xFF,    // SNR control, initiator: not used
            0xFF,    // SNR control, reflector: not used
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
