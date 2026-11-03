//! `navgrid monitor` and `navgrid decode`: frames from a sink's serial port or a capture file, as
//! text or JSON lines. With the fleet registry, nodes show by fleet number, and Health reports get
//! a battery level and, after an hour of readings, an estimate of the days until the cell is low
//! (architecture §13).

use std::collections::HashMap;
use std::io::Read;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context, Result};
use navgrid_proto::{BATTERY_CRITICAL_MV, BATTERY_GOOD_MV, BATTERY_LOW_MV, Firmware, Health, Message, NodeId};

use crate::calibration::Calibration;
use crate::fleet::{Registry, label_name};

/// A day-count estimate needs readings spanning at least this long.
const TREND_MIN_SPAN_S: f64 = 3600.0;
const TREND_WINDOW_S: f64 = 24.0 * 3600.0;

pub struct View {
    cal: Option<Calibration>,
    registry: Option<Registry>,
    batteries: HashMap<NodeId, Vec<(f64, f64)>>,
    start: Instant,
}

impl View {
    pub fn new(cal: Option<Calibration>, registry: Option<Registry>) -> Self {
        Self {
            cal,
            registry,
            batteries: HashMap::new(),
            start: Instant::now(),
        }
    }

    /// `N042 ce36dfcc4c2d`, or just the ID for a device the registry doesn't know.
    fn name(&self, id: NodeId) -> String {
        match self.registry.as_ref().and_then(|r| r.by_id(id)) {
            Some(d) => format!("{} {:012x}", label_name(d.number), id.0),
            None => format!("{:012x}", id.0),
        }
    }

    /// One COBS frame (without its `0x00` terminator) as a line of text.
    pub fn render(&mut self, frame: &[u8], json: bool) -> Result<String> {
        let msg = navgrid_proto::decode(&mut frame.to_vec())?;
        let now = self.start.elapsed().as_secs_f64();
        let calibrated = match (&msg, &self.cal) {
            (Message::Range(r), Some(cal)) => cal.apply(r),
            _ => None,
        };
        let battery = match &msg {
            Message::Health(h) => Some(self.battery(h, now)),
            _ => None,
        };
        let node = match &msg {
            Message::Hello { node, .. } => Some(*node),
            Message::Range(r) => Some(r.initiator),
            Message::Position(p) => Some(p.node),
            Message::Health(h) => Some(h.node),
            Message::Environment(e) => Some(e.node),
            Message::Link(l) => Some(l.node),
            Message::Update(_) => None,
        };
        if json {
            let mut v = serde_json::to_value(msg)?;
            if let Some(o) = v.as_object_mut() {
                if let Some(mm) = calibrated {
                    o.insert("calibrated_mm".into(), mm.into());
                }
                let number = node
                    .and_then(|n| self.registry.as_ref()?.by_id(n))
                    .map(|d| label_name(d.number));
                if let Some(number) = number {
                    o.insert("number".into(), number.into());
                }
                if let Some(b) = battery {
                    o.insert("battery_level".into(), b.level.into());
                    if let Some(days) = b.days_left {
                        o.insert("days_to_low".into(), days.into());
                    }
                }
            }
            return Ok(v.to_string());
        }
        Ok(match (&msg, battery) {
            (Message::Health(h), Some(b)) => {
                let days = b.days_left.map_or(String::new(), |d| format!(", ≈ {d:.0} days to low"));
                format!(
                    "{} health: battery {} mV {}{days}, {:.2} °C, up {} s, firmware {}{}, {} queued",
                    self.name(h.node),
                    h.battery_mv,
                    b.level,
                    f64::from(h.temperature_cdeg) / 100.0,
                    h.uptime_s,
                    h.firmware,
                    self.firmware_note(h.node, &h.firmware),
                    h.queued
                )
            }
            (
                Message::Hello {
                    node,
                    role,
                    protocol,
                    firmware,
                    antennas,
                },
                _,
            ) => format!(
                "{} hello: {role:?}, protocol {protocol}, firmware {firmware}{}, {antennas} antenna(s)",
                self.name(*node),
                self.firmware_note(*node, firmware)
            ),
            _ => {
                let prefix = node.map_or(String::new(), |n| format!("{}: ", self.name(n)));
                match calibrated {
                    Some(mm) => format!("{prefix}{msg:?} → calibrated {mm} mm"),
                    None => format!("{prefix}{msg:?}"),
                }
            }
        })
    }

    /// ` (registry: …)` when the device reports another build than the one last flashed.
    fn firmware_note(&self, id: NodeId, firmware: &Firmware) -> String {
        let flashed = self
            .registry
            .as_ref()
            .and_then(|r| r.by_id(id))
            .and_then(|d| d.firmware.as_ref());
        match flashed {
            Some(f) if *f != firmware.to_string() => format!(" (registry: {f})"),
            _ => String::new(),
        }
    }

    fn battery(&mut self, h: &Health, now: f64) -> Battery {
        let level = level(h.battery_mv);
        if h.battery_mv == 0 {
            return Battery { level, days_left: None };
        }
        let readings = self.batteries.entry(h.node).or_default();
        readings.push((now, f64::from(h.battery_mv)));
        readings.retain(|&(t, _)| now - t <= TREND_WINDOW_S);
        Battery {
            level,
            days_left: days_to_low(readings),
        }
    }
}

struct Battery {
    level: &'static str,
    days_left: Option<f64>,
}

/// The status display's battery levels (architecture §13), with the shared thresholds.
fn level(mv: u16) -> &'static str {
    match mv {
        0 => "unknown",
        mv if mv >= BATTERY_GOOD_MV => "good",
        mv if mv >= BATTERY_LOW_MV => "fine",
        mv if mv >= BATTERY_CRITICAL_MV => "low",
        _ => "critical",
    }
}

/// Days until the least-squares line through `readings` (seconds, millivolts) reaches the low
/// threshold, if they span an hour and fall.
fn days_to_low(readings: &[(f64, f64)]) -> Option<f64> {
    let (first, last) = (readings.first()?, readings.last()?);
    if readings.len() < 5 || last.0 - first.0 < TREND_MIN_SPAN_S {
        return None;
    }
    let n = readings.len() as f64;
    let (mt, mv) = readings
        .iter()
        .fold((0.0, 0.0), |(a, b), &(t, v)| (a + t / n, b + v / n));
    let (sxy, sxx) = readings.iter().fold((0.0, 0.0), |(a, b), &(t, v)| {
        (a + (t - mt) * (v - mv), b + (t - mt) * (t - mt))
    });
    let slope = sxy / sxx;
    let now_mv = mv + slope * (last.0 - mt);
    (slope < 0.0 && now_mv > f64::from(BATTERY_LOW_MV))
        .then(|| (now_mv - f64::from(BATTERY_LOW_MV)) / -slope / 86_400.0)
}

pub fn monitor(path: &str, baud: u32, json: bool, mut view: View) -> Result<()> {
    let mut port = serialport::new(path, baud)
        .timeout(std::time::Duration::from_secs(1))
        .open()
        .with_context(|| format!("opening {path}"))?;
    let mut pending = Vec::new();
    let mut buf = [0u8; 256];
    loop {
        match port.read(&mut buf) {
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e).with_context(|| format!("reading {path}")),
        }
        for frame in take_frames(&mut pending) {
            match view.render(&frame, json) {
                Ok(line) => println!("{line}"),
                Err(e) => eprintln!("bad frame ({e}): {frame:02x?}"),
            }
        }
    }
}

/// Removes the complete frames (those ended by `0x00`) from the front of `pending`, without their
/// terminators. A partial frame stays for the next read; empty frames are skipped.
pub fn take_frames(pending: &mut Vec<u8>) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    while let Some(end) = pending.iter().position(|&b| b == 0) {
        let frame: Vec<u8> = pending.drain(..=end).take(end).collect();
        if !frame.is_empty() {
            frames.push(frame);
        }
    }
    frames
}

pub fn decode(path: Option<PathBuf>, json: bool) -> Result<()> {
    let mut data = Vec::new();
    match path {
        Some(p) => data = std::fs::read(&p).with_context(|| format!("reading {}", p.display()))?,
        None => _ = std::io::stdin().read_to_end(&mut data)?,
    }
    let mut view = View::new(None, None);
    for frame in data.split(|&b| b == 0).filter(|f| !f.is_empty()) {
        match view.render(frame, json) {
            Ok(line) => println!("{line}"),
            Err(e) => eprintln!("bad frame ({e}): {frame:02x?}"),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use navgrid_proto::Environment;

    use super::*;
    use crate::fleet::{Device, Status};

    fn frame(msg: &Message) -> Vec<u8> {
        let mut buf = [0u8; navgrid_proto::MAX_FRAME_LEN];
        let frame = navgrid_proto::encode(msg, &mut buf).unwrap();
        frame[..frame.len() - 1].to_vec()
    }

    #[test]
    fn frames_split_across_reads() {
        let mut pending = vec![0, 1, 2, 0, 3];
        assert_eq!(take_frames(&mut pending), vec![vec![1, 2]]);
        assert_eq!(pending, vec![3]);
        pending.extend_from_slice(&[4, 0, 0, 5]);
        assert_eq!(take_frames(&mut pending), vec![vec![3, 4]]);
        assert_eq!(pending, vec![5]);
    }

    #[test]
    fn renders_environment_as_json() {
        let msg = Message::Environment(Environment {
            node: NodeId(0xc0ffee),
            uptime_s: 42,
            temperature_cdeg: 2150,
            humidity_cpct: 4100,
            pressure_cpa: 10_132_500,
            gas_ohm: None,
        });
        let line = View::new(None, None).render(&frame(&msg), true).unwrap();
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value["Environment"]["pressure_cpa"], 10_132_500);
        assert_eq!(value["Environment"]["gas_ohm"], serde_json::Value::Null);
    }

    #[test]
    fn health_shows_fleet_number_and_battery_level() {
        let id = NodeId(0xce36_dfcc_4c2d);
        let registry = Registry {
            devices: vec![Device {
                number: 42,
                id,
                role: navgrid_proto::Role::Node,
                enrolled: "2026-11-02".into(),
                status: Status::Enrolled,
                firmware: Some("0.1.0+00000001".into()),
                flashed: Some("2026-11-02".into()),
            }],
        };
        let msg = Message::Health(Health {
            node: id,
            uptime_s: 60,
            battery_mv: 2700,
            temperature_cdeg: 2475,
            firmware: navgrid_proto::Firmware::new(0, 1, 0, 0x1950_d45d, false),
            queued: 0,
        });
        let mut view = View::new(None, Some(registry));
        let line = view.render(&frame(&msg), false).unwrap();
        assert!(
            line.starts_with("N042 ce36dfcc4c2d health: battery 2700 mV fine"),
            "{line}"
        );
        assert!(
            line.contains("firmware 0.1.0+1950d45d (registry: 0.1.0+00000001)"),
            "{line}"
        );
        let json: serde_json::Value = serde_json::from_str(&view.render(&frame(&msg), true).unwrap()).unwrap();
        assert_eq!(
            (json["number"].as_str(), json["battery_level"].as_str()),
            (Some("N042"), Some("fine"))
        );
    }

    #[test]
    fn days_to_low_from_a_falling_trend() {
        // 10 mV an hour from 2900 mV: after 2 h it reads 2880 mV, 280 mV above low, so 28 hours.
        let readings: Vec<(f64, f64)> = (0..13)
            .map(|i| (i as f64 * 600.0, 2900.0 - i as f64 * 10.0 / 6.0))
            .collect();
        let days = days_to_low(&readings).unwrap();
        assert!((days - 28.0 / 24.0).abs() < 0.01, "{days}");
        assert!(days_to_low(&readings[..3]).is_none());
        let flat: Vec<(f64, f64)> = (0..13).map(|i| (i as f64 * 600.0, 3300.0)).collect();
        assert!(days_to_low(&flat).is_none());
    }
}
