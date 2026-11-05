//! Barometry on the host (FR-M3): the pressures nodes report in Environment records, kept in
//! `baro.jsonl` in the fleet directory; each sensor's offset, calibrated with the nodes side by
//! side (or a new one beside one already calibrated) and kept in the registry; and the heights
//! the pressure differences give, which the survey's solve takes as height observations.
//!
//! Readings are compared minute by minute: nodes sample within the same minute, so the weather
//! cancels in the differences. A BME688's absolute error is up to ±0.6 hPa, about ±5 m; after
//! calibration the differences are good to a few centimetres plus the room's own gradients.

use std::collections::{BTreeMap, VecDeque};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, NodeId, Record, Version};
use serde::{Deserialize, Serialize};

use crate::fleet::{Fleet, FleetArgs, Registry, label_name};
use crate::survey::now_s;

/// One Environment record's pressure, as the host saw it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Reading {
    /// Seconds since the Unix epoch when the host saw it, less the record's age.
    pub at_s: u64,
    pub node: NodeId,
    pub version: Version,
    pub pressure_cpa: u32,
    pub temperature_cdeg: i16,
    pub samples: u8,
    /// The node lets its pressure fix heights.
    pub height: bool,
}

impl Reading {
    pub fn from_record(record: &Record, now_s: u64) -> Option<Self> {
        let Body::Environment(e) = &record.body else {
            return None;
        };
        if e.pressure_cpa == 0 || e.samples == 0 {
            return None;
        }
        Some(Self {
            at_s: now_s.saturating_sub(u64::from(record.age_ms) / 1000),
            node: record.origin,
            version: record.version,
            pressure_cpa: e.pressure_cpa,
            temperature_cdeg: e.temperature_cdeg,
            samples: e.samples,
            height: e.height,
        })
    }

    pub fn pressure_pa(&self) -> f64 {
        f64::from(self.pressure_cpa) / 100.0
    }

    pub fn temperature_c(&self) -> f64 {
        f64::from(self.temperature_cdeg) / 100.0
    }

    /// The minute it falls in.
    fn minute(&self) -> u64 {
        self.at_s / 60
    }
}

/// Readings kept per node.
const KEEP_PER_NODE: usize = 2000;

/// The readings, per node, oldest first.
#[derive(Default)]
pub struct Readings {
    nodes: BTreeMap<NodeId, VecDeque<Reading>>,
}

impl Readings {
    /// Keeps `r` unless that record is there already.
    pub fn add(&mut self, r: Reading) -> bool {
        let list = self.nodes.entry(r.node).or_default();
        if list.iter().rev().any(|x| x.version == r.version) {
            return false;
        }
        if list.len() >= KEEP_PER_NODE {
            list.pop_front();
        }
        list.push_back(r);
        true
    }

    pub fn len(&self) -> usize {
        self.nodes.values().map(VecDeque::len).sum()
    }

    /// The newest reading of each node.
    pub fn latest(&self) -> impl Iterator<Item = &Reading> {
        self.nodes.values().filter_map(|l| l.back())
    }

    pub fn load(path: &Path) -> Result<Self> {
        let mut readings = Self::default();
        let Ok(file) = std::fs::File::open(path) else {
            return Ok(readings);
        };
        for line in BufReader::new(file).lines() {
            let line = line.with_context(|| format!("reading {}", path.display()))?;
            if !line.trim().is_empty() {
                readings.add(serde_json::from_str(&line).with_context(|| format!("a line of {}", path.display()))?);
            }
        }
        Ok(readings)
    }

    pub fn append(file: &mut impl Write, r: &Reading) -> Result<()> {
        writeln!(file, "{}", serde_json::to_string(r)?).context("writing the barometry")
    }

    /// Per minute since `since_s`, each node's reading in it (the last, if it sent two).
    fn by_minute(&self, since_s: u64) -> BTreeMap<u64, BTreeMap<NodeId, Reading>> {
        let mut minutes: BTreeMap<u64, BTreeMap<NodeId, Reading>> = BTreeMap::new();
        for list in self.nodes.values() {
            for r in list.iter().filter(|r| r.at_s >= since_s) {
                minutes.entry(r.minute()).or_default().insert(r.node, *r);
            }
        }
        minutes
    }
}

/// A sensor's calibration: what it reads above the fleet's reference at the same height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Offset {
    pub node: NodeId,
    pub offset_pa: f64,
    /// Minutes it was taken over, and the spread of the per-minute differences (a MAD as sigma).
    pub minutes: usize,
    pub spread_pa: f64,
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    let n = values.len();
    if n % 2 == 1 {
        values[n / 2]
    } else {
        (values[n / 2 - 1] + values[n / 2]) / 2.0
    }
}

fn mad_sigma(values: &[f64], centre: f64) -> f64 {
    let mut d: Vec<f64> = values.iter().map(|v| (v - centre).abs()).collect();
    if d.is_empty() {
        return f64::NAN;
    }
    1.4826 * median(&mut d)
}

/// The offsets of `nodes`, which sat side by side since `since_s`: each one's median difference,
/// minute by minute, from a reference. With `against` (a node already calibrated, and its offset)
/// the reference is that node's corrected pressure; without it, the median of `nodes` in each
/// minute, which sets the fleet's reference level.
pub fn calibrate(
    readings: &Readings,
    nodes: &[NodeId],
    against: Option<(NodeId, f64)>,
    since_s: u64,
) -> Result<Vec<Offset>> {
    if nodes.is_empty() {
        bail!("no nodes to calibrate");
    }
    if against.is_none() && nodes.len() < 3 {
        bail!("calibrate at least three nodes together, or one against a calibrated node");
    }
    let mut diffs: BTreeMap<NodeId, Vec<f64>> = BTreeMap::new();
    for minute in readings.by_minute(since_s).values() {
        let reference = match against {
            Some((id, offset)) => match minute.get(&id) {
                Some(r) => r.pressure_pa() - offset,
                None => continue,
            },
            None => {
                let mut present: Vec<f64> = nodes
                    .iter()
                    .filter_map(|n| minute.get(n))
                    .map(Reading::pressure_pa)
                    .collect();
                if present.len() < nodes.len().min(3) {
                    continue;
                }
                median(&mut present)
            }
        };
        for n in nodes {
            if let Some(r) = minute.get(n) {
                diffs.entry(*n).or_default().push(r.pressure_pa() - reference);
            }
        }
    }
    let mut out = Vec::new();
    for n in nodes {
        let Some(d) = diffs.get(n).filter(|d| d.len() >= 3) else {
            bail!(
                "{:012x}: fewer than three minutes of readings beside the reference",
                n.0
            );
        };
        let mut sorted = d.clone();
        let offset_pa = median(&mut sorted);
        out.push(Offset {
            node: *n,
            offset_pa,
            minutes: d.len(),
            spread_pa: mad_sigma(d, offset_pa),
        });
    }
    Ok(out)
}

/// A node's height from barometry, relative to the reference node, metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroHeight {
    pub node: NodeId,
    pub above_reference_m: f64,
    pub sigma_m: f64,
    pub minutes: usize,
}

/// The heights of the calibrated nodes that allow it, relative to `reference`, from the minutes
/// since `since_s`: each the median of its per-minute height above the reference.
pub fn heights(
    readings: &Readings,
    offsets: &BTreeMap<NodeId, f64>,
    reference: NodeId,
    since_s: u64,
) -> Vec<BaroHeight> {
    let Some(&ref_offset) = offsets.get(&reference) else {
        return Vec::new();
    };
    let mut per: BTreeMap<NodeId, Vec<f64>> = BTreeMap::new();
    for minute in readings.by_minute(since_s).values() {
        let Some(r) = minute.get(&reference).filter(|r| r.height) else {
            continue;
        };
        let p_ref = r.pressure_pa() - ref_offset;
        for (id, x) in minute {
            let Some(off) = offsets.get(id) else {
                continue;
            };
            if !x.height {
                continue;
            }
            let t = (r.temperature_c() + x.temperature_c()) / 2.0;
            per.entry(*id)
                .or_default()
                .push(navgrid_baro::height_above_m(p_ref, x.pressure_pa() - off, t));
        }
    }
    per.into_iter()
        .filter(|(_, h)| !h.is_empty())
        .map(|(node, h)| {
            let mut sorted = h.clone();
            let m = median(&mut sorted);
            let spread = mad_sigma(&h, m);
            let n = h.len() as f64;
            // The calibration's own error and the room's gradients: 8 cm floor.
            let sigma = (0.08f64.powi(2) + (if spread.is_finite() { spread } else { 0.3 }).powi(2) / n).sqrt();
            BaroHeight {
                node,
                above_reference_m: m,
                sigma_m: sigma,
                minutes: h.len(),
            }
        })
        .collect()
}

/// The offsets the registry holds, pascals.
pub fn offsets(registry: &Registry) -> BTreeMap<NodeId, f64> {
    registry
        .devices
        .iter()
        .filter_map(|d| d.baro_offset_cpa.map(|o| (d.id, f64::from(o) / 100.0)))
        .collect()
}

/// Writes `found` into the registry, read afresh.
pub fn save_offsets(fleet: &Fleet, found: &[Offset]) -> Result<Registry> {
    let mut registry = fleet.registry()?;
    let today = crate::fleet::today();
    for o in found {
        let d = registry
            .devices
            .iter_mut()
            .find(|d| d.id == o.node)
            .with_context(|| format!("{:012x} is not in the registry", o.node.0))?;
        d.baro_offset_cpa = Some((o.offset_pa * 100.0).round() as i32);
        d.baro_calibrated = Some(today.clone());
    }
    fleet.save(&registry)?;
    Ok(registry)
}

pub fn readings_path(dir: &Path) -> PathBuf {
    dir.join("baro.jsonl")
}

#[derive(clap::Args)]
pub struct BaroArgs {
    #[command(subcommand)]
    cmd: BaroCmd,
    #[command(flatten)]
    fleet: FleetArgs,
}

#[derive(clap::Subcommand)]
enum BaroCmd {
    /// Calibrate barometers that sit side by side: from the readings `navgrid serve` kept over the
    /// last `--minutes`. Three or more together set the fleet's reference; with `--against` a
    /// calibrated node, any number join it.
    Calibrate {
        /// Fleet numbers (`N001 N002 …`) or `all`.
        #[arg(required = true)]
        nodes: Vec<String>,
        #[arg(long)]
        against: Option<String>,
        #[arg(long, default_value_t = 10)]
        minutes: u64,
    },
    /// The latest pressures, offsets and heights relative to `--reference`.
    Show {
        #[arg(long)]
        reference: Option<String>,
        #[arg(long, default_value_t = 30)]
        minutes: u64,
    },
}

pub fn run(args: &BaroArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let registry = fleet.registry()?;
    let dir = args.fleet.dir()?;
    let readings = Readings::load(&readings_path(&dir))?;
    let id_of = |s: &str| crate::control::parse_target(s, &registry);
    let name = |id: NodeId| crate::mesh_view::node_name(id, Some(&registry));
    match &args.cmd {
        BaroCmd::Calibrate {
            nodes,
            against,
            minutes,
        } => {
            let ids: Vec<NodeId> = if nodes.len() == 1 && nodes[0] == "all" {
                readings.latest().map(|r| r.node).collect()
            } else {
                nodes.iter().map(|n| id_of(n)).collect::<Result<_>>()?
            };
            let against = match against {
                Some(a) => {
                    let id = id_of(a)?;
                    let off = offsets(&registry)
                        .get(&id)
                        .copied()
                        .with_context(|| format!("{a} has no calibration yet"))?;
                    Some((id, off))
                }
                None => None,
            };
            let found = calibrate(&readings, &ids, against, now_s().saturating_sub(minutes * 60))?;
            save_offsets(&fleet, &found)?;
            for o in &found {
                println!(
                    "{:<6} offset {:+7.2} Pa over {} minutes, spread {:.2} Pa",
                    name(o.node),
                    o.offset_pa,
                    o.minutes,
                    o.spread_pa
                );
            }
            println!("kept in the registry");
        }
        BaroCmd::Show { reference, minutes } => {
            let offs = offsets(&registry);
            for r in readings.latest() {
                println!(
                    "{:<6} {:10.2} Pa  {:5.2} °C  mean of {}{}  offset {}",
                    name(r.node),
                    r.pressure_pa(),
                    r.temperature_c(),
                    r.samples,
                    if r.height { "" } else { ", no heights" },
                    offs.get(&r.node).map_or("none".into(), |o| format!("{o:+.2} Pa"))
                );
            }
            if let Some(reference) = reference {
                let id = id_of(reference)?;
                println!();
                for h in heights(&readings, &offs, id, now_s().saturating_sub(minutes * 60)) {
                    println!(
                        "{:<6} {:+6.2} m ± {:.2} above {} over {} minutes",
                        name(h.node),
                        h.above_reference_m,
                        h.sigma_m,
                        label_name(registry.by_id(id).map_or(0, |d| d.number)),
                        h.minutes
                    );
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reading(node: u64, minute: u64, pressure_pa: f64, counter: u32) -> Reading {
        Reading {
            at_s: minute * 60 + 5,
            node: NodeId(node),
            version: Version { epoch: 1, counter },
            pressure_cpa: (pressure_pa * 100.0) as u32,
            temperature_cdeg: 2200,
            samples: 6,
            height: true,
        }
    }

    #[test]
    fn side_by_side_calibration_then_heights_cancel_the_weather() {
        let mut r = Readings::default();
        // Three sensors side by side, offsets +5, -3 and 0 Pa, while the weather falls 1 Pa a minute.
        for m in 0..10u64 {
            let weather = 101_300.0 - m as f64;
            r.add(reading(1, m, weather + 5.0, m as u32));
            r.add(reading(2, m, weather - 3.0, m as u32));
            r.add(reading(3, m, weather, m as u32));
        }
        let found = calibrate(&r, &[NodeId(1), NodeId(2), NodeId(3)], None, 0).unwrap();
        let off: BTreeMap<NodeId, f64> = found.iter().map(|o| (o.node, o.offset_pa)).collect();
        assert!((off[&NodeId(1)] - 5.0).abs() < 0.01 && (off[&NodeId(2)] + 3.0).abs() < 0.01);
        // A fourth joins against node 3.
        for m in 10..15u64 {
            let weather = 101_300.0 - m as f64;
            r.add(reading(3, m, weather, m as u32));
            r.add(reading(4, m, weather + 7.5, m as u32));
        }
        let joined = calibrate(&r, &[NodeId(4)], Some((NodeId(3), off[&NodeId(3)])), 600).unwrap();
        assert!((joined[0].offset_pa - 7.5).abs() < 0.01);
        let mut offsets = off.clone();
        offsets.insert(NodeId(4), joined[0].offset_pa);
        // Spread out: node 1 goes 2 m up, node 4 1 m down; the weather keeps falling.
        for m in 20..30u64 {
            let weather = 101_300.0 - m as f64;
            r.add(reading(3, m, weather, m as u32));
            r.add(reading(1, m, weather + 5.0 - 2.0 * 11.8, m as u32));
            r.add(reading(4, m, weather + 7.5 + 11.8, m as u32));
        }
        let h: BTreeMap<NodeId, BaroHeight> = heights(&r, &offsets, NodeId(3), 20 * 60)
            .into_iter()
            .map(|h| (h.node, h))
            .collect();
        assert!(h[&NodeId(3)].above_reference_m.abs() < 1e-6);
        assert!(
            (h[&NodeId(1)].above_reference_m - 2.0).abs() < 0.05,
            "{:?}",
            h[&NodeId(1)]
        );
        assert!(
            (h[&NodeId(4)].above_reference_m + 1.0).abs() < 0.05,
            "{:?}",
            h[&NodeId(4)]
        );
        assert!(h[&NodeId(1)].sigma_m < 0.15);
    }

    #[test]
    fn calibration_needs_company_and_minutes() {
        let r = Readings::default();
        assert!(calibrate(&r, &[NodeId(1), NodeId(2)], None, 0).is_err());
        assert!(calibrate(&r, &[NodeId(1), NodeId(2), NodeId(3)], None, 0).is_err());
    }
}
