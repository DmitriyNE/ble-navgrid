//! Range calibration on the host. Devices report raw distances; a calibration file holds one
//! constant offset per (initiator, reflector, antenna paths, method), fitted from ranges recorded
//! at measured distances. The antenna path count stands in for the CS timing configuration, which
//! moves the offset too, so changing the firmware's CS timing calls for a new fit.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use navgrid_proto::{Message, NodeId, Range, RangeMethod};
use serde::{Deserialize, Serialize};

#[derive(clap::Args)]
pub struct CalibrateArgs {
    /// Calibration file to write. Existing entries for other keys are kept.
    #[arg(long)]
    out: PathBuf,
    /// Captures with the true distance in metres: `monitor --json` output, as `FILE=METRES`.
    #[arg(required = true, value_parser = parse_capture)]
    captures: Vec<(PathBuf, f64)>,
}

fn parse_capture(s: &str) -> Result<(PathBuf, f64), String> {
    let (path, metres) = s.rsplit_once('=').ok_or("expected FILE=METRES")?;
    let metres: f64 = metres.parse().map_err(|_| format!("bad distance {metres:?}"))?;
    Ok((PathBuf::from(path), metres))
}

/// What an offset applies to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Key {
    pub initiator: NodeId,
    pub reflector: NodeId,
    pub antenna_paths: u8,
    pub method: Method,
}

/// [`RangeMethod`] with an ordering, for keys.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Method {
    PhaseSlope,
    Rtt,
    Fused,
}

impl From<RangeMethod> for Method {
    fn from(m: RangeMethod) -> Self {
        match m {
            RangeMethod::PhaseSlope => Self::PhaseSlope,
            RangeMethod::Rtt => Self::Rtt,
            RangeMethod::Fused => Self::Fused,
        }
    }
}

impl Key {
    pub fn of(r: &Range) -> Self {
        Self {
            initiator: r.initiator,
            reflector: r.reflector,
            antenna_paths: r.antenna_paths,
            method: r.method.into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    #[serde(flatten)]
    pub key: Key,
    /// Subtracted from reported distances.
    pub offset_mm: i32,
    /// Ranges the offset was fitted from, and the p10–p90 spread of their errors.
    pub samples: usize,
    pub spread_mm: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    pub entries: Vec<Entry>,
}

impl Calibration {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        let text = serde_json::to_string_pretty(self)?;
        std::fs::write(path, text + "\n").with_context(|| format!("writing {}", path.display()))
    }

    /// The calibrated distance, or `None` when no entry matches the range's key.
    pub fn apply(&self, r: &Range) -> Option<i64> {
        let key = Key::of(r);
        let e = self.entries.iter().find(|e| e.key == key)?;
        Some(i64::from(r.distance_mm) - i64::from(e.offset_mm))
    }

    /// Replaces or adds the entries in `fitted`.
    pub fn merge(&mut self, fitted: Calibration) {
        for e in fitted.entries {
            match self.entries.iter_mut().find(|o| o.key == e.key) {
                Some(old) => *old = e,
                None => self.entries.push(e),
            }
        }
        self.entries.sort_by_key(|e| e.key);
    }
}

/// One offset per key: the median of (reported − true) over that key's ranges.
pub fn fit(samples: &[(Range, f64)]) -> Calibration {
    let mut errors: BTreeMap<Key, Vec<i64>> = BTreeMap::new();
    for (r, truth_m) in samples {
        let error = i64::from(r.distance_mm) - (truth_m * 1000.0).round() as i64;
        errors.entry(Key::of(r)).or_default().push(error);
    }
    let entries = errors
        .into_iter()
        .map(|(key, mut e)| {
            e.sort_unstable();
            let q = |f: f64| e[((e.len() - 1) as f64 * f).round() as usize];
            Entry {
                key,
                offset_mm: q(0.5) as i32,
                samples: e.len(),
                spread_mm: (q(0.9) - q(0.1)) as u32,
            }
        })
        .collect();
    Calibration { entries }
}

/// The ranges in a `monitor --json` (or `decode --json`) capture. Other messages are skipped.
pub fn ranges(path: &Path) -> Result<Vec<Range>> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty()) {
        // `monitor --calibration` adds a `calibrated_mm` field next to the message.
        let parsed = serde_json::from_str::<serde_json::Value>(line).and_then(|mut v| {
            if let Some(o) = v.as_object_mut() {
                o.remove("calibrated_mm");
            }
            serde_json::from_value::<Message>(v)
        });
        let msg = match parsed {
            Ok(msg) => msg,
            Err(e) => bail!("{}:{}: {e}", path.display(), n + 1),
        };
        if let Message::Range(r) = msg {
            out.push(r);
        }
    }
    Ok(out)
}

pub fn run(args: &CalibrateArgs) -> Result<()> {
    let mut samples = Vec::new();
    for (path, truth) in &args.captures {
        let r = ranges(path)?;
        if r.is_empty() {
            bail!("{}: no ranges", path.display());
        }
        samples.extend(r.into_iter().map(|r| (r, *truth)));
    }
    let fitted = fit(&samples);
    for e in &fitted.entries {
        println!(
            "{:012x} → {:012x}, {} path(s), {:?}: offset {} mm from {} ranges (p10–p90 spread {} mm)",
            e.key.initiator.0,
            e.key.reflector.0,
            e.key.antenna_paths,
            e.key.method,
            e.offset_mm,
            e.samples,
            e.spread_mm
        );
    }
    let mut cal = if args.out.exists() {
        Calibration::load(&args.out)?
    } else {
        Calibration::default()
    };
    cal.merge(fitted);
    cal.save(&args.out)?;
    println!("wrote {} ({} entries)", args.out.display(), cal.entries.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn range(initiator: u64, paths: u8, distance_mm: u32) -> Range {
        Range {
            initiator: NodeId(initiator),
            reflector: NodeId(2),
            uptime_ms: 0,
            distance_mm,
            sigma_mm: 100,
            quality: 255,
            method: RangeMethod::PhaseSlope,
            steps: 70,
            antenna_paths: paths,
        }
    }

    #[test]
    fn fits_one_offset_per_key_and_applies_it() {
        let samples = [
            (range(1, 4, 3300), 0.9),
            (range(1, 4, 3400), 0.9),
            (range(1, 4, 3350), 0.9),
            (range(1, 1, 3600), 0.9),
        ];
        let cal = fit(&samples);
        assert_eq!(cal.entries.len(), 2);
        let four = cal.entries.iter().find(|e| e.key.antenna_paths == 4).unwrap();
        assert_eq!((four.offset_mm, four.samples), (2450, 3));
        assert_eq!(cal.apply(&range(1, 4, 3350)), Some(900));
        assert_eq!(cal.apply(&range(1, 1, 3600)), Some(900));
        // Another initiator has no entry.
        assert_eq!(cal.apply(&range(3, 4, 3350)), None);
    }

    #[test]
    fn merge_replaces_matching_keys() {
        let mut cal = fit(&[(range(1, 4, 3300), 0.9)]);
        cal.merge(fit(&[(range(1, 4, 3000), 0.9), (range(1, 1, 2000), 1.0)]));
        assert_eq!(cal.entries.len(), 2);
        assert_eq!(cal.apply(&range(1, 4, 3000)), Some(900));
    }

    #[test]
    fn round_trips_through_json() {
        let cal = fit(&[(range(1, 4, 3300), 0.9)]);
        let back: Calibration = serde_json::from_str(&serde_json::to_string(&cal).unwrap()).unwrap();
        assert_eq!(back, cal);
    }
}
