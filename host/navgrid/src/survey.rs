//! The survey (FR-M6): every Link record the sinks mirror, kept per node pair as a history, and
//! the robust aggregate per pair the solver takes instead of the latest burst alone. One burst
//! gives a median with a spread of a metre or more indoors (log 2026-11-05); the median over the
//! bursts of an hour, with its standard error, is what a surveyed solution needs. `navgrid serve`
//! keeps the history in `survey.jsonl` in the fleet directory, one observation a line, and
//! `navgrid solve --survey` reads it.

use std::collections::{BTreeMap, VecDeque};
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

use anyhow::{Context, Result};
use navgrid_mesh::{Body, NodeId, Record, Version};
use serde::{Deserialize, Serialize};

/// One burst's result on a directed link, as its Link record carried it, and when the host saw it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Observation {
    /// Seconds since the Unix epoch when the host saw it; 0 from logs without a clock.
    pub at_s: u64,
    pub from: NodeId,
    pub to: NodeId,
    pub version: Version,
    pub distance_mm: u32,
    pub sigma_mm: u32,
    pub quality: u8,
    pub paths: u8,
    pub procedures: u16,
    pub failures: u16,
}

impl Observation {
    /// From a Link record, if `record` is one.
    pub fn from_record(record: &Record, at_s: u64) -> Option<Self> {
        let Body::Link(l) = &record.body else {
            return None;
        };
        Some(Self {
            at_s,
            from: record.origin,
            to: l.peer,
            version: record.version,
            distance_mm: l.distance_mm,
            sigma_mm: l.sigma_mm,
            quality: l.quality,
            paths: l.antenna_paths,
            procedures: l.procedures,
            failures: l.failures,
        })
    }

    /// The pair, the lower ID first.
    pub fn pair(&self) -> (NodeId, NodeId) {
        if self.from < self.to {
            (self.from, self.to)
        } else {
            (self.to, self.from)
        }
    }

    pub fn distance_m(&self) -> f32 {
        self.distance_mm as f32 / 1000.0
    }

    pub fn sigma_m(&self) -> f32 {
        self.sigma_mm as f32 / 1000.0
    }
}

/// Observations kept per pair; older ones go.
pub const KEEP_PER_PAIR: usize = 1000;

/// The smallest standard error believed for an aggregate, metres.
const SIGMA_FLOOR_M: f32 = 0.05;

/// The observations per pair, oldest first.
#[derive(Default)]
pub struct History {
    pairs: BTreeMap<(NodeId, NodeId), VecDeque<Observation>>,
}

impl History {
    /// Keeps `obs`, unless that burst (its origin and version) is there already.
    pub fn observe(&mut self, obs: Observation) -> bool {
        let list = self.pairs.entry(obs.pair()).or_default();
        if list
            .iter()
            .rev()
            .any(|o| o.from == obs.from && o.version == obs.version)
        {
            return false;
        }
        if list.len() >= KEEP_PER_PAIR {
            list.pop_front();
        }
        list.push_back(obs);
        true
    }

    /// Observations held, over every pair.
    pub fn len(&self) -> usize {
        self.pairs.values().map(VecDeque::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn pairs(&self) -> usize {
        self.pairs.len()
    }

    /// The history in `path` (one observation a line, as `append` writes them); empty if the
    /// file isn't there.
    pub fn load(path: &Path) -> Result<Self> {
        let mut history = Self::default();
        let Ok(file) = std::fs::File::open(path) else {
            return Ok(history);
        };
        for line in BufReader::new(file).lines() {
            let line = line.with_context(|| format!("reading {}", path.display()))?;
            if line.trim().is_empty() {
                continue;
            }
            let obs: Observation =
                serde_json::from_str(&line).with_context(|| format!("a line of {}", path.display()))?;
            history.observe(obs);
        }
        Ok(history)
    }

    /// One observation as a line of `file`.
    pub fn append(file: &mut impl Write, obs: &Observation) -> Result<()> {
        let line = serde_json::to_string(obs).context("encoding an observation")?;
        writeln!(file, "{line}").context("writing the survey")
    }

    /// The aggregate per pair over its observations since `since_s` (seconds since the epoch)
    /// with a quality of at least `min_quality`; pairs with none are left out.
    pub fn aggregate(&self, since_s: u64, min_quality: u8) -> Vec<PairStats> {
        self.pairs
            .iter()
            .filter_map(|(&(a, b), list)| {
                let taken: Vec<&Observation> = list
                    .iter()
                    .filter(|o| o.at_s >= since_s && o.quality >= min_quality)
                    .collect();
                PairStats::of(a, b, &taken)
            })
            .collect()
    }
}

/// What a pair's bursts agree on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairStats {
    pub a: NodeId,
    pub b: NodeId,
    /// Bursts taken, and how many each side ran.
    pub n: usize,
    pub from_a: usize,
    pub from_b: usize,
    pub median_m: f32,
    /// The spread between bursts, a MAD scaled to a sigma; NaN below three bursts.
    pub spread_m: f32,
    /// What the solver believes: the median's standard error from the spread, or, below three
    /// bursts, the best single burst's own sigma.
    pub sigma_m: f32,
    pub last_m: f32,
    pub last_at_s: u64,
    /// The best burst's quality, and the most antenna paths a burst had.
    pub quality: u8,
    pub paths: u8,
}

impl PairStats {
    /// The statistics of `taken`, which must all be of the pair `a`–`b`; none for no bursts.
    pub fn of(a: NodeId, b: NodeId, taken: &[&Observation]) -> Option<Self> {
        if taken.is_empty() {
            return None;
        }
        let mut distances: Vec<f32> = taken.iter().map(|o| o.distance_m()).collect();
        let median_m = median(&mut distances);
        let n = taken.len();
        let own_sigma = taken.iter().map(|o| o.sigma_m()).fold(f32::INFINITY, f32::min);
        let spread_m = if n >= 3 {
            let mut deviations: Vec<f32> = distances.iter().map(|d| (d - median_m).abs()).collect();
            1.4826 * median(&mut deviations)
        } else {
            f32::NAN
        };
        let sigma_m = if n >= 3 {
            (spread_m.max(own_sigma) / (n as f32).sqrt()).max(SIGMA_FLOOR_M)
        } else {
            own_sigma.max(SIGMA_FLOOR_M)
        };
        let last = taken.iter().max_by_key(|o| (o.at_s, o.version)).expect("some");
        Some(Self {
            a,
            b,
            n,
            from_a: taken.iter().filter(|o| o.from == a).count(),
            from_b: taken.iter().filter(|o| o.from == b).count(),
            median_m,
            spread_m,
            sigma_m,
            last_m: last.distance_m(),
            last_at_s: last.at_s,
            quality: taken.iter().map(|o| o.quality).max().unwrap_or(0),
            paths: taken.iter().map(|o| o.paths).max().unwrap_or(0),
        })
    }
}

/// The median of `values`, which it sorts.
fn median(values: &mut [f32]) -> f32 {
    values.sort_by(f32::total_cmp);
    let n = values.len();
    if n % 2 == 1 {
        values[n / 2]
    } else {
        (values[n / 2 - 1] + values[n / 2]) / 2.0
    }
}

/// Seconds since the Unix epoch, now.
pub fn now_s() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(from: u64, to: u64, counter: u32, at_s: u64, distance_mm: u32, sigma_mm: u32, quality: u8) -> Observation {
        Observation {
            at_s,
            from: NodeId(from),
            to: NodeId(to),
            version: Version { epoch: 1, counter },
            distance_mm,
            sigma_mm,
            quality,
            paths: 4,
            procedures: 10,
            failures: 0,
        }
    }

    #[test]
    fn a_pair_aggregates_both_directions_and_the_same_burst_once() {
        let mut h = History::default();
        assert!(h.observe(obs(1, 2, 1, 100, 5_000, 1_000, 200)));
        assert!(!h.observe(obs(1, 2, 1, 100, 5_000, 1_000, 200)), "the same burst again");
        assert!(h.observe(obs(2, 1, 1, 110, 5_200, 800, 220)));
        assert!(h.observe(obs(1, 2, 2, 120, 4_900, 1_200, 150)));
        assert!(h.observe(obs(1, 2, 3, 130, 9_000, 1_000, 50)), "a bad burst is kept");
        assert_eq!(h.len(), 4);
        let agg = h.aggregate(0, 100);
        assert_eq!(agg.len(), 1);
        let p = agg[0];
        assert_eq!((p.a, p.b, p.n, p.from_a, p.from_b), (NodeId(1), NodeId(2), 3, 2, 1));
        assert!((p.median_m - 5.0).abs() < 1e-6);
        assert!(p.spread_m > 0.1 && p.spread_m < 0.3, "{}", p.spread_m);
        // Three bursts: the standard error of the median, from the larger of the spread and a
        // burst's own sigma, over the square root of the count.
        assert!((p.sigma_m - 0.8 / 3f32.sqrt()).abs() < 1e-3, "{}", p.sigma_m);
        assert_eq!(p.last_m, 4.9);
        assert_eq!(p.quality, 220);
        // A window that leaves one burst: its own sigma.
        let late = h.aggregate(120, 100);
        assert_eq!(late[0].n, 1);
        assert_eq!(late[0].sigma_m, 1.2);
        assert!(late[0].spread_m.is_nan());
    }

    #[test]
    fn the_history_round_trips_through_its_file() {
        let dir = std::env::temp_dir().join(format!("navgrid-survey-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("survey.jsonl");
        let mut file = std::fs::File::create(&path).unwrap();
        History::append(&mut file, &obs(1, 2, 1, 100, 5_000, 1_000, 200)).unwrap();
        History::append(&mut file, &obs(3, 2, 1, 100, 7_000, 1_000, 200)).unwrap();
        let h = History::load(&path).unwrap();
        assert_eq!((h.len(), h.pairs()), (2, 2));
        assert!(History::load(&dir.join("none.jsonl")).unwrap().is_empty());
        std::fs::remove_dir_all(&dir).ok();
    }
}
