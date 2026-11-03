//! `navgrid tones`: the host estimators over the raw tones initiators send through the sink
//! (`Message::Tones`, one antenna path a second, three chunks a procedure), read from
//! `navgrid monitor --json` logs. For each (node, procedure, path) with enough channels it prints
//! the phase slope, the first-path estimate and the delay profile's peaks, then medians per node.

use std::collections::BTreeMap;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use navgrid_proto::{NodeId, Tones};
use navgrid_ranging::{
    ChannelResponse, FIRST_PATH_THRESHOLD, Peak, Window, first_path, phase_slope, profile_peaks, steps,
};
use serde_json::{Value, json};

#[derive(clap::Args)]
pub struct TonesArgs {
    /// `navgrid monitor --json` logs.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// Fewest channels for an estimate.
    #[arg(long, default_value_t = 40)]
    min_channels: usize,
    /// Peaks below this fraction of the strongest are not listed.
    #[arg(long, default_value_t = 0.05)]
    min_peak: f32,
    /// One JSON object per procedure and path.
    #[arg(long)]
    json: bool,
}

struct Estimate {
    slope: Option<f32>,
    coherence: Option<f32>,
    first: Option<(f32, f32, u8)>,
    peaks: Vec<Peak>,
}

pub fn run(args: &TonesArgs) -> Result<()> {
    let mut chunks: BTreeMap<(NodeId, u16, u8), (u8, Vec<Tones>)> = BTreeMap::new();
    let mut names: BTreeMap<NodeId, String> = BTreeMap::new();
    for path in &args.files {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        for line in text.lines() {
            let Ok(v) = serde_json::from_str::<Value>(line) else {
                continue;
            };
            let Some(t) = v.get("Tones") else {
                continue;
            };
            let t: Tones = serde_json::from_value(t.clone()).context("a Tones record")?;
            if let Some(n) = v.get("number").and_then(Value::as_str) {
                names.insert(t.node, n.to_string());
            }
            let e = chunks
                .entry((t.node, t.procedure, t.path))
                .or_insert((t.paths, Vec::new()));
            e.1.push(t);
        }
    }
    if chunks.is_empty() {
        bail!("no Tones records found");
    }
    let name = |node: &NodeId| names.get(node).cloned().unwrap_or_else(|| format!("{node:?}"));
    let mut per_node: BTreeMap<NodeId, Vec<Estimate>> = BTreeMap::new();
    for ((node, procedure, path), (paths, parts)) in &chunks {
        let mut resp = ChannelResponse::new();
        for t in parts {
            for (i, b) in t.pct.chunks_exact(3).enumerate() {
                if b != [0, 0, 0] {
                    resp.insert(t.first + i as u8, steps::pct([b[0], b[1], b[2]]));
                }
            }
        }
        let channels = resp.len();
        if channels < args.min_channels {
            continue;
        }
        let slope = phase_slope(&resp, 20);
        let fp = first_path(&resp, Window::Hann, FIRST_PATH_THRESHOLD, args.min_channels);
        let peaks: Vec<Peak> = profile_peaks(&resp, Window::Hann)
            .into_iter()
            .flatten()
            .filter(|p| p.magnitude >= args.min_peak)
            .collect();
        let e = Estimate {
            slope: slope.map(|s| s.distance_m),
            coherence: slope.map(|s| s.coherence),
            first: fp.map(|f| (f.distance_m, f.strength, f.peaks)),
            peaks,
        };
        if args.json {
            println!(
                "{}",
                json!({
                    "node": name(node), "procedure": procedure, "path": path, "paths": paths,
                    "channels": channels,
                    "phase_slope_m": e.slope.map(|v| round(v, 3)),
                    "coherence": e.coherence.map(|v| round(v, 3)),
                    "first_path_m": e.first.map(|f| round(f.0, 3)),
                    "first_path_strength": e.first.map(|f| round(f.1, 3)),
                    "components": e.first.map(|f| f.2),
                    "peaks": e.peaks.iter().map(|p| json!([round(p.distance_m, 2), round(p.magnitude, 3)])).collect::<Vec<_>>(),
                })
            );
        } else {
            let peaks: Vec<String> = e
                .peaks
                .iter()
                .map(|p| format!("{:.2} ({:.2})", p.distance_m, p.magnitude))
                .collect();
            println!(
                "{} procedure {procedure} path {path} of {paths}: {channels} channels, slope {}, first path {}, peaks {}",
                name(node),
                e.slope.map_or("-".into(), |v| format!(
                    "{v:.2} m (coherence {:.3})",
                    e.coherence.unwrap_or(0.0)
                )),
                e.first.map_or("-".into(), |f| format!(
                    "{:.2} m ({:.2} of strongest, {} components)",
                    f.0, f.1, f.2
                )),
                peaks.join(", ")
            );
        }
        per_node.entry(*node).or_default().push(e);
    }
    if args.json {
        return Ok(());
    }
    println!();
    for (node, es) in &per_node {
        let med = |v: Vec<f32>| -> String {
            let mut v = v;
            if v.is_empty() {
                return "-".into();
            }
            v.sort_by(f32::total_cmp);
            format!(
                "{:.2} (p10 {:.2}, p90 {:.2}, n={})",
                v[v.len() / 2],
                v[v.len() / 10],
                v[9 * v.len() / 10],
                v.len()
            )
        };
        println!(
            "{}: {} estimates; phase slope {}; first path {}; strongest peak {}; earliest peak above 0.3: {}",
            name(node),
            es.len(),
            med(es.iter().filter_map(|e| e.slope).collect()),
            med(es.iter().filter_map(|e| e.first.map(|f| f.0)).collect()),
            med(es
                .iter()
                .filter_map(|e| e.peaks.first().map(|p| p.distance_m))
                .collect()),
            med(es
                .iter()
                .filter_map(|e| e
                    .peaks
                    .iter()
                    .filter(|p| p.magnitude >= 0.3)
                    .map(|p| p.distance_m)
                    .reduce(f32::min))
                .collect()),
        );
    }
    Ok(())
}

fn round(v: f32, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (f64::from(v) * f).round() / f
}
