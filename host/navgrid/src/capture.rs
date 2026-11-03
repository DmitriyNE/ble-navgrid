//! `navgrid cs`: distances from raw CS captures, the `CSR`/`CSC` lines both boards log over RTT
//! (firmware `cs.rs`). Lines are found anywhere in the input, so probe-rs output works as is;
//! everything else is ignored.
//!
//! Each side's subevents are grouped into procedures (a `CSC` line continues the `CSR` before it
//! from the same side), the two sides are joined by procedure counter, and mode-2 tones on the
//! same channel are paired in order. Counters restart when the link reconnects, so a capture
//! should cover one connection.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use anyhow::{Context, Result, bail, ensure};
use navgrid_ranging::steps::{self, Role, Step, ToneQuality, Tones};
use navgrid_ranging::{
    ChannelResponse, Combine, FIRST_PATH_THRESHOLD, FirstPath, PhaseSlope, Window, combine_paths, first_path,
    phase_slope, rtt_distance_m,
};
use serde_json::{Value, json};

use crate::scenario::CombineArg;

#[derive(clap::Args)]
pub struct CsArgs {
    /// Capture files: probe-rs output from either board, or both.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// Fewest adjacent-channel pairs for a phase-slope estimate.
    #[arg(long, default_value_t = 20)]
    min_pairs: u8,
    /// How per-path estimates are combined into one range.
    #[arg(long, value_enum, default_value_t = CombineArg::ResidualWeighted)]
    combine: CombineArg,
    /// One JSON object per procedure.
    #[arg(long)]
    json: bool,
    /// The capture used inline PCT transfer: estimate from the initiator's tones alone, whose
    /// PCTs already carry the round trip.
    #[arg(long)]
    ipt: bool,
}

pub fn run(args: &CsArgs) -> Result<()> {
    let mut subevents = Vec::new();
    for path in &args.files {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        for (n, line) in text.lines().enumerate() {
            match parse_line(line) {
                Some(Ok(s)) => subevents.push(s),
                Some(Err(e)) => eprintln!("{}:{}: {e:#}", path.display(), n + 1),
                None => {}
            }
        }
    }
    if subevents.is_empty() {
        bail!("no CSR/CSC lines found");
    }
    let procedures = assemble(subevents);
    let estimates: Vec<Estimate> = procedures
        .iter()
        .map(|(&counter, sides)| estimate(counter, sides, args.min_pairs, args.combine.into(), args.ipt))
        .collect();
    for e in &estimates {
        if args.json {
            println!("{}", e.to_json());
        } else {
            println!("{e}");
        }
    }
    if !args.json {
        println!("{}", Summary::of(&estimates));
    }
    Ok(())
}

/// One `CSR` or `CSC` line.
#[derive(Debug, PartialEq)]
struct Subevent {
    role: Role,
    /// A `CSC` line: more steps of the previous subevent. Its counter and frequency fields are
    /// placeholders.
    continuation: bool,
    procedure: u16,
    freq_comp: u16,
    ref_power: i8,
    procedure_done: u8,
    subevent_done: u8,
    abort: u8,
    paths: u8,
    steps: Vec<u8>,
}

/// Done status for "aborted" (0 complete, 1 partial).
const ABORTED: u8 = 0xf;

/// Parses one capture line; `None` if it isn't a CS line.
fn parse_line(raw: &str) -> Option<Result<Subevent>> {
    let line = strip_ansi(raw);
    let (continuation, rest) = ["CSR ", "CSC "].iter().find_map(|tag| {
        let at = line.find(tag)?;
        (at == 0 || line[..at].ends_with(' ')).then(|| (*tag == "CSC ", &line[at + tag.len()..]))
    })?;
    Some(parse_fields(continuation, rest).with_context(|| format!("bad CS line: {}", line.trim())))
}

/// The fields after the tag: `<role> <procedure> <freq comp> <ref power> <procedure done>
/// <subevent done> <abort> <paths> <steps> [step bytes]`, as firmware `cs::log_steps` writes them.
fn parse_fields(continuation: bool, rest: &str) -> Result<Subevent> {
    let (head, tail) = rest.split_once('[').context("no step bytes")?;
    let (bytes, _) = tail.split_once(']').context("unterminated step bytes")?;
    let fields: Vec<&str> = head.split_whitespace().collect();
    let [
        role,
        procedure,
        freq_comp,
        ref_power,
        procedure_done,
        subevent_done,
        abort,
        paths,
        count,
    ] = fields[..]
    else {
        bail!("expected 9 fields before the step bytes, found {}", fields.len());
    };
    let hex = |s: &str| u8::from_str_radix(s, 16);
    let subevent = Subevent {
        role: match role {
            "I" => Role::Initiator,
            "R" => Role::Reflector,
            r => bail!("unknown role {r:?}"),
        },
        continuation,
        procedure: procedure.parse()?,
        freq_comp: freq_comp.parse()?,
        ref_power: ref_power.parse()?,
        procedure_done: hex(procedure_done)?,
        subevent_done: hex(subevent_done)?,
        abort: hex(abort)?,
        paths: paths.parse()?,
        steps: bytes
            .split(',')
            .map(str::trim)
            .filter(|b| !b.is_empty())
            .map(hex)
            .collect::<Result<_, _>>()?,
    };
    let announced: usize = count.parse()?;
    let framed = steps::split(&subevent.steps).count();
    ensure!(framed == announced, "{announced} steps announced, {framed} framed");
    Ok(subevent)
}

/// Drops terminal escape sequences (probe-rs colours its output on a terminal).
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\x1b' {
            for c in chars.by_ref() {
                if c.is_ascii_alphabetic() {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// One side of one procedure.
#[derive(Debug, Default)]
struct Side {
    paths: u8,
    subevents: u32,
    aborted: u32,
    /// Abort bytes seen on aborted subevents.
    aborts: BTreeSet<u8>,
    procedure_done: u8,
    freq_comp_ppm: Option<f32>,
    /// Step bytes of all subevents, concatenated: each step carries its own length.
    steps: Vec<u8>,
}

/// Procedures by counter, each with the initiator's side and the reflector's.
type Procedures = BTreeMap<u16, [Option<Side>; 2]>;

fn slot(role: Role) -> usize {
    match role {
        Role::Initiator => 0,
        Role::Reflector => 1,
    }
}

fn assemble(subevents: impl IntoIterator<Item = Subevent>) -> Procedures {
    let mut procedures = Procedures::new();
    let mut current: [Option<u16>; 2] = [None; 2];
    for s in subevents {
        let counter = if s.continuation {
            let Some(counter) = current[slot(s.role)] else {
                eprintln!("CSC line before any CSR line from {:?}; skipped", s.role);
                continue;
            };
            counter
        } else {
            current[slot(s.role)] = Some(s.procedure);
            s.procedure
        };
        let side = procedures.entry(counter).or_default()[slot(s.role)].get_or_insert_with(Side::default);
        if !s.continuation {
            side.subevents += 1;
            side.freq_comp_ppm = steps::freq_ppm(s.freq_comp).or(side.freq_comp_ppm);
        }
        if s.subevent_done == ABORTED {
            side.aborted += 1;
            side.aborts.insert(s.abort);
        }
        side.paths = s.paths;
        side.procedure_done = s.procedure_done;
        side.steps.extend_from_slice(&s.steps);
    }
    procedures
}

/// A side's steps, decoded.
#[derive(Default)]
struct Decoded {
    /// Mode-2 and mode-3 tones in step order, with their channel.
    tones: Vec<(u8, Tones)>,
    /// Mode-1 and mode-3 ToA/ToD values in step order.
    rtt: Vec<i16>,
}

fn decode(role: Role, side: &Side) -> Result<Decoded> {
    let mut d = Decoded::default();
    for raw in steps::split(&side.steps) {
        let raw = raw.map_err(|e| anyhow::anyhow!("{e:?}"))?;
        match steps::decode(role, side.paths, raw).map_err(|e| anyhow::anyhow!("{e:?} on channel {}", raw.channel))? {
            Step::Mode0 { .. } => {}
            Step::Mode1(r) => d.rtt.push(r.toa_tod),
            Step::Mode2(t) => d.tones.push((raw.channel, t)),
            Step::Mode3(r, t) => {
                d.rtt.push(r.toa_tod);
                d.tones.push((raw.channel, t));
            }
        }
    }
    Ok(d)
}

/// Per-antenna-path responses: the i-th tone on a channel from one side pairs with the i-th on
/// that channel from the other. Tones the controller marks unavailable are left out.
fn responses(initiator: &[(u8, Tones)], reflector: &[(u8, Tones)], paths: u8) -> Vec<ChannelResponse> {
    let by_channel = |tones: &[(u8, Tones)]| {
        let mut m: BTreeMap<u8, Vec<Tones>> = BTreeMap::new();
        for (channel, t) in tones {
            m.entry(*channel).or_default().push(*t);
        }
        m
    };
    let (init, refl) = (by_channel(initiator), by_channel(reflector));
    let usable = |q: ToneQuality| q != ToneQuality::Unavailable;
    (0..paths)
        .map(|path| {
            let mut resp = ChannelResponse::new();
            for (&channel, ours) in &init {
                for (a, b) in ours.iter().zip(refl.get(&channel).into_iter().flatten()) {
                    if let (Some(a), Some(b)) = (a.path(path), b.path(path))
                        && usable(a.quality)
                        && usable(b.quality)
                    {
                        resp.insert_pct(channel, a.pct, b.pct);
                    }
                }
            }
            resp
        })
        .collect()
}

/// Per-antenna-path responses from the initiator's tones alone (inline PCT transfer). Repeats on
/// a channel are summed, as in [`responses`].
fn responses_ipt(initiator: &[(u8, Tones)], paths: u8) -> Vec<ChannelResponse> {
    (0..paths)
        .map(|path| {
            let mut resp = ChannelResponse::new();
            for (channel, t) in initiator {
                if let Some(a) = t.path(path)
                    && a.quality != ToneQuality::Unavailable
                {
                    resp.insert(*channel, a.pct);
                }
            }
            resp
        })
        .collect()
}

struct Estimate {
    procedure: u16,
    sides: [Option<SideSummary>; 2],
    /// Channels with a usable tone pair, on the first path.
    channels: usize,
    paths: Vec<PhaseSlope>,
    combined: Option<PhaseSlope>,
    /// The first-path estimator per antenna path, and the median of those distances.
    first_paths: Vec<FirstPath>,
    first_path_m: Option<f32>,
    rtt_m: Option<f32>,
    /// Why there is no distance, when there isn't.
    note: Option<String>,
}

struct SideSummary {
    subevents: u32,
    aborted: u32,
    aborts: Vec<String>,
    procedure_done: u8,
    freq_comp_ppm: Option<f32>,
}

fn estimate(procedure: u16, sides: &[Option<Side>; 2], min_pairs: u8, how: Combine, ipt: bool) -> Estimate {
    let summary = |s: &Option<Side>| {
        s.as_ref().map(|s| SideSummary {
            subevents: s.subevents,
            aborted: s.aborted,
            aborts: s.aborts.iter().map(|&a| abort_reasons(a)).collect(),
            procedure_done: s.procedure_done,
            freq_comp_ppm: s.freq_comp_ppm,
        })
    };
    let mut e = Estimate {
        procedure,
        sides: [summary(&sides[0]), summary(&sides[1])],
        channels: 0,
        paths: Vec::new(),
        combined: None,
        first_paths: Vec::new(),
        first_path_m: None,
        rtt_m: None,
        note: None,
    };
    if ipt {
        let Some(init) = &sides[0] else {
            e.note = Some("no initiator side".into());
            return e;
        };
        let i = match decode(Role::Initiator, init) {
            Ok(i) => i,
            Err(err) => {
                e.note = Some(format!("bad step data: {err:#}"));
                return e;
            }
        };
        if i.tones.is_empty() {
            e.note = Some("no mode-2 tones".into());
            return e;
        }
        let resp = responses_ipt(&i.tones, init.paths);
        return finish(e, &resp, init.paths, min_pairs, how);
    }
    let [Some(init), Some(refl)] = sides else {
        e.note = Some(format!(
            "only the {} side",
            if sides[0].is_some() { "initiator" } else { "reflector" }
        ));
        return e;
    };
    if init.paths != refl.paths {
        e.note = Some(format!(
            "sides disagree on antenna paths ({} vs {})",
            init.paths, refl.paths
        ));
        return e;
    }
    let (i, r) = match (decode(Role::Initiator, init), decode(Role::Reflector, refl)) {
        (Ok(i), Ok(r)) => (i, r),
        (Err(err), _) | (_, Err(err)) => {
            e.note = Some(format!("bad step data: {err:#}"));
            return e;
        }
    };
    if !i.rtt.is_empty() && i.rtt.len() == r.rtt.len() {
        e.rtt_m = rtt_distance_m(i.rtt.iter().copied().zip(r.rtt.iter().copied()));
    }
    if i.tones.is_empty() || r.tones.is_empty() {
        e.note = Some("no mode-2 tones".into());
        return e;
    }
    let resp = responses(&i.tones, &r.tones, init.paths);
    finish(e, &resp, init.paths, min_pairs, how)
}

/// Phase slope per antenna path, then the combined range.
fn finish(mut e: Estimate, resp: &[ChannelResponse], paths: u8, min_pairs: u8, how: Combine) -> Estimate {
    e.channels = resp.first().map_or(0, ChannelResponse::len);
    if e.channels == 0 && paths > 1 {
        e.note = Some(format!("{paths} antenna paths: tone mapping not implemented"));
        return e;
    }
    e.paths = resp.iter().filter_map(|r| phase_slope(r, min_pairs)).collect();
    e.combined = combine_paths(&e.paths, how);
    e.first_paths = resp
        .iter()
        .filter_map(|r| first_path(r, Window::Hann, FIRST_PATH_THRESHOLD, usize::from(min_pairs)))
        .collect();
    let mut ds: Vec<f32> = e.first_paths.iter().map(|f| f.distance_m).collect();
    ds.sort_by(f32::total_cmp);
    e.first_path_m = ds.get(ds.len() / 2).copied();
    if e.combined.is_none() {
        e.note = Some(format!("{} usable channels, too few adjacent pairs", e.channels));
    }
    e
}

/// The abort byte (procedure reason in the low nibble, subevent reason in the high one) in words.
fn abort_reasons(abort: u8) -> String {
    let procedure = match abort & 0xf {
        0 => None,
        1 => Some("procedure: host or peer request"),
        2 => Some("procedure: channel map under 15 channels"),
        3 => Some("procedure: channel map instant passed"),
        _ => Some("procedure: unspecified"),
    };
    let subevent = match abort >> 4 {
        0 => None,
        1 => Some("subevent: host or peer request"),
        2 => Some("subevent: no CS_SYNC received"),
        3 => Some("subevent: scheduling conflict or limited resources"),
        _ => Some("subevent: unspecified"),
    };
    let reasons: Vec<&str> = procedure.into_iter().chain(subevent).collect();
    if reasons.is_empty() {
        "no reason given".into()
    } else {
        reasons.join(", ")
    }
}

impl std::fmt::Display for Estimate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "proc {:5}  ", self.procedure)?;
        match self.combined {
            Some(c) => write!(
                f,
                "{:7.3} m  coherence {:.3}  residual {:.3} rad  {} pairs",
                c.distance_m, c.coherence, c.residual_rad, c.pairs
            )?,
            None => write!(f, "      -  {}", self.note.as_deref().unwrap_or("no estimate"))?,
        }
        if let Some(rtt) = self.rtt_m {
            write!(f, "  rtt {rtt:.2} m")?;
        }
        for (name, side) in ["I", "R"].iter().zip(&self.sides) {
            if let Some(s) = side {
                let plural = if s.subevents == 1 { "" } else { "s" };
                write!(f, "  {name}: {} subevent{plural}", s.subevents)?;
                if s.aborted > 0 {
                    write!(f, ", {} aborted ({})", s.aborted, s.aborts.join("; "))?;
                }
                if let Some(ppm) = s.freq_comp_ppm {
                    write!(f, ", {ppm:+.2} ppm")?;
                }
            }
        }
        Ok(())
    }
}

impl Estimate {
    fn to_json(&self) -> Value {
        let slope = |p: &PhaseSlope| {
            json!({
                "distance_m": round(p.distance_m, 4),
                "coherence": round(p.coherence, 4),
                "residual_rad": round(p.residual_rad, 4),
                "pairs": p.pairs,
            })
        };
        let side = |s: &Option<SideSummary>| {
            s.as_ref().map(|s| {
                json!({
                    "subevents": s.subevents,
                    "aborted": s.aborted,
                    "abort_reasons": s.aborts,
                    "procedure_done": s.procedure_done,
                    "freq_comp_ppm": s.freq_comp_ppm.map(|v| round(v, 2)),
                })
            })
        };
        json!({
            "procedure": self.procedure,
            "estimate": self.combined.as_ref().map(slope),
            "paths": self.paths.iter().map(slope).collect::<Vec<_>>(),
            "first_paths": self.first_paths.iter().map(|f| json!({
                "distance_m": round(f.distance_m, 4),
                "strength": round(f.strength, 4),
                "strongest_m": round(f.strongest_m, 4),
                "peaks": f.peaks,
                "components": f.components[..usize::from(f.peaks)].iter().map(|(d, s)| [round(*d, 3), round(*s, 4)]).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
            "first_path_m": self.first_path_m.map(|d| round(d, 4)),
            "channels": self.channels,
            "rtt_m": self.rtt_m.map(|v| round(v, 3)),
            "initiator": side(&self.sides[0]),
            "reflector": side(&self.sides[1]),
            "note": self.note,
        })
    }
}

/// `v` to `digits` decimals, so JSON doesn't show f32 rounding noise.
fn round(v: f32, digits: i32) -> f64 {
    let scale = 10f64.powi(digits);
    (f64::from(v) * scale).round() / scale
}

/// Totals over a capture; the distance spread matters for a fixed-distance capture.
struct Summary {
    procedures: usize,
    joined: usize,
    distances: Vec<f32>,
    subevents: [u32; 2],
    aborted: [u32; 2],
}

impl Summary {
    fn of(estimates: &[Estimate]) -> Self {
        let mut s = Summary {
            procedures: estimates.len(),
            joined: estimates.iter().filter(|e| e.sides.iter().all(Option::is_some)).count(),
            distances: estimates
                .iter()
                .filter_map(|e| e.combined.map(|c| c.distance_m))
                .collect(),
            subevents: [0; 2],
            aborted: [0; 2],
        };
        for e in estimates {
            for (k, side) in e.sides.iter().enumerate() {
                if let Some(side) = side {
                    s.subevents[k] += side.subevents;
                    s.aborted[k] += side.aborted;
                }
            }
        }
        s.distances.sort_by(f32::total_cmp);
        s
    }
}

impl std::fmt::Display for Summary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} procedures, {} with both sides, {} with a distance",
            self.procedures,
            self.joined,
            self.distances.len()
        )?;
        let d = &self.distances;
        if !d.is_empty() {
            let at = |q: f32| d[((d.len() - 1) as f32 * q).round() as usize];
            write!(f, "; median {:.3} m, p10–p90 {:.3}–{:.3} m", at(0.5), at(0.1), at(0.9))?;
        }
        write!(
            f,
            "\naborted subevents: initiator {}/{}, reflector {}/{}",
            self.aborted[0], self.subevents[0], self.aborted[1], self.subevents[1]
        )
    }
}

#[cfg(test)]
mod tests {
    use navgrid_ranging::sim::{Path, SplitMix64, for_each_tone};
    use navgrid_ranging::steps::encode_pct;
    use navgrid_ranging::{Complex, is_cs_channel};

    use super::*;

    #[test]
    fn parses_probe_rs_lines() {
        let line = "2.396482 [INFO ] CSR I 0 32162 -30 0 f 30 1 0 [] (navgrid_node node/src/cs.rs:183)";
        let s = parse_line(line).unwrap().unwrap();
        assert_eq!(
            s,
            Subevent {
                role: Role::Initiator,
                continuation: false,
                procedure: 0,
                freq_comp: 32162,
                ref_power: -30,
                procedure_done: 0,
                subevent_done: 0xf,
                abort: 0x30,
                paths: 1,
                steps: vec![],
            }
        );
        let coloured = "\x1b[2m1.0\x1b[0m [INFO ] CSC R 0 0 0 1 1 00 1 1 [00, 0a, 03, 01, 02, 03] (x)";
        let s = parse_line(coloured).unwrap().unwrap();
        assert!(s.continuation);
        assert_eq!(s.steps, [0, 10, 3, 1, 2, 3]);
        assert!(parse_line("0.19 [INFO ] advertising (navgrid_node node/src/main.rs:275)").is_none());
        assert!(
            parse_line("CSR I 0 0 0 0 0 00 1 2 [00, 0a, 03, 01, 02, 03]")
                .unwrap()
                .is_err()
        );
    }

    #[test]
    fn abort_bytes_in_words() {
        assert_eq!(
            abort_reasons(0x30),
            "subevent: scheduling conflict or limited resources"
        );
        assert_eq!(
            abort_reasons(0x21),
            "procedure: host or peer request, subevent: no CS_SYNC received"
        );
    }

    /// Writes one side of a simulated procedure as the firmware logs it: mode-0 then mode-2
    /// steps, split over subevents of at most `per_subevent` steps, the first of each subevent as
    /// `CSR` and the rest as `CSC`.
    fn log_side(role: Role, procedure: u16, tones: &[(u8, Complex)], per_subevent: usize) -> Vec<String> {
        let (tag_role, freq_comp, mode0) = match role {
            Role::Initiator => ("I", 0x7da2, vec![0, 0xe2, 0, 0xa2, 0x7d]),
            Role::Reflector => ("R", 0xc000, vec![0, 0xe2, 0]),
        };
        let mut steps: Vec<Vec<u8>> = (0..3)
            .map(|_| [vec![0, 40, mode0.len() as u8], mode0.clone()].concat())
            .collect();
        for &(channel, pct) in tones {
            let p = encode_pct(pct);
            // Permutation 0, the path tone (high quality), the extension slot (no tone expected).
            steps.push(vec![2, channel, 9, 0, p[0], p[1], p[2], 0x00, 0, 0, 0, 0x13]);
        }
        let chunks: Vec<&[Vec<u8>]> = steps.chunks(per_subevent).collect();
        chunks
            .iter()
            .enumerate()
            .map(|(k, chunk)| {
                let last = k + 1 == chunks.len();
                let bytes: Vec<String> = chunk.concat().iter().map(|b| format!("{b:02x}")).collect();
                format!(
                    "{}.0 [INFO ] {} {tag_role} {procedure} {freq_comp} -30 {:x} {:x} 00 1 {} [{}] (navgrid_node node/src/cs.rs:183)",
                    k,
                    if k % 2 == 0 { "CSR" } else { "CSC" },
                    if last { 0 } else { 1 },
                    if k % 2 == 1 || last { 0 } else { 1 },
                    chunk.len(),
                    bytes.join(", ")
                )
            })
            .collect()
    }

    #[test]
    fn simulated_capture_round_trips_to_distance() {
        let mut rng = SplitMix64::new(5);
        let mut lines = Vec::new();
        for (procedure, truth) in [(7, 4.2), (8, 4.2)] {
            let (mut init, mut refl) = (Vec::new(), Vec::new());
            let path = [Path {
                length_m: truth,
                amplitude: 1.0,
            }];
            for_each_tone(&path, 0.05, &mut rng, |channel, i, r| {
                assert!(is_cs_channel(channel));
                // PCTs are 12-bit; use most of the range.
                let scale = |c: Complex| Complex::new(c.re * 1500.0, c.im * 1500.0);
                init.push((channel, scale(i)));
                refl.push((channel, scale(r)));
            });
            lines.extend(log_side(Role::Initiator, procedure, &init, 20));
            lines.extend(log_side(Role::Reflector, procedure, &refl, 30));
        }
        let subevents: Vec<Subevent> = lines.iter().map(|l| parse_line(l).unwrap().unwrap()).collect();
        let procedures = assemble(subevents);
        assert_eq!(procedures.keys().copied().collect::<Vec<_>>(), [7, 8]);
        for (&counter, sides) in &procedures {
            let e = estimate(counter, sides, 20, Combine::ResidualWeighted, false);
            let c = e.combined.unwrap_or_else(|| panic!("{e}"));
            assert!((c.distance_m - 4.2).abs() < 0.1, "{e}");
            assert_eq!(e.channels, 72);
            assert_eq!(e.sides[0].as_ref().unwrap().freq_comp_ppm, Some(-6.06));
        }
    }

    #[test]
    fn aborted_procedures_say_why() {
        let lines = [
            "CSR I 3 32162 -30 0 f 30 1 0 []",
            "CSR R 3 49152 0 0 f 30 1 0 []",
            "CSR R 4 49152 0 0 f 20 1 0 []",
        ];
        let procedures = assemble(lines.iter().map(|l| parse_line(l).unwrap().unwrap()));
        let three = estimate(3, &procedures[&3], 20, Combine::Mean, false);
        assert_eq!(three.note.as_deref(), Some("no mode-2 tones"));
        assert!(
            three
                .to_string()
                .contains("I: 1 subevent, 1 aborted (subevent: scheduling conflict"),
            "{three}"
        );
        assert_eq!(three.to_json()["initiator"]["freq_comp_ppm"], -6.06);
        let four = estimate(4, &procedures[&4], 20, Combine::Mean, false);
        assert_eq!(four.note.as_deref(), Some("only the reflector side"));
        let summary = Summary::of(&[three, four]).to_string();
        assert!(
            summary.starts_with("2 procedures, 1 with both sides, 0 with a distance"),
            "{summary}"
        );
        assert!(summary.contains("initiator 1/1, reflector 2/2"), "{summary}");
    }
}
