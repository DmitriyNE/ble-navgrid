//! `navgrid serve`: the host as a service (FR-H, FR-S6, D-23). It owns the sink's serial port:
//! every frame the sink forwards goes into the host replica (mesh records), the monitor's lines
//! (ranges, health, boots, logs) and a JSON log the offline commands read. It solves the mesh on
//! a schedule and writes the positions back through the sink, and serves a JSON state endpoint
//! and one static page: the floor plan, the nodes, links, health and commands, controls, the
//! find pointer, and pin placement.
//!
//! The HTTP side is plain `std`: one thread per connection, GET and POST with a content length,
//! no keep-alive. It is meant for a laptop on the bench, not the open internet.

use std::collections::{BTreeMap, VecDeque};
use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Command, Heap, Key, NodeId, Record, Replica, Status, Topic, Writer, auth};
use navgrid_proto::Message;
use navgrid_solver::frame::{compass_to_heading, pointer};
use serde_json::{Value, json};

use crate::control::{command_text, parse_target};
use crate::fleet::{Fleet, FleetArgs, Registry};
use crate::mesh_view::{HOST, node_name};
use crate::monitor::{self, View};
use crate::sign::parse_tag;
use crate::solve::{
    Solution, anchor_pairs, controller_writer, is_tag, latest_pairs, position_records, send_record, solve_pairs,
};
use crate::survey::{History, Observation, PairStats, now_s};

#[derive(clap::Args)]
pub struct ServeArgs {
    /// A sink's serial port (its VCOM1); repeat for several sinks, whose records merge here and
    /// which all get the controller's records.
    #[arg(long, required_unless_present = "stream", conflicts_with = "stream")]
    sink: Vec<String>,
    /// Instead of a port: a file or FIFO with the sink's raw frames (a replay, or a bench
    /// without a sink); nothing is written back.
    #[arg(long)]
    stream: Option<String>,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    /// Where the page and the JSON endpoint listen.
    #[arg(long, default_value = "127.0.0.1:8080")]
    http: String,
    /// Solve the mesh and write the positions back this often, 0 = never.
    #[arg(long, default_value_t = 60)]
    solve_every_s: u64,
    /// Taken off every distance: the chip's delay (DEF-6).
    #[arg(long, default_value_t = 1.5)]
    offset_m: f32,
    #[arg(long, default_value_t = 0.3)]
    height_sigma_m: f32,
    #[arg(long, default_value_t = 100)]
    min_quality: u8,
    #[command(flatten)]
    fleet: FleetArgs,
}

const RECENT_LINES: usize = 200;
const PAGE: &str = include_str!("../web/index.html");
/// The survey window to start with: an anchor link bursts every 15 min, so eight bursts.
const SURVEY_WINDOW_S: u64 = 2 * 3600;

struct Service {
    replica: Replica<Heap>,
    registry: Option<Registry>,
    /// Pins placed on the page, by node; they override the nodes' own.
    pins: BTreeMap<NodeId, [f32; 3]>,
    pins_path: PathBuf,
    recent: VecDeque<String>,
    /// Beacons the sinks heard, by node: the latest, and when.
    nearby: BTreeMap<NodeId, (navgrid_proto::Heard, Instant)>,
    frames: u64,
    last_solve: Option<(Instant, Solution)>,
    /// Every burst the sinks mirrored, per pair; the solver takes each pair's median.
    history: History,
    survey_log: Option<std::fs::File>,
    /// Bursts older than this don't count; 0 = all of them.
    window_s: u64,
    writer: Writer,
    controller_key: Option<[u8; 16]>,
    started: Instant,
    view: View,
    log: Option<std::fs::File>,
    offset_m: f32,
    height_sigma_m: f32,
    min_quality: u8,
}

type Shared = Arc<Mutex<Service>>;
/// The ways down to the sinks, if any.
type Port = Arc<Mutex<Vec<Box<dyn Write + Send>>>>;

impl Service {
    fn now_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    fn name(&self, id: NodeId) -> String {
        node_name(id, self.registry.as_ref())
    }

    fn resolve(&self, s: &str) -> Result<NodeId> {
        match &self.registry {
            Some(r) => parse_target(s, r),
            None => {
                if s.len() == 12
                    && let Ok(id) = u64::from_str_radix(s, 16)
                {
                    return Ok(NodeId(id));
                }
                bail!("{s}: no registry to look up a fleet number; give a node ID")
            }
        }
    }

    /// One frame from the sink: into the replica, the lines and the log.
    fn frame(&mut self, frame: &[u8]) {
        self.frames += 1;
        if let Ok(line) = self.view.render(frame, true)
            && let Some(log) = self.log.as_mut()
        {
            let _ = writeln!(log, "{line}");
        }
        if let Ok(line) = self.view.render(frame, false) {
            if self.recent.len() >= RECENT_LINES {
                self.recent.pop_front();
            }
            self.recent.push_back(line);
        }
        match navgrid_proto::decode(&mut frame.to_vec()) {
            Ok(Message::Record(r)) => {
                if let Ok(record) = postcard::from_bytes::<Record>(&r.bytes) {
                    let now = self.now_ms();
                    if let Some(obs) = Observation::from_record(&record, now_s())
                        && self.history.observe(obs)
                        && let Some(log) = self.survey_log.as_mut()
                        && let Err(e) = History::append(log, &obs)
                    {
                        eprintln!("survey: {e:#}");
                    }
                    let _ = self.replica.receive(record, now);
                }
            }
            Ok(Message::Heard(h)) => {
                self.nearby.insert(h.of, (h, Instant::now()));
            }
            _ => {}
        }
    }

    /// Where the last solve put `id`, if it did: what the page draws when the mesh holds no
    /// position for it yet (a local frame, or a solve not written back).
    fn solved_of(&self, id: NodeId) -> Option<Value> {
        let (_, s) = self.last_solve.as_ref()?;
        let p = s.placed.iter().find(|p| p.id == id)?;
        let sigma = |v: f32| if v.is_finite() { Value::from(v) } else { Value::Null };
        Some(json!({
            "x": p.position[0], "y": p.position[1], "z": p.position[2],
            "sigma": [sigma(p.sigma_m[0]), sigma(p.sigma_m[1]), sigma(p.sigma_m[2])],
            "gauge": p.pinned && s.local_frame.is_some(), "local": s.local_frame.is_some(),
        }))
    }

    /// Bursts before this don't count in the survey.
    fn since_s(&self) -> u64 {
        if self.window_s == 0 {
            0
        } else {
            now_s().saturating_sub(self.window_s)
        }
    }

    /// The pairs the solver takes: the survey's aggregates, or, with no survey yet (a replay),
    /// the latest bursts in the replica.
    fn survey_pairs(&self) -> Vec<PairStats> {
        if self.history.is_empty() {
            latest_pairs(&self.replica, self.min_quality)
        } else {
            self.history.aggregate(self.since_s(), self.min_quality)
        }
    }

    /// The survey for the page: every pair's aggregate, with how the last solve fitted it.
    fn survey(&self) -> Value {
        let now = now_s();
        let pairs: Vec<Value> = self
            .survey_pairs()
            .iter()
            .map(|p| {
                let fit = self.last_solve.as_ref().and_then(|(_, s)| s.fit(p.a, p.b));
                json!({
                    "a": self.name(p.a), "b": self.name(p.b),
                    "n": p.n, "from_a": p.from_a, "from_b": p.from_b,
                    "median_m": p.median_m, "spread_m": p.spread_m, "sigma_m": p.sigma_m,
                    "last_m": p.last_m, "last_age_s": if p.last_at_s == 0 { Value::Null } else { now.saturating_sub(p.last_at_s).into() },
                    "quality": p.quality, "paths": p.paths,
                    "solved_m": fit.and_then(|f| f.solved_m), "residual_m": fit.and_then(|f| f.residual_m),
                    "cut": fit.is_some_and(|f| f.cut),
                    "tag": is_tag(&self.replica, self.registry.as_ref(), p.a)
                        || is_tag(&self.replica, self.registry.as_ref(), p.b),
                })
            })
            .collect();
        json!({
            "window_s": self.window_s, "observations": self.history.len(), "pairs": pairs,
        })
    }

    fn position_of(&self, origin: NodeId, of: NodeId) -> Option<Value> {
        let s = self.replica.get(&Key {
            origin,
            topic: Topic::Position(of),
        })?;
        let Body::Position(p) = &s.record().body else {
            return None;
        };
        Some(json!({
            "x": p.point.x as f64 / 1000.0, "y": p.point.y as f64 / 1000.0, "z": p.point.z as f64 / 1000.0,
            "sigma": [p.sigma_mm[0] as f64 / 1000.0, p.sigma_mm[1] as f64 / 1000.0, p.sigma_mm[2] as f64 / 1000.0],
            "age_s": s.age_ms(self.now_ms()) / 1000,
        }))
    }

    fn state(&self) -> Value {
        let now = self.now_ms();
        let mut nodes = Vec::new();
        let mut links = Vec::new();
        let mut commands = Vec::new();
        for s in self.replica.records() {
            let r = s.record();
            if r.origin == HOST {
                continue;
            }
            match &r.body {
                Body::Node(n) => {
                    let health = self
                        .replica
                        .get(&Key {
                            origin: r.origin,
                            topic: Topic::Health,
                        })
                        .and_then(|h| match &h.record().body {
                            Body::Health(x) => Some(json!({
                                "battery_mv": x.battery_mv, "temperature_c": f64::from(x.temperature_cdeg) / 100.0,
                                "uptime_s": x.uptime_s, "queued": x.queued, "age_s": h.age_ms(now) / 1000,
                            })),
                            _ => None,
                        });
                    let pinned = self
                        .pins
                        .get(&r.origin)
                        .map(|p| json!({"x": p[0], "y": p[1], "z": p[2], "placed": true}))
                        .or_else(|| {
                            n.pinned.map(|p| {
                                json!({"x": p.x as f64 / 1000.0, "y": p.y as f64 / 1000.0, "z": p.z as f64 / 1000.0, "placed": false})
                            })
                        });
                    let label = self
                        .registry
                        .as_ref()
                        .and_then(|reg| reg.by_id(r.origin))
                        .and_then(|d| d.name.clone());
                    nodes.push(json!({
                        "id": format!("{:012x}", r.origin.0), "name": self.name(r.origin), "label": label,
                        "role": format!("{:?}", n.role), "number": n.number,
                        "hops": if n.hops == u8::MAX { Value::Null } else { n.hops.into() },
                        "neighbours": n.neighbours.iter().map(|&id| self.name(id)).collect::<Vec<_>>(),
                        "firmware": n.firmware.to_string(), "age_s": s.age_ms(now) / 1000,
                        "pinned": pinned, "health": health,
                        "position": self.position_of(CONTROLLER, r.origin),
                        "own_position": self.position_of(r.origin, r.origin),
                        "solved": self.solved_of(r.origin),
                    }));
                }
                Body::Link(l) => links.push(json!({
                    "from": self.name(r.origin), "to": self.name(l.peer),
                    "distance_m": l.distance_mm as f64 / 1000.0, "sigma_m": l.sigma_mm as f64 / 1000.0,
                    "quality": l.quality, "procedures": l.procedures, "failures": l.failures,
                    "age_s": s.age_ms(now) / 1000,
                })),
                Body::Desired(d) => {
                    let reported = self
                        .replica
                        .get(&Key {
                            origin: d.target,
                            topic: Topic::Reported(d.command.setting()),
                        })
                        .and_then(|rep| match &rep.record().body {
                            Body::Reported(x) if x.applied == r.version => Some(match x.status {
                                Status::Applied => "applied",
                                Status::Unsupported => "unsupported",
                                Status::Invalid => "invalid",
                            }),
                            Body::Reported(_) => Some("an older version"),
                            _ => None,
                        })
                        .unwrap_or("pending");
                    commands.push(json!({
                        "target": self.name(d.target), "command": d.command, "text": command_text(&d.command),
                        "version": format!("{}.{}", r.version.epoch, r.version.counter),
                        "reported": reported, "age_s": s.age_ms(now) / 1000,
                    }));
                }
                _ => {}
            }
        }
        let solve = self.last_solve.as_ref().map(|(at, s)| {
            json!({
                "ago_s": at.elapsed().as_secs(), "nodes": s.nodes, "pins": s.pins, "links": s.links,
                "iterations": s.iterations, "normalized_rms": s.normalized_rms, "outliers": s.outliers,
                "unsolved": s.unsolved.iter().map(|&id| self.name(id)).collect::<Vec<_>>(),
                "local_frame": s.local_frame.map(|(a, b)| [self.name(a), self.name(b)]),
            })
        });
        let heard_ids: Vec<NodeId> = nodes
            .iter()
            .filter_map(|n| {
                n["id"]
                    .as_str()
                    .and_then(|s| u64::from_str_radix(s, 16).ok())
                    .map(NodeId)
            })
            .collect();
        let fleet: Vec<Value> = self
            .registry
            .as_ref()
            .map(|r| {
                r.devices
                    .iter()
                    .map(|d| {
                        json!({
                            "name": crate::fleet::label_name(d.number), "label": d.name,
                            "role": format!("{:?}", d.role), "sink": d.sink,
                            "status": format!("{:?}", d.status).to_lowercase(),
                            "heard": heard_ids.contains(&d.id) || self.nearby.contains_key(&d.id),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        let nearby: Vec<Value> = self
            .nearby
            .iter()
            .map(|(id, (h, at))| {
                json!({
                    "name": self.name(*id), "by": self.name(h.node), "rssi_dbm": h.rssi_dbm,
                    "role": format!("{:?}", h.role), "sink": h.sink, "attention": h.attention, "install": h.install,
                    "hops": if h.hops == u8::MAX { Value::Null } else { h.hops.into() },
                    "free_links": h.free_links, "battery_mv": h.battery_mv, "age_s": at.elapsed().as_secs(),
                })
            })
            .collect();
        json!({
            "uptime_s": self.started.elapsed().as_secs(), "frames": self.frames,
            "records": self.replica.records().len(),
            "nodes": nodes, "links": links, "commands": commands, "solve": solve,
            "survey": self.survey(), "fleet": fleet, "nearby": nearby,
            "recent": self.recent.iter().rev().take(50).collect::<Vec<_>>(),
        })
    }

    fn pointer(&self, from: &str, to: &str, heading_deg: f32) -> Result<Value> {
        let from = self.resolve(from)?;
        let to = self.resolve(to)?;
        let point = |id: NodeId| -> Option<([f32; 3], &'static str)> {
            let own = self.position_of(id, id).map(|v| (v, "its own fix"));
            let ctrl = self.position_of(CONTROLLER, id).map(|v| (v, "the controller's solve"));
            let (v, how) = own.or(ctrl)?;
            Some((
                [
                    v["x"].as_f64()? as f32,
                    v["y"].as_f64()? as f32,
                    v["z"].as_f64()? as f32,
                ],
                how,
            ))
        };
        let (a, a_how) = point(from).with_context(|| format!("{} has no position yet", self.name(from)))?;
        let (b, b_how) = point(to).with_context(|| format!("{} has no position yet", self.name(to)))?;
        let p = pointer(a, b, compass_to_heading(heading_deg.to_radians()));
        Ok(json!({
            "from": self.name(from), "from_how": a_how, "to": self.name(to), "to_how": b_how,
            "relative_azimuth_deg": p.relative_azimuth_rad.to_degrees(),
            "elevation_deg": p.elevation_rad.to_degrees(), "distance_m": p.distance_m,
        }))
    }

    fn save_pins(&self) {
        let named: BTreeMap<String, [f32; 3]> = self.pins.iter().map(|(&id, p)| (self.name(id), *p)).collect();
        if let Ok(text) = serde_json::to_string_pretty(&named) {
            let _ = std::fs::write(&self.pins_path, text + "\n");
        }
    }
}

pub fn run(args: &ServeArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet).ok();
    let registry = fleet.as_ref().and_then(|f| f.registry().ok());
    let dir = args.fleet.dir()?;
    std::fs::create_dir_all(&dir).ok();
    let controller_key = fleet
        .as_ref()
        .and_then(|f| f.key().ok())
        .map(|k| auth::controller_key(&k));
    if controller_key.is_none() {
        eprintln!("no fleet key: commands can't be signed (run `navgrid fleet init`)");
    }
    let log_path = dir.join("serve.jsonl");
    let log = OpenOptions::new().append(true).create(true).open(&log_path).ok();
    let survey_path = dir.join("survey.jsonl");
    let history = History::load(&survey_path).unwrap_or_else(|e| {
        eprintln!("survey: {e:#}; starting one afresh");
        History::default()
    });
    let survey_log = OpenOptions::new().append(true).create(true).open(&survey_path).ok();
    let pins_path = dir.join("pins.json");
    let mut replica = crate::mesh_view::host_replica();
    replica.config_mut().controller_key = controller_key;
    let mut service = Service {
        replica,
        registry: registry.clone(),
        pins: BTreeMap::new(),
        pins_path: pins_path.clone(),
        recent: VecDeque::new(),
        nearby: BTreeMap::new(),
        frames: 0,
        last_solve: None,
        history,
        survey_log,
        window_s: SURVEY_WINDOW_S,
        writer: controller_writer(),
        controller_key,
        started: Instant::now(),
        view: View::new(None, registry),
        log,
        offset_m: args.offset_m,
        height_sigma_m: args.height_sigma_m,
        min_quality: args.min_quality,
    };
    if let Ok(text) = std::fs::read_to_string(&pins_path)
        && let Ok(named) = serde_json::from_str::<BTreeMap<String, [f32; 3]>>(&text)
    {
        for (name, p) in named {
            if let Ok(id) = service.resolve(&name) {
                service.pins.insert(id, p);
            }
        }
    }
    let mut readers: Vec<(String, Box<dyn Read + Send>)> = Vec::new();
    let mut downs: Vec<Box<dyn Write + Send>> = Vec::new();
    for sink in &args.sink {
        let port = serialport::new(sink, args.baud)
            .timeout(Duration::from_millis(200))
            .open()
            .with_context(|| format!("opening {sink}"))?;
        downs.push(Box::new(port.try_clone().context("cloning the port")?));
        readers.push((sink.clone(), Box::new(port)));
    }
    if let Some(stream) = &args.stream {
        let file = std::fs::File::open(stream).with_context(|| format!("opening {stream}"))?;
        readers.push((stream.clone(), Box::new(file)));
    }
    if readers.is_empty() {
        bail!("a --sink port or a --stream path");
    }
    let writer_port: Port = Arc::new(Mutex::new(downs));
    let shared: Shared = Arc::new(Mutex::new(service));
    println!(
        "serving {} on http://{} ; frames logged to {}; survey of {} bursts in {}",
        readers
            .iter()
            .map(|(name, _)| name.as_str())
            .collect::<Vec<_>>()
            .join(", "),
        args.http,
        log_path.display(),
        shared.lock().expect("service").history.len(),
        survey_path.display()
    );

    // A reader per sink: its frames into the one state.
    for (name, mut reader) in readers {
        let shared = shared.clone();
        thread::spawn(move || {
            let mut pending = Vec::new();
            let mut buf = [0u8; 512];
            loop {
                match reader.read(&mut buf) {
                    // A file or FIFO at its end: keep the state, wait for more.
                    Ok(0) => thread::sleep(Duration::from_millis(200)),
                    Ok(n) => pending.extend_from_slice(&buf[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                    Err(e) => {
                        eprintln!("{name}: {e}");
                        thread::sleep(Duration::from_secs(1));
                        continue;
                    }
                }
                for frame in monitor::take_frames(&mut pending) {
                    shared.lock().expect("service").frame(&frame);
                }
            }
        });
    }
    // The solver, on its schedule.
    if args.solve_every_s > 0 {
        let shared = shared.clone();
        let port = writer_port.clone();
        let every = Duration::from_secs(args.solve_every_s);
        thread::spawn(move || {
            loop {
                thread::sleep(every);
                if let Err(e) = solve_now(&shared, &port) {
                    eprintln!("solve: {e:#}");
                }
            }
        });
    }
    // The page and the endpoint.
    let listener = TcpListener::bind(&args.http).with_context(|| format!("listening on {}", args.http))?;
    for stream in listener.incoming() {
        let Ok(stream) = stream else {
            continue;
        };
        let shared = shared.clone();
        let port = writer_port.clone();
        thread::spawn(move || {
            if let Err(e) = handle(stream, &shared, &port) {
                eprintln!("http: {e:#}");
            }
        });
    }
    Ok(())
}

fn solve_now(shared: &Shared, port: &Port) -> Result<Value> {
    let (solution, records) = {
        let mut s = shared.lock().expect("service");
        let pins: Vec<(NodeId, [f32; 3])> = s.pins.iter().map(|(&id, &p)| (id, p)).collect();
        let pairs = anchor_pairs(&s.replica, s.registry.as_ref(), s.survey_pairs());
        let solution = solve_pairs(&s.replica, &pins, &pairs, s.offset_m, s.height_sigma_m)?;
        // A local frame stays on the page: the mesh gets positions once pins fix the frame.
        let records = if solution.local_frame.is_some() {
            Vec::new()
        } else {
            position_records(&solution, &mut s.writer)
        };
        s.last_solve = Some((Instant::now(), solution.clone()));
        // Into the host's own view at once; the sink mirrors the same versions back later.
        let now = s.now_ms();
        for r in &records {
            let _ = s.replica.receive(r.clone(), now);
        }
        (solution, records)
    };
    {
        let mut ports = port.lock().expect("port");
        for port in ports.iter_mut() {
            for r in &records {
                send_record(&mut **port, r)?;
            }
        }
    }
    Ok(json!({
        "nodes": solution.nodes, "pins": solution.pins, "links": solution.links, "placed": solution.placed.len(),
        "iterations": solution.iterations, "normalized_rms": solution.normalized_rms, "outliers": solution.outliers,
    }))
}

/// One HTTP exchange.
fn handle(mut stream: TcpStream, shared: &Shared, port: &Port) -> Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut request = String::new();
    reader.read_line(&mut request)?;
    let mut parts = request.split_whitespace();
    let (method, target) = (parts.next().unwrap_or(""), parts.next().unwrap_or("/"));
    let mut content_length = 0usize;
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 || line == "\r\n" || line == "\n" {
            break;
        }
        if let Some(v) = line.to_ascii_lowercase().strip_prefix("content-length:") {
            content_length = v.trim().parse().unwrap_or(0);
        }
    }
    let mut body = vec![0u8; content_length];
    if content_length > 0 {
        reader.read_exact(&mut body)?;
    }
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let params: BTreeMap<String, String> = query
        .split('&')
        .filter_map(|kv| kv.split_once('=').map(|(k, v)| (k.to_string(), percent_decode(v))))
        .collect();
    let (status, content_type, text) = match (method, path) {
        ("GET", "/") | ("GET", "/index.html") => ("200 OK", "text/html; charset=utf-8", PAGE.to_string()),
        ("GET", "/api/state") => (
            "200 OK",
            "application/json",
            shared.lock().expect("service").state().to_string(),
        ),
        ("GET", "/api/pointer") => {
            let s = shared.lock().expect("service");
            let heading: f32 = params.get("heading").and_then(|h| h.parse().ok()).unwrap_or(0.0);
            match s.pointer(
                params.get("from").map_or("", String::as_str),
                params.get("to").map_or("", String::as_str),
                heading,
            ) {
                Ok(v) => ("200 OK", "application/json", v.to_string()),
                Err(e) => (
                    "400 Bad Request",
                    "application/json",
                    json!({"error": format!("{e:#}")}).to_string(),
                ),
            }
        }
        ("POST", "/api/solve") => match solve_now(shared, port) {
            Ok(v) => ("200 OK", "application/json", v.to_string()),
            Err(e) => (
                "400 Bad Request",
                "application/json",
                json!({"error": format!("{e:#}")}).to_string(),
            ),
        },
        ("POST", "/api/control") => match control(shared, port, &body) {
            Ok(v) => ("200 OK", "application/json", v.to_string()),
            Err(e) => (
                "400 Bad Request",
                "application/json",
                json!({"error": format!("{e:#}")}).to_string(),
            ),
        },
        ("POST", "/api/survey") => match survey_window(shared, &body) {
            Ok(v) => ("200 OK", "application/json", v.to_string()),
            Err(e) => (
                "400 Bad Request",
                "application/json",
                json!({"error": format!("{e:#}")}).to_string(),
            ),
        },
        ("POST", "/api/pin") => match pin(shared, &body) {
            Ok(v) => ("200 OK", "application/json", v.to_string()),
            Err(e) => (
                "400 Bad Request",
                "application/json",
                json!({"error": format!("{e:#}")}).to_string(),
            ),
        },
        _ => ("404 Not Found", "text/plain", "not here".to_string()),
    };
    let response = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n",
        text.len()
    );
    stream.write_all(response.as_bytes())?;
    stream.write_all(text.as_bytes())?;
    Ok(())
}

/// A command from the page: `{"target": "N042", "command": {"Blink": {"times": 3}}}`, or
/// `{"target": "N042", "firmware": "0.1.0+1950d45d"}` for the build the node should run.
fn control(shared: &Shared, port: &Port, body: &[u8]) -> Result<Value> {
    let v: Value = serde_json::from_slice(body).context("the request body")?;
    let target = v.get("target").and_then(Value::as_str).context("no target")?;
    let command: Command = match v.get("firmware").and_then(Value::as_str) {
        Some(tag) => Command::Firmware(parse_tag(tag).with_context(|| format!("{tag}: not a build tag"))?),
        None => serde_json::from_value(v.get("command").cloned().context("no command")?).context("the command")?,
    };
    let record = {
        let mut s = shared.lock().expect("service");
        let id = s.resolve(target)?;
        let key = s.controller_key.context("no fleet key to sign with")?;
        auth::sign(&mut s.writer, &key, id, command)
    };
    let mut ports = port.lock().expect("port");
    if ports.is_empty() {
        bail!("no sink to send to (a stream, not a port)");
    }
    for port in ports.iter_mut() {
        send_record(&mut **port, &record)?;
    }
    Ok(json!({"sent": true, "version": format!("{}.{}", record.version.epoch, record.version.counter)}))
}

/// The survey window from the page: `{"window_s": 7200}`, 0 for every burst kept.
fn survey_window(shared: &Shared, body: &[u8]) -> Result<Value> {
    let v: Value = serde_json::from_slice(body).context("the request body")?;
    let window_s = v.get("window_s").and_then(Value::as_u64).context("no window_s")?;
    let mut s = shared.lock().expect("service");
    s.window_s = window_s;
    Ok(s.survey())
}

/// A pin from the page: `{"node": "N004", "x": 1.0, "y": 2.0, "z": 2.5}` or `{"node": "N004", "clear": true}`.
fn pin(shared: &Shared, body: &[u8]) -> Result<Value> {
    let v: Value = serde_json::from_slice(body).context("the request body")?;
    let node = v.get("node").and_then(Value::as_str).context("no node")?;
    let mut s = shared.lock().expect("service");
    let id = s.resolve(node)?;
    if v.get("clear").and_then(Value::as_bool) == Some(true) {
        s.pins.remove(&id);
    } else {
        let coord = |k: &str| {
            v.get(k)
                .and_then(Value::as_f64)
                .map(|x| x as f32)
                .with_context(|| format!("no {k}"))
        };
        let p = [coord("x")?, coord("y")?, coord("z")?];
        s.pins.insert(id, p);
    }
    s.save_pins();
    Ok(json!({"pins": s.pins.len()}))
}

fn percent_decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'%' if i + 2 < bytes.len() => {
                let hex = &s[i + 1..i + 3];
                match u8::from_str_radix(hex, 16) {
                    Ok(b) => {
                        out.push(b);
                        i += 3;
                        continue;
                    }
                    Err(_) => out.push(b'%'),
                }
            }
            b'+' => out.push(b' '),
            b => out.push(b),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use navgrid_mesh::{Link, NodeInfo, PointMm, Role};
    use navgrid_proto::{MAX_FRAME_LEN, MESH_RECORD_LEN, MeshRecord};

    fn service() -> Service {
        Service {
            replica: crate::mesh_view::host_replica(),
            registry: None,
            pins: BTreeMap::new(),
            pins_path: std::env::temp_dir().join(format!("navgrid-serve-test-{}.json", std::process::id())),
            recent: VecDeque::new(),
            nearby: BTreeMap::new(),
            frames: 0,
            last_solve: None,
            history: History::default(),
            survey_log: None,
            window_s: 0,
            writer: controller_writer(),
            controller_key: Some([7; 16]),
            started: Instant::now(),
            view: View::new(None, None),
            log: None,
            offset_m: 1.5,
            height_sigma_m: 0.3,
            min_quality: 100,
        }
    }

    /// A record as the sink mirrors it: a `Message::Record` frame without its terminator.
    fn frame(record: &Record) -> Vec<u8> {
        let mut rb = [0u8; MESH_RECORD_LEN];
        let bytes = postcard::to_slice(record, &mut rb).unwrap();
        let msg = Message::Record(MeshRecord {
            bytes: heapless::Vec::from_slice(bytes).unwrap(),
        });
        let mut buf = [0u8; MAX_FRAME_LEN];
        let f = navgrid_proto::encode(&msg, &mut buf).unwrap();
        f[..f.len() - 1].to_vec()
    }

    fn node(id: u64, pinned: Option<[f32; 3]>) -> Record {
        Writer::new(NodeId(id), 1).record(Body::Node(NodeInfo {
            role: Role::Anchor,
            number: Some(id as u16),
            firmware: navgrid_proto::Firmware::new(0, 1, 0, 1, false),
            config: 0,
            hops: 1,
            frame: 0,
            pinned: pinned.map(|p| PointMm {
                x: (p[0] * 1000.0) as i32,
                y: (p[1] * 1000.0) as i32,
                z: (p[2] * 1000.0) as i32,
            }),
            neighbours: Default::default(),
        }))
    }

    fn link(from: u64, to: u64, distance_m: f32) -> Record {
        let mut w = Writer::new(NodeId(from), 1);
        w.record(Body::Health(navgrid_mesh::Health {
            battery_mv: 2900,
            temperature_cdeg: 2200,
            uptime_s: 10,
            queued: 0,
            bursts: 0,
            procedures: 0,
            sessions: 0,
        }));
        w.record(Body::Link(Link {
            peer: NodeId(to),
            distance_mm: ((distance_m + 1.5) * 1000.0) as u32,
            sigma_mm: 80,
            quality: 220,
            antenna_paths: 4,
            procedures: 12,
            failures: 1,
        }))
    }

    fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    }

    #[test]
    fn frames_fill_the_state_and_the_solve_places_the_unpinned_node() {
        let pins = [[0.0, 0.0, 2.5], [6.0, 0.0, 2.5], [0.0, 6.0, 2.5]];
        let fourth = [6.0, 6.0, 2.5];
        let mut records = vec![
            node(1, Some(pins[0])),
            node(2, Some(pins[1])),
            node(3, Some(pins[2])),
            node(4, None),
        ];
        for (i, p) in pins.iter().enumerate() {
            records.push(link(4, 1 + i as u64, dist(fourth, *p)));
        }
        records.push(link(1, 2, dist(pins[0], pins[1])));
        records.push(link(1, 3, dist(pins[0], pins[2])));
        records.push(link(2, 3, dist(pins[1], pins[2])));
        let shared: Shared = Arc::new(Mutex::new(service()));
        for r in &records {
            shared.lock().unwrap().frame(&frame(r));
        }
        let state = shared.lock().unwrap().state();
        assert_eq!(state["frames"], 10);
        assert_eq!(state["nodes"].as_array().unwrap().len(), 4);
        assert_eq!(state["links"].as_array().unwrap().len(), 6);
        assert_eq!(state["nodes"][0]["pinned"]["x"], 0.0);
        assert_eq!(state["recent"].as_array().unwrap().len(), 10);

        let port: Port = Arc::new(Mutex::new(Vec::new()));
        let summary = solve_now(&shared, &port).unwrap();
        assert_eq!(summary["placed"], 4);
        let state = shared.lock().unwrap().state();
        let n4 = state["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .find(|n| n["id"] == "000000000004")
            .unwrap();
        let p = &n4["position"];
        let solved = [
            p["x"].as_f64().unwrap() as f32,
            p["y"].as_f64().unwrap() as f32,
            p["z"].as_f64().unwrap() as f32,
        ];
        assert!(dist(solved, fourth) < 0.3, "{solved:?}");

        // The pointer from the solved node to a pin, facing east.
        let pointer = shared
            .lock()
            .unwrap()
            .pointer("000000000004", "000000000001", 90.0)
            .unwrap();
        assert!((pointer["distance_m"].as_f64().unwrap() - 8.49).abs() < 0.3);
        assert!(
            pointer["relative_azimuth_deg"].as_f64().unwrap().abs() > 90.0,
            "behind, facing east: {pointer}"
        );

        // A pin from the page overrides, and clears.
        let s = &mut shared.lock().unwrap();
        s.pins.insert(NodeId(4), [6.0, 6.0, 2.5]);
        assert_eq!(s.state()["nodes"][3]["pinned"]["placed"], true);
        s.pins.remove(&NodeId(4));
        s.save_pins();
        let _ = std::fs::remove_file(&s.pins_path);
    }

    #[test]
    fn a_command_needs_a_sink_and_a_known_target() {
        let shared: Shared = Arc::new(Mutex::new(service()));
        let port: Port = Arc::new(Mutex::new(Vec::new()));
        let body = br#"{"target":"000000000004","command":{"Blink":{"times":2}}}"#;
        let err = control(&shared, &port, body).unwrap_err();
        assert!(format!("{err:#}").contains("no sink"), "{err:#}");
        let body = br#"{"target":"N004","command":{"Blink":{"times":2}}}"#;
        let err = control(&shared, &port, body).unwrap_err();
        assert!(format!("{err:#}").contains("no registry"), "{err:#}");
    }
}
