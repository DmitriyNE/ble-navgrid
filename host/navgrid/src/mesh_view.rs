//! `navgrid mesh`: the mesh state as a sink saw it (FR-S5), from the records it mirrored into
//! `navgrid monitor --json` logs: the nodes with their role, hops, neighbours, firmware and
//! health, the links with their distances, the positions, and the controller's commands with
//! what the nodes reported. The records go through a host replica, so the newest version of
//! each wins whatever the order in the logs.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Config, Heap, NodeId, Record, Replica, Status, Topic};
use serde_json::Value;

use crate::fleet::{Fleet, FleetArgs, Registry, label_name};

#[derive(clap::Args)]
pub struct MeshViewArgs {
    /// `navgrid monitor --json` logs, oldest first.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// Print the records as JSON lines instead of tables.
    #[arg(long)]
    json: bool,
    #[command(flatten)]
    fleet: FleetArgs,
}

/// A host replica (a sink's view) filled from the records in monitor logs; how many record
/// lines there were and how many were kept as newest.
pub fn replica_from_logs(files: &[PathBuf]) -> Result<(Replica<Heap>, usize, usize)> {
    let mut replica: Replica<Heap> = Replica::new(
        Config::new(HOST, CONTROLLER, true),
        1,
        navgrid_mesh::NodeInfo {
            role: navgrid_mesh::Role::Anchor,
            number: None,
            firmware: navgrid_proto::Firmware::new(0, 0, 0, 0, false),
            config: 0,
            hops: 0,
            frame: 0,
            pinned: None,
            neighbours: Default::default(),
        },
        0,
    );
    let mut taken = 0usize;
    let mut lines = 0usize;
    for path in files {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        for line in text.lines() {
            let Ok(v) = serde_json::from_str::<Value>(line) else {
                continue;
            };
            let Some(r) = v.get("Record") else {
                continue;
            };
            lines += 1;
            let record: Record = serde_json::from_value(r.clone()).context("a Record line")?;
            if let Ok(rx) = replica.receive(record, 0)
                && rx.kept()
            {
                taken += 1;
            }
        }
    }
    if lines == 0 {
        bail!("no Record lines in the logs: the sink mirrors them on protocol 6");
    }
    Ok((replica, lines, taken))
}

/// The host replica's own ID, kept out of the tables.
pub const HOST: NodeId = NodeId(0xffff_ffff_ffff);

/// A node's name: its fleet number when the registry knows it.
pub fn node_name(id: NodeId, registry: Option<&Registry>) -> String {
    if id == CONTROLLER {
        return "controller".into();
    }
    registry
        .and_then(|r| r.by_id(id))
        .map_or_else(|| format!("{:012x}", id.0), |d| label_name(d.number))
}

pub fn run(args: &MeshViewArgs) -> Result<()> {
    let registry = Fleet::open(&args.fleet).ok().and_then(|f| f.registry().ok());
    let (replica, lines, taken) = replica_from_logs(&args.files)?;
    let host = HOST;
    let name = |id: NodeId| -> String { node_name(id, registry.as_ref()) };
    if args.json {
        for s in replica.records() {
            if s.record().origin == host {
                continue;
            }
            println!("{}", serde_json::to_string(s.record())?);
        }
        return Ok(());
    }
    println!("{lines} record lines, {taken} kept as newest\n");
    let records: Vec<&Record> = replica
        .records()
        .iter()
        .map(|s| s.record())
        .filter(|r| r.origin != host)
        .collect();
    println!(
        "{:<8} {:<8} {:>4} {:<30} {:<18} {:<20} {:>5}",
        "node", "role", "hops", "neighbours", "firmware", "pinned at (m)", "age s"
    );
    for r in &records {
        if let Body::Node(n) = &r.body {
            let neighbours: Vec<String> = n.neighbours.iter().map(|&id| name(id)).collect();
            let pinned = n.pinned.map_or(String::new(), |p| {
                format!(
                    "{:.2} {:.2} {:.2}",
                    f64::from(p.x) / 1000.0,
                    f64::from(p.y) / 1000.0,
                    f64::from(p.z) / 1000.0
                )
            });
            println!(
                "{:<8} {:<8} {:>4} {:<30} {:<18} {:<20} {:>5}",
                name(r.origin),
                format!("{:?}", n.role),
                if n.hops == u8::MAX {
                    "-".into()
                } else {
                    n.hops.to_string()
                },
                neighbours.join(" "),
                n.firmware.to_string(),
                pinned,
                r.age_ms / 1000
            );
        }
    }
    println!(
        "\n{:<8} {:>7} {:>8} {:>8} {:>6} {:>5}",
        "node", "mV", "°C", "up s", "queue", "age s"
    );
    for r in &records {
        if let Body::Health(h) = &r.body {
            println!(
                "{:<8} {:>7} {:>8.2} {:>8} {:>6} {:>5}",
                name(r.origin),
                h.battery_mv,
                f64::from(h.temperature_cdeg) / 100.0,
                h.uptime_s,
                h.queued,
                r.age_ms / 1000
            );
        }
    }
    println!(
        "\n{:<8} {:<8} {:>8} {:>7} {:>7} {:>5} {:>5} {:>5}",
        "from", "to", "m", "σ m", "quality", "proc", "fail", "age s"
    );
    for r in &records {
        if let Body::Link(l) = &r.body {
            println!(
                "{:<8} {:<8} {:>8.2} {:>7.2} {:>7} {:>5} {:>5} {:>5}",
                name(r.origin),
                name(l.peer),
                f64::from(l.distance_mm) / 1000.0,
                f64::from(l.sigma_mm) / 1000.0,
                l.quality,
                l.procedures,
                l.failures,
                r.age_ms / 1000
            );
        }
    }
    let positions: Vec<&&Record> = records.iter().filter(|r| matches!(r.body, Body::Position(_))).collect();
    if !positions.is_empty() {
        println!(
            "\n{:<10} {:<8} {:>8} {:>8} {:>8} {:>6} {:>5}",
            "by", "of", "x m", "y m", "z m", "frame", "age s"
        );
        for r in positions {
            if let Body::Position(p) = &r.body {
                println!(
                    "{:<10} {:<8} {:>8.2} {:>8.2} {:>8.2} {:>6} {:>5}",
                    name(r.origin),
                    name(p.of),
                    f64::from(p.point.x) / 1000.0,
                    f64::from(p.point.y) / 1000.0,
                    f64::from(p.point.z) / 1000.0,
                    p.frame,
                    r.age_ms / 1000
                );
            }
        }
    }
    let desired: Vec<&&Record> = records.iter().filter(|r| matches!(r.body, Body::Desired(_))).collect();
    if !desired.is_empty() {
        println!("\n{:<8} {:<32} {:<12} reported", "target", "command", "version");
        for r in desired {
            if let Body::Desired(d) = &r.body {
                let reported = replica
                    .get(&navgrid_mesh::Key {
                        origin: d.target,
                        topic: Topic::Reported(d.command.setting()),
                    })
                    .and_then(|s| match &s.record().body {
                        Body::Reported(rep) if rep.applied == r.version => Some(match rep.status {
                            Status::Applied => "applied",
                            Status::Unsupported => "unsupported",
                            Status::Invalid => "invalid",
                        }),
                        Body::Reported(_) => Some("an older version"),
                        _ => None,
                    })
                    .unwrap_or("pending");
                println!(
                    "{:<8} {:<32} {:<12} {}",
                    name(d.target),
                    format!("{:?}", d.command),
                    format!("{}.{}", r.version.epoch, r.version.counter),
                    reported
                );
            }
        }
    }
    Ok(())
}
