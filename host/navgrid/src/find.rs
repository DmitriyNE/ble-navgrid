//! `navgrid find`: the find-my-keys pointer (FR-F3) from the mesh state a sink mirrored: the
//! seeker's latest position (its own fix, or the controller's position of it) to the asset's
//! last published one, as a bearing relative to the seeker's heading, an elevation and a
//! distance. The host view stands in for the HUD (Q13).

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use navgrid_mesh::{Body, CONTROLLER, Heap, Key, NodeId, Record, Replica, Topic};
use navgrid_solver::frame::{compass_to_heading, pointer};

use crate::fleet::{Fleet, FleetArgs};
use crate::mesh_view::{node_name, replica_from_logs};

#[derive(clap::Args)]
pub struct FindArgs {
    /// `navgrid monitor --json` logs, oldest first.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// The seeker: a mobile tag's fleet number (`N009`) or node ID.
    #[arg(long)]
    from: String,
    /// The asset: its fleet number or node ID.
    #[arg(long)]
    to: String,
    /// The seeker's compass heading, degrees clockwise from north (the HUD's sensor).
    #[arg(long, default_value_t = 0.0)]
    heading_deg: f32,
    #[command(flatten)]
    fleet: FleetArgs,
}

pub fn run(args: &FindArgs) -> Result<()> {
    let registry = Fleet::open(&args.fleet).ok().and_then(|f| f.registry().ok());
    let (replica, _, _) = replica_from_logs(&args.files)?;
    let name = |id: NodeId| node_name(id, registry.as_ref());
    let resolve = |s: &str| -> Result<NodeId> {
        if let Some(number) = s.strip_prefix('N').and_then(|n| n.parse::<u16>().ok())
            && let Some(d) = registry
                .as_ref()
                .and_then(|r| r.devices.iter().find(|d| d.number == number))
        {
            return Ok(d.id);
        }
        if s.len() == 12
            && let Ok(id) = u64::from_str_radix(s, 16)
        {
            return Ok(NodeId(id));
        }
        bail!("{s}: not a fleet number in the registry nor a node ID")
    };
    let from = resolve(&args.from)?;
    let to = resolve(&args.to)?;
    let (seeker, seeker_how) =
        position_of(&replica, from).with_context(|| format!("{} has no position yet", name(from)))?;
    let (asset, asset_how) = position_of(&replica, to).with_context(|| format!("{} has no position yet", name(to)))?;
    let heading = compass_to_heading(args.heading_deg.to_radians());
    let p = pointer(seeker.0, asset.0, heading);
    let side = if p.relative_azimuth_rad > 0.0 { "left" } else { "right" };
    println!(
        "{} is at {:.2} {:.2} {:.2} m ({seeker_how}, {} s old); {} at {:.2} {:.2} {:.2} m ({asset_how}, {} s old)",
        name(from),
        seeker.0[0],
        seeker.0[1],
        seeker.0[2],
        seeker.1 / 1000,
        name(to),
        asset.0[0],
        asset.0[1],
        asset.0[2],
        asset.1 / 1000
    );
    println!(
        "pointer: {:.1}° to the {side}, {:.1}° {}, {:.2} m away",
        p.relative_azimuth_rad.abs().to_degrees(),
        p.elevation_rad.abs().to_degrees(),
        if p.elevation_rad >= 0.0 { "up" } else { "down" },
        p.distance_m
    );
    Ok(())
}

/// A node's latest position: its own fix if it has one, else the controller's; with how it was
/// got and its age.
fn position_of(replica: &Replica<Heap>, node: NodeId) -> Option<(([f32; 3], u32), &'static str)> {
    let own = replica.get(&Key {
        origin: node,
        topic: Topic::Position(node),
    });
    let controller = replica.get(&Key {
        origin: CONTROLLER,
        topic: Topic::Position(node),
    });
    let point = |r: &Record| match &r.body {
        Body::Position(p) => Some((
            [
                p.point.x as f32 / 1000.0,
                p.point.y as f32 / 1000.0,
                p.point.z as f32 / 1000.0,
            ],
            r.age_ms,
        )),
        _ => None,
    };
    own.and_then(|s| point(s.record()))
        .map(|p| (p, "its own fix"))
        .or_else(|| {
            controller
                .and_then(|s| point(s.record()))
                .map(|p| (p, "the controller's solve"))
        })
}
