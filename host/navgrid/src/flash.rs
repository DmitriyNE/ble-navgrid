//! `navgrid flash`: firmware onto devices through their probes, several at once, recording in the
//! registry which build each device got (architecture §13). The image says which build it is: the
//! firmware embeds `navgrid-firmware 0.1.0+1950d45d[-dirty]`.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use navgrid_proto::NodeId;

use crate::fleet::{Fleet, FleetArgs, label_name, today};
use crate::probe;

#[derive(clap::Args)]
pub struct FlashArgs {
    /// The firmware ELF, e.g. `firmware/target/thumbv8m.main-none-eabihf/release/navgrid-node`.
    elf: PathBuf,
    /// A probe to flash through, as `probe-rs list` shows it; repeat it to flash several devices
    /// at once.
    #[arg(long = "probe", required = true)]
    probes: Vec<String>,
    #[command(flatten)]
    fleet: FleetArgs,
}

const TAG: &[u8] = b"navgrid-firmware ";

/// The build an image says it is, from its embedded tag.
pub fn firmware_tag(image: &[u8]) -> Option<String> {
    let start = image.windows(TAG.len()).position(|w| w == TAG)? + TAG.len();
    let len = image[start..].iter().position(|&b| b == 0)?;
    let tag = std::str::from_utf8(&image[start..start + len]).ok()?;
    (!tag.is_empty() && tag.len() < 64).then(|| tag.to_owned())
}

pub fn run(args: &FlashArgs) -> Result<()> {
    let image = std::fs::read(&args.elf).with_context(|| format!("reading {}", args.elf.display()))?;
    let tag = firmware_tag(&image).context("no navgrid firmware tag in the image")?;
    println!(
        "flashing {tag} through {} probe(s); about 2 min at 100 kHz",
        args.probes.len()
    );
    let results: Vec<(String, Result<NodeId>)> = std::thread::scope(|s| {
        let jobs: Vec<_> = args
            .probes
            .iter()
            .map(|p| {
                let elf = &args.elf;
                s.spawn(move || {
                    let id = probe::read_id(Some(p))?;
                    probe::download(Some(p), elf, None)?;
                    probe::reset(Some(p))?;
                    Ok(id)
                })
            })
            .collect();
        args.probes
            .iter()
            .cloned()
            .zip(jobs.into_iter().map(|j| j.join().expect("flash thread panicked")))
            .collect()
    });
    let fleet = Fleet::open(&args.fleet).ok();
    let mut registry = fleet.as_ref().map(Fleet::registry).transpose()?;
    let mut failed = 0;
    for (p, result) in &results {
        match result {
            Ok(id) => {
                let device = registry
                    .as_mut()
                    .and_then(|r| r.devices.iter_mut().find(|d| d.id == *id));
                match device {
                    Some(d) => {
                        d.firmware = Some(tag.clone());
                        d.flashed = Some(today());
                        println!("{p}: {} {:012x} now runs {tag}", label_name(d.number), id.0);
                    }
                    None => println!("{p}: {:012x} now runs {tag} (not enrolled, so not recorded)", id.0),
                }
            }
            Err(e) => {
                failed += 1;
                println!("{p}: failed: {e:#}");
            }
        }
    }
    if let (Some(fleet), Some(registry)) = (&fleet, &registry) {
        fleet.save(registry)?;
    }
    if failed > 0 {
        bail!("{failed} of {} device(s) failed", results.len());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_tag_in_an_image() {
        let mut image = vec![0xffu8; 64];
        image.extend_from_slice(b"navgrid-firmware 0.1.0+1950d45d-dirty\0rest");
        assert_eq!(firmware_tag(&image).as_deref(), Some("0.1.0+1950d45d-dirty"));
        assert_eq!(firmware_tag(b"nothing here"), None);
    }
}
