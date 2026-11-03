//! `navgrid flash`: firmware onto devices through their probes, several at once, recording in the
//! registry which build each device got (architecture §13). The image says which build it is: the
//! firmware embeds `navgrid-firmware 0.1.0+1950d45d[-dirty]`.
//!
//! Images run under the bootloader: `--bootloader` writes it too, which a device needs once.
//! Every flash blanks the bootloader's state, so nothing left in the update slot gets swapped over
//! the image just written, and the staged and kept update headers: the image wasn't installed from
//! an update, so it has none to offer its neighbours.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use navgrid_proto::layout::{
    ACTIVE_HEADER, ACTIVE_HEADER_LEN, BOOTLOADER_STATE, BOOTLOADER_STATE_LEN, STAGED_UPDATE, STAGED_UPDATE_LEN,
};
use navgrid_proto::{Board, NodeId};

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
    /// Also write this bootloader ELF (`firmware/bootloader`), as a device without one needs.
    #[arg(long)]
    bootloader: Option<PathBuf>,
    #[command(flatten)]
    fleet: FleetArgs,
}

/// The build an image says it is, from its embedded tag.
pub fn firmware_tag(image: &[u8]) -> Option<String> {
    embedded(image, b"navgrid-firmware ")
}

/// The board an image is built for, from its embedded tag.
pub fn board_tag(image: &[u8]) -> Option<Board> {
    Board::from_name(&embedded(image, b"navgrid-board ")?)
}

/// The text after `marker` up to its NUL, as the firmware embeds its tags.
fn embedded(image: &[u8], marker: &[u8]) -> Option<String> {
    let start = image.windows(marker.len()).position(|w| w == marker)? + marker.len();
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
    // A blank bootloader state: no update pending, nothing to revert.
    let state = std::env::temp_dir().join(format!("navgrid-boot-state-{}.bin", std::process::id()));
    std::fs::write(&state, vec![0xffu8; BOOTLOADER_STATE_LEN as usize])
        .with_context(|| format!("writing {}", state.display()))?;
    // And blank update headers, kept and staged, which sit next to each other.
    assert_eq!(ACTIVE_HEADER + ACTIVE_HEADER_LEN, STAGED_UPDATE);
    let headers = std::env::temp_dir().join(format!("navgrid-headers-{}.bin", std::process::id()));
    std::fs::write(&headers, vec![0xffu8; (ACTIVE_HEADER_LEN + STAGED_UPDATE_LEN) as usize])
        .with_context(|| format!("writing {}", headers.display()))?;
    let results: Vec<(String, Result<NodeId>)> = std::thread::scope(|s| {
        let jobs: Vec<_> = args
            .probes
            .iter()
            .map(|p| {
                let (elf, bootloader, state, headers) = (&args.elf, &args.bootloader, &state, &headers);
                s.spawn(move || {
                    let id = probe::read_id(Some(p))?;
                    if let Some(bootloader) = bootloader {
                        probe::download(Some(p), bootloader, None)?;
                    }
                    probe::download(Some(p), state, Some(BOOTLOADER_STATE))?;
                    probe::download(Some(p), headers, Some(ACTIVE_HEADER))?;
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
    let _ = std::fs::remove_file(&state);
    let _ = std::fs::remove_file(&headers);
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

    #[test]
    fn finds_the_board_in_an_image() {
        let image = b"navgrid-firmware 0.1.0+1950d45d\0navgrid-board tag\0";
        assert_eq!(board_tag(image), Some(Board::Tag));
        assert_eq!(board_tag(b"navgrid-board nrf52\0"), None);
        assert_eq!(board_tag(b"navgrid-firmware 0.1.0+1950d45d\0"), None);
    }
}
