//! `navgrid sign` and `navgrid update`: signed firmware updates (D-35, architecture §13).
//!
//! An update file is an [`UpdateHeader`] followed by the raw image for the active slot. The
//! signature is Ed25519 over the image's SHA-512 digest, as embassy-boot verifies it on the device.
//! `update` stages one through the probe: the image into the update slot, the header into the
//! staging page. The device checks it at boot and has the bootloader swap it in.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use ed25519_dalek::Signer;
use navgrid_proto::layout::{ACTIVE, ACTIVE_LEN, DFU, STAGED_UPDATE};
use navgrid_proto::{Firmware, UPDATE_HEADER_LEN, UpdateHeader};
use sha2::{Digest, Sha512};

use crate::fleet::{Fleet, FleetArgs};
use crate::{flash, probe};

#[derive(clap::Args)]
pub struct SignArgs {
    /// The firmware ELF, built for the active slot.
    elf: PathBuf,
    /// Where to write the update file.
    #[arg(long)]
    out: PathBuf,
    #[command(flatten)]
    fleet: FleetArgs,
}

#[derive(clap::Args)]
pub struct UpdateArgs {
    /// A signed update file, from `navgrid sign`.
    file: PathBuf,
    /// The probe of the device to update.
    #[arg(long)]
    probe: String,
    #[command(flatten)]
    fleet: FleetArgs,
}

/// `0.1.0+0cf980a7-dirty` → its firmware ID.
pub fn parse_tag(tag: &str) -> Option<Firmware> {
    let (semver, rest) = tag.split_once('+')?;
    let (commit, dirty) = match rest.strip_suffix("-dirty") {
        Some(c) => (c, true),
        None => (rest, false),
    };
    let mut parts = semver.split('.').map(|p| p.parse::<u8>().ok());
    let (major, minor, patch) = (parts.next()??, parts.next()??, parts.next()??);
    Some(Firmware::new(
        major,
        minor,
        patch,
        u32::from_str_radix(commit, 16).ok()?,
        dirty,
    ))
}

/// The bytes an ELF loads, from its lowest load address to its highest, gaps erased (0xFF), and
/// that lowest address. Little-endian ELF32 only, as the firmware is.
pub fn load_image(elf: &[u8]) -> Result<(u32, Vec<u8>)> {
    if elf.get(..6) != Some(&[0x7f, b'E', b'L', b'F', 1, 1][..]) {
        bail!("not a little-endian 32-bit ELF");
    }
    let u16_at = |i: usize| u16::from_le_bytes([elf[i], elf[i + 1]]);
    let u32_at = |i: usize| u32::from_le_bytes([elf[i], elf[i + 1], elf[i + 2], elf[i + 3]]);
    let (phoff, phentsize, phnum) = (u32_at(28) as usize, usize::from(u16_at(42)), usize::from(u16_at(44)));
    let mut segments = Vec::new();
    for i in 0..phnum {
        let h = phoff + i * phentsize;
        if elf.len() < h + 32 {
            bail!("program header {i} is cut short");
        }
        // PT_LOAD with bytes in the file; the load address is the physical one.
        let (kind, offset, paddr, filesz) = (u32_at(h), u32_at(h + 4) as usize, u32_at(h + 12), u32_at(h + 16));
        if kind == 1 && filesz > 0 {
            let bytes = elf
                .get(offset..offset + filesz as usize)
                .context("a segment lies outside the file")?;
            segments.push((paddr, bytes));
        }
    }
    let start = segments.iter().map(|s| s.0).min().context("nothing to load")?;
    let end = segments.iter().map(|s| s.0 + s.1.len() as u32).max().unwrap_or(start);
    let mut image = vec![0xffu8; (end - start) as usize];
    for (addr, bytes) in segments {
        let at = (addr - start) as usize;
        image[at..at + bytes.len()].copy_from_slice(bytes);
    }
    Ok((start, image))
}

/// The update file for an image: its header, then the image.
pub fn sign(image: &[u8], firmware: Firmware, key: &ed25519_dalek::SigningKey) -> Vec<u8> {
    let digest = Sha512::digest(image);
    let header = UpdateHeader {
        firmware,
        len: image.len() as u32,
        signature: key.sign(&digest).to_bytes(),
    };
    let mut file = header.encode().to_vec();
    file.extend_from_slice(image);
    file
}

pub fn run_sign(args: &SignArgs) -> Result<()> {
    let elf = std::fs::read(&args.elf).with_context(|| format!("reading {}", args.elf.display()))?;
    let tag = flash::firmware_tag(&elf).context("no navgrid firmware tag in the image")?;
    let firmware = parse_tag(&tag).with_context(|| format!("can't read the firmware tag {tag:?}"))?;
    let (start, image) = load_image(&elf)?;
    if start != ACTIVE {
        bail!("the image starts at {start:#x}, not the active slot ({ACTIVE:#x}): built for the bootloader?");
    }
    if image.len() > ACTIVE_LEN as usize {
        bail!(
            "the image ({} bytes) doesn't fit the active slot ({ACTIVE_LEN})",
            image.len()
        );
    }
    let key = Fleet::open(&args.fleet)?.signing_key()?;
    let file = sign(&image, firmware, &key);
    std::fs::write(&args.out, &file).with_context(|| format!("writing {}", args.out.display()))?;
    println!("{}: {firmware}, {} bytes, signed", args.out.display(), image.len());
    Ok(())
}

/// Splits an update file into its header and image, checking they agree.
pub fn read_update(path: &Path) -> Result<(UpdateHeader, Vec<u8>)> {
    let file = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let header =
        UpdateHeader::decode(&file).map_err(|e| anyhow::anyhow!("{}: bad update header: {e:?}", path.display()))?;
    let image = file[UPDATE_HEADER_LEN..].to_vec();
    if image.len() != header.len as usize {
        bail!(
            "{}: the header says {} bytes, the file holds {}",
            path.display(),
            header.len,
            image.len()
        );
    }
    Ok((header, image))
}

pub fn run_update(args: &UpdateArgs) -> Result<()> {
    let (header, image) = read_update(&args.file)?;
    let probe = Some(args.probe.as_str());
    let id = probe::read_id(probe)?;
    println!(
        "staging {} ({} bytes) on {:012x}; about {} s at 100 kHz",
        header.firmware,
        header.len,
        id.0,
        image.len() / 3000
    );
    let tmp = std::env::temp_dir();
    let (image_file, header_file) = (
        tmp.join(format!("navgrid-update-{}.bin", std::process::id())),
        tmp.join(format!("navgrid-update-{}.hdr", std::process::id())),
    );
    std::fs::write(&image_file, &image)?;
    std::fs::write(&header_file, header.encode())?;
    let staged = probe::download(probe, &image_file, Some(DFU))
        .and_then(|()| probe::download(probe, &header_file, Some(STAGED_UPDATE)))
        .and_then(|()| probe::reset(probe));
    let _ = std::fs::remove_file(&image_file);
    let _ = std::fs::remove_file(&header_file);
    staged?;
    println!("staged; the device checks the signature at boot, then restarts into the update");
    // Recorded as what the device runs; if it rejects the update, the monitor shows the mismatch.
    if let Ok(fleet) = Fleet::open(&args.fleet) {
        let mut registry = fleet.registry()?;
        if let Some(d) = registry.devices.iter_mut().find(|d| d.id == id) {
            d.firmware = Some(header.firmware.to_string());
            d.flashed = Some(crate::fleet::today());
            fleet.save(&registry)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_firmware_tags() {
        assert_eq!(
            parse_tag("0.1.2+0cf980a7-dirty"),
            Some(Firmware::new(0, 1, 2, 0x0cf9_80a7, true))
        );
        assert_eq!(parse_tag("1.0.0+00000001"), Some(Firmware::new(1, 0, 0, 1, false)));
        assert_eq!(parse_tag("1.0+00000001"), None);
    }

    #[test]
    fn signature_verifies_over_the_digest() {
        let key = ed25519_dalek::SigningKey::from_bytes(&[7; 32]);
        let image = vec![0xa5u8; 5000];
        let file = sign(&image, Firmware::new(0, 1, 0, 1, false), &key);
        let header = UpdateHeader::decode(&file).unwrap();
        assert_eq!(&file[UPDATE_HEADER_LEN..], &image[..]);
        let digest = Sha512::digest(&image);
        let signature = ed25519_dalek::Signature::from_bytes(&header.signature);
        assert!(key.verifying_key().verify_strict(&digest, &signature).is_ok());
    }

    #[test]
    fn loads_elf_segments() {
        // An ELF32 with two PT_LOAD segments, 0x10 bytes apart.
        let mut elf = vec![0u8; 0x100];
        elf[..6].copy_from_slice(&[0x7f, b'E', b'L', b'F', 1, 1]);
        elf[28..32].copy_from_slice(&52u32.to_le_bytes());
        elf[42..44].copy_from_slice(&32u16.to_le_bytes());
        elf[44..46].copy_from_slice(&2u16.to_le_bytes());
        for (i, (offset, paddr, len)) in [(0x80u32, 0xb000u32, 4u32), (0x90, 0xb010, 2)].into_iter().enumerate() {
            let h = 52 + 32 * i;
            elf[h..h + 4].copy_from_slice(&1u32.to_le_bytes());
            elf[h + 4..h + 8].copy_from_slice(&offset.to_le_bytes());
            elf[h + 12..h + 16].copy_from_slice(&paddr.to_le_bytes());
            elf[h + 16..h + 20].copy_from_slice(&len.to_le_bytes());
            elf[offset as usize..(offset + len) as usize].fill(0x11 * (i as u8 + 1));
        }
        let (start, image) = load_image(&elf).unwrap();
        assert_eq!(start, 0xb000);
        assert_eq!(image.len(), 0x12);
        assert_eq!(&image[..4], &[0x11; 4]);
        assert_eq!(&image[4..0x10], &[0xff; 12]);
        assert_eq!(&image[0x10..], &[0x22; 2]);
    }
}
