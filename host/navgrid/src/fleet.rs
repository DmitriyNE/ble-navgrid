//! The fleet directory (architecture §13): the fleet key, the device registry, and labels.
//!
//! ```text
//! fleet.key       the fleet key, 32 hex digits; secret, never in git
//! signing.key     the Ed25519 seed firmware updates are signed with (D-35), 64 hex digits;
//!                 secret, never in git
//! devices.json    every enrolled device: fleet number, node ID, role, enrolment date, status
//! labels/         one SVG label per device: fleet number and a QR code of its ID
//! builds/         the ELF of every build flashed or signed, by firmware tag and board
//!                 (`0.1.0+7ccdf3ca-tag.elf`): the monitor decodes devices' log frames and crash
//!                 addresses with it. A dirty build replaces an earlier one with the same tag.
//! ```

use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use navgrid_proto::{Board, NodeId, Role};
use serde::{Deserialize, Serialize};

/// Where the fleet lives: `--fleet`, else `NAVGRID_FLEET`, else `~/.config/navgrid`.
#[derive(clap::Args, Clone, Default)]
pub struct FleetArgs {
    #[arg(long = "fleet", env = "NAVGRID_FLEET")]
    dir: Option<PathBuf>,
}

impl FleetArgs {
    pub fn dir(&self) -> Result<PathBuf> {
        if let Some(dir) = &self.dir {
            return Ok(dir.clone());
        }
        let home = std::env::var_os("HOME").context("HOME is not set; pass --fleet")?;
        Ok(Path::new(&home).join(".config/navgrid"))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Status {
    Enrolled,
    Placed,
    Retired,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Device {
    pub number: u16,
    #[serde(with = "hex_id")]
    pub id: NodeId,
    pub role: Role,
    /// It also has a host link (a sink).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub sink: bool,
    /// Where it is, in words (FR-O4): "kitchen ceiling", "Dmitriy's keys".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// UTC date of the last enrolment.
    pub enrolled: String,
    pub status: Status,
    /// The build `navgrid flash` or `navgrid update` last wrote, or the device last reported to
    /// the monitor, and when.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmware: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flashed: Option<String>,
    /// It ranges: its links survey the anchors and fix positions. Off for a node that only routes
    /// (the DK sink, whose one biased antenna path spoils the survey).
    #[serde(default = "yes", skip_serializing_if = "is_yes")]
    pub ranging: bool,
}

fn yes() -> bool {
    true
}

fn is_yes(b: &bool) -> bool {
    *b
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Registry {
    pub devices: Vec<Device>,
}

impl Registry {
    pub fn by_number(&self, number: u16) -> Option<&Device> {
        self.devices
            .iter()
            .find(|d| d.number == number && d.status != Status::Retired)
    }

    /// Names a device and marks it placed (FR-O4).
    pub fn place(&mut self, number: u16, name: String) -> Result<()> {
        let d = self
            .devices
            .iter_mut()
            .find(|d| d.number == number && d.status != Status::Retired)
            .with_context(|| format!("{} is not enrolled", label_name(number)))?;
        d.name = Some(name);
        d.status = Status::Placed;
        Ok(())
    }

    /// Whether a device ranges for the survey and fixes, or only routes.
    pub fn set_ranging(&mut self, number: u16, ranging: bool) -> Result<NodeId> {
        let d = self
            .devices
            .iter_mut()
            .find(|d| d.number == number && d.status != Status::Retired)
            .with_context(|| format!("{} is not enrolled", label_name(number)))?;
        d.ranging = ranging;
        Ok(d.id)
    }

    /// Retires a device: its number is free again, and the mesh is told (the caller sends the
    /// retire record).
    pub fn retire(&mut self, number: u16) -> Result<NodeId> {
        let d = self
            .devices
            .iter_mut()
            .find(|d| d.number == number && d.status != Status::Retired)
            .with_context(|| format!("{} is not enrolled", label_name(number)))?;
        d.status = Status::Retired;
        Ok(d.id)
    }

    pub fn by_id(&self, id: NodeId) -> Option<&Device> {
        self.devices.iter().find(|d| d.id == id)
    }

    /// One more than the highest number ever given, so numbers are never reused.
    pub fn next_number(&self) -> u16 {
        self.devices.iter().map(|d| d.number).max().unwrap_or(0) + 1
    }

    /// Records an enrolment: updates the device's entry or adds one. A number another device
    /// holds is refused.
    pub fn enrol(&mut self, id: NodeId, number: u16, role: Role, sink: bool, date: String) -> Result<()> {
        // A retired device's number is free for its replacement.
        if let Some(other) = self
            .devices
            .iter()
            .find(|d| d.number == number && d.id != id && d.status != Status::Retired)
        {
            bail!("{} already belongs to {:012x}", label_name(number), other.id.0);
        }
        self.devices
            .retain(|d| !(d.number == number && d.id != id && d.status == Status::Retired));
        match self.devices.iter_mut().find(|d| d.id == id) {
            Some(d) => {
                d.number = number;
                d.role = role;
                d.sink = sink;
                d.enrolled = date;
                d.status = Status::Enrolled;
            }
            None => self.devices.push(Device {
                number,
                id,
                role,
                sink,
                name: None,
                enrolled: date,
                status: Status::Enrolled,
                firmware: None,
                flashed: None,
                ranging: true,
            }),
        }
        self.devices.sort_by_key(|d| d.number);
        Ok(())
    }
}

/// `N042`.
/// A kept build's file name in `builds/`.
pub fn build_file(firmware: &str, board: Board) -> String {
    format!("{firmware}-{}.elf", board.name())
}

pub fn label_name(number: u16) -> String {
    format!("N{number:03}")
}

pub struct Fleet {
    dir: PathBuf,
}

impl Fleet {
    pub fn open(args: &FleetArgs) -> Result<Self> {
        let dir = args.dir()?;
        if !dir.join("fleet.key").exists() {
            bail!("no fleet key in {}; run `navgrid fleet init` first", dir.display());
        }
        Ok(Self { dir })
    }

    /// Creates the directory, a fleet key and an empty registry, keeping any that exist.
    pub fn init(args: &FleetArgs) -> Result<Self> {
        let dir = args.dir()?;
        std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
        let fleet = Self { dir };
        for (file, len, what) in [("fleet.key", 16, "fleet key"), ("signing.key", 32, "signing key")] {
            let path = fleet.dir.join(file);
            if path.exists() {
                println!("keeping the {what} in {}", path.display());
            } else {
                new_key(&path, len)?;
                println!("wrote a new {what} to {}", path.display());
            }
        }
        if !fleet.registry_path().exists() {
            fleet.save(&Registry::default())?;
            println!("wrote an empty registry to {}", fleet.registry_path().display());
        }
        Ok(fleet)
    }

    pub fn key(&self) -> Result<[u8; 16]> {
        read_key(&self.dir.join("fleet.key"))
    }

    /// The key updates are signed with; `fleet init` creates it.
    pub fn signing_key(&self) -> Result<ed25519_dalek::SigningKey> {
        let seed: [u8; 32] = read_key(&self.dir.join("signing.key"))?;
        Ok(ed25519_dalek::SigningKey::from_bytes(&seed))
    }

    fn registry_path(&self) -> PathBuf {
        self.dir.join("devices.json")
    }

    pub fn registry(&self) -> Result<Registry> {
        let path = self.registry_path();
        if !path.exists() {
            return Ok(Registry::default());
        }
        let text = std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
        let mut value: serde_json::Value =
            serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
        // Registries from before the sink flag: the sink was a role.
        if let Some(devices) = value.get_mut("devices").and_then(serde_json::Value::as_array_mut) {
            for d in devices
                .iter_mut()
                .filter(|d| d.get("role").and_then(serde_json::Value::as_str) == Some("Sink"))
            {
                d["role"] = "Anchor".into();
                d["sink"] = true.into();
            }
        }
        serde_json::from_value(value).with_context(|| format!("parsing {}", path.display()))
    }

    pub fn save(&self, registry: &Registry) -> Result<()> {
        let path = self.registry_path();
        let text = serde_json::to_string_pretty(registry)? + "\n";
        std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))
    }

    /// Keeps a build's ELF, so the monitor can decode what devices running it report.
    pub fn keep_build(&self, firmware: &str, board: Board, elf: &[u8]) -> Result<PathBuf> {
        let dir = self.builds_dir();
        std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
        let path = dir.join(build_file(firmware, board));
        std::fs::write(&path, elf).with_context(|| format!("writing {}", path.display()))?;
        Ok(path)
    }

    pub fn builds_dir(&self) -> PathBuf {
        self.dir.join("builds")
    }

    /// Writes the device's label and returns its path.
    pub fn write_label(&self, number: u16, id: NodeId) -> Result<PathBuf> {
        let dir = self.dir.join("labels");
        std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
        let path = dir.join(format!("{}.svg", label_name(number)));
        std::fs::write(&path, label_svg(number, id)?).with_context(|| format!("writing {}", path.display()))?;
        Ok(path)
    }
}

/// A 50 × 25 mm label: a QR code of `NAVGRID:N042:CE36DFCC4C2D` (QR alphanumeric mode), the
/// fleet number large, and the node ID small.
pub fn label_svg(number: u16, id: NodeId) -> Result<String> {
    let name = label_name(number);
    let payload = format!("NAVGRID:{name}:{:012X}", id.0);
    let code = qrcode::QrCode::with_error_correction_level(&payload, qrcode::EcLevel::M)?;
    let width = code.width();
    // Modules plus a 4-module quiet zone, scaled into 21 mm.
    let module = 21.0 / (width + 8) as f64;
    let mut rects = String::new();
    for (i, color) in code.to_colors().iter().enumerate() {
        if *color == qrcode::Color::Dark {
            let (x, y) = (i % width, i / width);
            rects += &format!(
                r#"<rect x="{:.3}" y="{:.3}" width="{module:.3}" height="{module:.3}"/>"#,
                2.0 + (x + 4) as f64 * module,
                2.0 + (y + 4) as f64 * module
            );
        }
    }
    Ok(format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="50mm" height="25mm" viewBox="0 0 50 25">
<rect width="50" height="25" fill="white"/>
<g fill="black" shape-rendering="crispEdges">{rects}</g>
<text x="25" y="13" font-family="Helvetica, Arial, sans-serif" font-size="9" font-weight="bold">{name}</text>
<text x="25" y="19" font-family="Menlo, monospace" font-size="2.6">{:012x}</text>
<text x="25" y="22.5" font-family="Helvetica, Arial, sans-serif" font-size="2.2">navgrid</text>
</svg>
"#,
        id.0
    ))
}

/// Reads a key file: `N` bytes as hex.
fn read_key<const N: usize>(path: &Path) -> Result<[u8; N]> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let hex = text.trim();
    if hex.len() != 2 * N {
        bail!("{}: expected {} hex digits", path.display(), 2 * N);
    }
    let mut key = [0u8; N];
    for (i, b) in key.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).with_context(|| format!("{}: not hex", path.display()))?;
    }
    Ok(key)
}

/// A new random key of `len` bytes, readable by its owner only.
fn new_key(path: &Path, len: usize) -> Result<()> {
    use std::os::unix::fs::OpenOptionsExt;
    let mut key = vec![0u8; len];
    std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut key))
        .context("reading /dev/urandom")?;
    let hex: String = key.iter().map(|b| format!("{b:02x}")).collect();
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)
        .with_context(|| format!("creating {}", path.display()))?;
    writeln!(file, "{hex}").with_context(|| format!("writing {}", path.display()))
}

/// Today's date in UTC, `YYYY-MM-DD`.
pub fn today() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    civil_date(secs / 86_400)
}

/// The proleptic Gregorian date of a day count since 1970-01-01 (Howard Hinnant's algorithm).
fn civil_date(days: u64) -> String {
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}")
}

pub fn list(args: &FleetArgs) -> Result<()> {
    let fleet = Fleet::open(args)?;
    let registry = fleet.registry()?;
    if registry.devices.is_empty() {
        println!("no devices enrolled in {}", fleet.dir.display());
    }
    for d in &registry.devices {
        let firmware = match (&d.firmware, &d.flashed) {
            (Some(f), Some(on)) => format!("{f} (flashed {on})"),
            _ => "firmware unknown".into(),
        };
        println!(
            "{}  {:012x}  {:<13}  {:<8}  enrolment {}  {firmware}{}",
            label_name(d.number),
            d.id.0,
            format!("{:?}{}", d.role, if d.sink { " + sink" } else { "" }),
            format!("{:?}", d.status).to_lowercase(),
            d.enrolled,
            d.name.as_ref().map_or(String::new(), |n| format!("  \"{n}\""))
        );
        if !d.ranging {
            println!("      routes only: no ranging for the survey or fixes");
        }
    }
    Ok(())
}

/// Node IDs as 12 hex digits, as devices log them.
mod hex_id {
    use navgrid_proto::NodeId;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(id: &NodeId, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&format!("{:012x}", id.0))
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<NodeId, D::Error> {
        let s = String::deserialize(d)?;
        u64::from_str_radix(&s, 16)
            .map(NodeId)
            .map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_are_kept_and_never_reused() {
        let mut r = Registry::default();
        let (a, b) = (NodeId(0xc000_0000_0001), NodeId(0xc000_0000_0002));
        r.enrol(a, r.next_number(), Role::Anchor, false, "2026-11-02".into())
            .unwrap();
        r.enrol(b, r.next_number(), Role::Anchor, true, "2026-11-02".into())
            .unwrap();
        assert_eq!(r.by_id(b).unwrap().number, 2);
        // Re-enrolling keeps the entry; taking another device's number is refused.
        r.enrol(a, 1, Role::Asset, false, "2026-11-03".into()).unwrap();
        assert_eq!(r.devices.len(), 2);
        assert_eq!(r.by_id(a).unwrap().role, Role::Asset);
        assert!(r.enrol(a, 2, Role::Anchor, false, "2026-11-03".into()).is_err());
        let back: Registry = serde_json::from_str(&serde_json::to_string(&r).unwrap()).unwrap();
        assert_eq!(back, r);
        assert!(serde_json::to_string(&r).unwrap().contains("\"c00000000001\""));
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil_date(0), "1970-01-01");
        assert_eq!(civil_date(20_394), "2025-11-02");
        assert_eq!(civil_date(11_016), "2000-02-29");
    }

    #[test]
    fn label_has_number_id_and_qr() {
        let svg = label_svg(42, NodeId(0xce36_dfcc_4c2d)).unwrap();
        assert!(svg.contains(">N042<") && svg.contains(">ce36dfcc4c2d<"));
        assert!(svg.matches("<rect").count() > 100);
    }
}

#[derive(clap::Args)]
pub struct PlaceArgs {
    /// The device's fleet number (`N042`).
    number: String,
    /// Where it is, in words.
    name: String,
    #[command(flatten)]
    fleet: FleetArgs,
}

pub fn place(args: &PlaceArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let mut registry = fleet.registry()?;
    let number = parse_number(&args.number)?;
    registry.place(number, args.name.clone())?;
    fleet.save(&registry)?;
    println!("{} placed: \"{}\"", label_name(number), args.name);
    Ok(())
}

#[derive(clap::Args)]
pub struct RangingArgs {
    /// The device's fleet number (`N003`).
    number: String,
    /// `on`: its links survey and fix positions; `off`: it only routes.
    #[arg(value_parser = clap::builder::BoolishValueParser::new(), action = clap::ArgAction::Set)]
    on: bool,
    #[command(flatten)]
    fleet: FleetArgs,
}

pub fn ranging(args: &RangingArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let mut registry = fleet.registry()?;
    let number = parse_number(&args.number)?;
    registry.set_ranging(number, args.on)?;
    fleet.save(&registry)?;
    println!(
        "{}: {}",
        label_name(number),
        if args.on {
            "ranges for the survey and fixes"
        } else {
            "routes only; the survey and fixes leave its links out"
        }
    );
    Ok(())
}

#[derive(clap::Args)]
pub struct RetireArgs {
    /// The device's fleet number (`N042`).
    number: String,
    /// A sink's serial port, to tell the mesh; without it only the registry changes.
    #[arg(long)]
    sink: Option<String>,
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    #[command(flatten)]
    fleet: FleetArgs,
}

pub fn retire(args: &RetireArgs) -> Result<()> {
    let fleet = Fleet::open(&args.fleet)?;
    let mut registry = fleet.registry()?;
    let number = parse_number(&args.number)?;
    let id = registry.retire(number)?;
    fleet.save(&registry)?;
    println!("{} retired in the registry ({:012x})", label_name(number), id.0);
    let Some(port) = &args.sink else {
        println!("no --sink: the mesh keeps its records until one is told");
        return Ok(());
    };
    let key = navgrid_mesh::auth::controller_key(&fleet.key()?);
    let mut writer = crate::solve::controller_writer();
    let record = navgrid_mesh::auth::sign_retire(&mut writer, &key, id);
    let mut port = serialport::new(port, args.baud)
        .timeout(std::time::Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening {port}"))?;
    crate::solve::send_record(&mut *port, &record)?;
    println!(
        "retire record v{}.{} sent to the sink",
        record.version.epoch, record.version.counter
    );
    Ok(())
}

fn parse_number(s: &str) -> Result<u16> {
    s.strip_prefix('N')
        .unwrap_or(s)
        .parse::<u16>()
        .with_context(|| format!("{s}: not a fleet number like N042"))
}
