//! The fleet directory (architecture §13): the fleet key, the device registry, and labels.
//!
//! ```text
//! fleet.key       the fleet key, 32 hex digits; secret, never in git
//! devices.json    every enrolled device: fleet number, node ID, role, enrolment date, status
//! labels/         one SVG label per device: fleet number and a QR code of its ID
//! ```

use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use navgrid_proto::{NodeId, Role};
use serde::{Deserialize, Serialize};

/// Where the fleet lives: `--fleet`, else `NAVGRID_FLEET`, else `~/.config/navgrid`.
#[derive(clap::Args, Clone)]
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
    /// UTC date of the last enrolment.
    pub enrolled: String,
    pub status: Status,
    /// The build `navgrid flash` last wrote, and when.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmware: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flashed: Option<String>,
}

#[derive(Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Registry {
    pub devices: Vec<Device>,
}

impl Registry {
    pub fn by_id(&self, id: NodeId) -> Option<&Device> {
        self.devices.iter().find(|d| d.id == id)
    }

    /// One more than the highest number ever given, so numbers are never reused.
    pub fn next_number(&self) -> u16 {
        self.devices.iter().map(|d| d.number).max().unwrap_or(0) + 1
    }

    /// Records an enrolment: updates the device's entry or adds one. A number another device
    /// holds is refused.
    pub fn enrol(&mut self, id: NodeId, number: u16, role: Role, date: String) -> Result<()> {
        if let Some(other) = self.devices.iter().find(|d| d.number == number && d.id != id) {
            bail!("{} already belongs to {:012x}", label_name(number), other.id.0);
        }
        match self.devices.iter_mut().find(|d| d.id == id) {
            Some(d) => {
                d.number = number;
                d.role = role;
                d.enrolled = date;
                d.status = Status::Enrolled;
            }
            None => self.devices.push(Device {
                number,
                id,
                role,
                enrolled: date,
                status: Status::Enrolled,
                firmware: None,
                flashed: None,
            }),
        }
        self.devices.sort_by_key(|d| d.number);
        Ok(())
    }
}

/// `N042`.
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
        let key = fleet.dir.join("fleet.key");
        if key.exists() {
            println!("keeping the fleet key in {}", key.display());
        } else {
            new_key(&key)?;
            println!("wrote a new fleet key to {}", key.display());
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

    fn registry_path(&self) -> PathBuf {
        self.dir.join("devices.json")
    }

    pub fn registry(&self) -> Result<Registry> {
        let path = self.registry_path();
        if !path.exists() {
            return Ok(Registry::default());
        }
        let text = std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
        serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))
    }

    pub fn save(&self, registry: &Registry) -> Result<()> {
        let path = self.registry_path();
        let text = serde_json::to_string_pretty(registry)? + "\n";
        std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))
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

/// Reads a fleet key file: 32 hex digits.
fn read_key(path: &Path) -> Result<[u8; 16]> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let hex = text.trim();
    if hex.len() != 32 {
        bail!("{}: a fleet key is 32 hex digits", path.display());
    }
    let mut key = [0u8; 16];
    for (i, b) in key.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).with_context(|| format!("{}: not hex", path.display()))?;
    }
    Ok(key)
}

/// A new random fleet key, readable by its owner only.
fn new_key(path: &Path) -> Result<()> {
    use std::os::unix::fs::OpenOptionsExt;
    let mut key = [0u8; 16];
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
            "{}  {:012x}  {:<6}  {:<8}  enrolment {}  {firmware}",
            label_name(d.number),
            d.id.0,
            format!("{:?}", d.role),
            format!("{:?}", d.status).to_lowercase(),
            d.enrolled
        );
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
        r.enrol(a, r.next_number(), Role::Node, "2026-11-02".into()).unwrap();
        r.enrol(b, r.next_number(), Role::Sink, "2026-11-02".into()).unwrap();
        assert_eq!(r.by_id(b).unwrap().number, 2);
        // Re-enrolling keeps the entry; taking another device's number is refused.
        r.enrol(a, 1, Role::Asset, "2026-11-03".into()).unwrap();
        assert_eq!(r.devices.len(), 2);
        assert_eq!(r.by_id(a).unwrap().role, Role::Asset);
        assert!(r.enrol(a, 2, Role::Node, "2026-11-03".into()).is_err());
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
