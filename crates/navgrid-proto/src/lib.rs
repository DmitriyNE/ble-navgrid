//! Wire messages shared by firmware and host.
//!
//! Messages are postcard-encoded. On byte streams (UART, RTT, capture files) each message is one
//! COBS frame terminated by `0x00`. Lengths and coordinates are integer millimetres so firmware
//! never formats floats. The host's frames to a sink are [`UpdateMsg`]s ([`encode_downlink`]).

#![no_std]

#[cfg(test)]
extern crate std;

use serde::{Deserialize, Serialize};

/// Bumped on any incompatible change to [`Message`] or to what a sink takes from the host.
pub const PROTOCOL_VERSION: u16 = 6;

/// Upper bound for one encoded frame, including COBS overhead and the `0x00` terminator.
pub const MAX_FRAME_LEN: usize = 192;

/// L2CAP LE credit-based channel (SPSM) that carries telemetry toward a sink (D-6). Each SDU is
/// one encoded frame, terminator included.
pub const TELEMETRY_SPSM: u16 = 0x0081;

/// Device identity: the 48-bit BLE device address in the low bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NodeId(pub u64);

impl NodeId {
    /// A device's ID from its FICR `DEVICEADDR` words: the 48-bit address made a static random
    /// one (top two bits set), as the firmware uses it on air.
    pub fn from_ficr(deviceaddr: [u32; 2]) -> Self {
        let raw = (u64::from(deviceaddr[1]) << 32) | u64::from(deviceaddr[0]);
        Self((raw & 0xffff_ffff_ffff) | (0xc0 << 40))
    }

    /// The device address, least significant byte first.
    pub fn address(self) -> [u8; 6] {
        let mut addr = [0u8; 6];
        addr.copy_from_slice(&self.0.to_le_bytes()[..6]);
        addr
    }
}

/// A firmware build: its version, the git commit it was built from, and whether the tree had
/// uncommitted changes. Shown as `0.1.0+1950d45d-dirty`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Firmware {
    /// Major, minor and patch in the low three bytes.
    pub version: u32,
    /// The first 32 bits of the commit hash.
    pub commit: u32,
    pub dirty: bool,
}

impl Firmware {
    pub const fn new(major: u8, minor: u8, patch: u8, commit: u32, dirty: bool) -> Self {
        Self {
            version: ((major as u32) << 16) | ((minor as u32) << 8) | patch as u32,
            commit,
            dirty,
        }
    }

    pub fn semver(&self) -> (u8, u8, u8) {
        (
            (self.version >> 16) as u8,
            (self.version >> 8) as u8,
            self.version as u8,
        )
    }
}

impl core::fmt::Display for Firmware {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let (major, minor, patch) = self.semver();
        write!(f, "{major}.{minor}.{patch}+{:08x}", self.commit)?;
        if self.dirty {
            f.write_str("-dirty")?;
        }
        Ok(())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Firmware {
    fn format(&self, f: defmt::Formatter) {
        let (major, minor, patch) = self.semver();
        let dirty = if self.dirty { "-dirty" } else { "" };
        defmt::write!(
            f,
            "{=u8}.{=u8}.{=u8}+{=u32:08x}{=str}",
            major,
            minor,
            patch,
            self.commit,
            dirty
        );
    }
}

/// The controller (the host) as the origin of mesh records. No device has this ID: static random
/// addresses have their top two bits set.
pub const CONTROLLER: NodeId = NodeId(0);

/// What a device does in positioning. Numbered as in a [`Beacon`]. Any role can also be a sink
/// ([`NodeConfig::sink`]): a device with a host link.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Role {
    /// Stationary: ranges with its neighbours (the self-survey) and serves as a reference for tags.
    #[serde(alias = "Node")]
    Anchor,
    /// Tag on a thing (keys): still most of the time; fixes and publishes its position when it settles.
    Asset,
    /// Tag on something that moves fast (a person with a HUD, a robot): tracked continuously
    /// while it moves, at the fast rate, and a consumer of positions.
    #[serde(alias = "Person")]
    Mobile,
}

/// The hardware an image is built for. An update carries it, so no device installs another
/// board's image; a sink only passes one on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Board {
    /// nRF54L15 DK.
    Dk = 1,
    /// nRF54L15 TAG.
    Tag = 2,
}

impl Board {
    /// As the image names it (`navgrid-board tag`).
    pub const fn name(self) -> &'static str {
        match self {
            Self::Dk => "dk",
            Self::Tag => "tag",
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        [Self::Dk, Self::Tag].into_iter().find(|b| b.name() == name)
    }

    const fn from_u8(b: u8) -> Option<Self> {
        match b {
            1 => Some(Self::Dk),
            2 => Some(Self::Tag),
            _ => None,
        }
    }
}

/// Point in the building frame, millimetres.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointMm {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Per-device settings, written at provisioning and stored in flash (postcard).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NodeConfig {
    pub role: Role,
    /// Also a sink: it has a host link (UART over USB) and serves the host with the mesh's state
    /// and telemetry, whatever its role. A sink is usually an anchor too.
    pub sink: bool,
    /// Use every antenna the board has. `false` holds the radio on antenna 0 and limits every
    /// link with this device to one antenna path: shorter procedures and less energy, no diversity.
    pub diversity: bool,
    /// Surveyed position of a pinned node.
    pub pinned: Option<PointMm>,
    /// Advertising interval. Longer saves energy; others take longer to find the device.
    pub adv_interval_ms: u16,
    /// Environmental sampling period in seconds, 0 = off. Pressure also serves as the
    /// barometric height reference for nearby targets.
    pub env_interval_s: u16,
    /// Also sample the BME688 gas sensor. Its heater makes this by far the costliest sample.
    pub gas: bool,
}

impl Default for NodeConfig {
    fn default() -> Self {
        Self {
            role: Role::Anchor,
            sink: false,
            diversity: true,
            pinned: None,
            adv_interval_ms: 1000,
            env_interval_s: 60,
            gas: false,
        }
    }
}

/// What `navgrid provision` stores on a device, read at boot (FR-O1).
///
/// Not `defmt::Format`: it holds the fleet key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Provisioning {
    pub config: NodeConfig,
    /// The fleet network key. Link keys (D-7) and the controller's key derive from it.
    pub fleet_key: [u8; 16],
    /// Bring-up: the reflector this device ranges with, as CS initiator. Without one it reflects.
    pub peer: Option<NodeId>,
    /// Bring-up: the sink it reports its ranges to over a dedicated link (DEF-1).
    pub sink: Option<NodeId>,
    /// The fleet number on its label (`N042`), assigned at enrolment (D-31).
    pub number: Option<u16>,
    /// The Ed25519 public key that firmware updates must be signed with (D-35). Without one the
    /// device takes no updates.
    pub signing_key: Option<[u8; 32]>,
    /// How much of its log the device sends toward the host.
    pub report_level: ReportLevel,
}

/// How much of its log a device sends toward the host as [`Log`] records (architecture §13): the
/// levels from `Error` down to this one. RTT always gets everything.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ReportLevel {
    Off,
    /// Production: only what went wrong.
    #[default]
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

/// Format 4 of the page, before the sink flag (the sink was a role).
#[derive(Deserialize)]
#[cfg_attr(test, derive(Serialize))]
struct ProvisioningV4 {
    config: NodeConfigV4,
    fleet_key: [u8; 16],
    peer: Option<NodeId>,
    sink: Option<NodeId>,
    number: Option<u16>,
    signing_key: Option<[u8; 32]>,
    report_level: ReportLevel,
}

/// `NodeConfig` as formats 1–4 of the page stored it.
#[derive(Deserialize)]
#[cfg_attr(test, derive(Serialize))]
struct NodeConfigV4 {
    role: RoleV4,
    diversity: bool,
    pinned: Option<PointMm>,
    adv_interval_ms: u16,
    env_interval_s: u16,
    gas: bool,
}

/// `Role` as formats 1–4 of the page stored it: the sink was a role of its own.
#[derive(Clone, Copy, Deserialize)]
#[cfg_attr(test, derive(Serialize))]
enum RoleV4 {
    Node,
    Asset,
    Person,
    Sink,
}

impl From<NodeConfigV4> for NodeConfig {
    fn from(c: NodeConfigV4) -> Self {
        Self {
            role: match c.role {
                RoleV4::Node | RoleV4::Sink => Role::Anchor,
                RoleV4::Asset => Role::Asset,
                RoleV4::Person => Role::Mobile,
            },
            sink: matches!(c.role, RoleV4::Sink),
            diversity: c.diversity,
            pinned: c.pinned,
            adv_interval_ms: c.adv_interval_ms,
            env_interval_s: c.env_interval_s,
            gas: c.gas,
        }
    }
}

/// Format 3 of the page, before the report level.
#[derive(Deserialize)]
struct ProvisioningV3 {
    config: NodeConfigV4,
    fleet_key: [u8; 16],
    peer: Option<NodeId>,
    sink: Option<NodeId>,
    number: Option<u16>,
    signing_key: Option<[u8; 32]>,
}

/// Format 2 of the page, before the signing key.
#[derive(Deserialize)]
struct ProvisioningV2 {
    config: NodeConfigV4,
    fleet_key: [u8; 16],
    peer: Option<NodeId>,
    sink: Option<NodeId>,
    number: Option<u16>,
}

/// Format 1 of the page, before fleet numbers.
#[derive(Deserialize)]
struct ProvisioningV1 {
    config: NodeConfigV4,
    fleet_key: [u8; 16],
    peer: Option<NodeId>,
    sink: Option<NodeId>,
}

/// RRAM layout under the bootloader (architecture §13), as `firmware/node/memory.x` and
/// `firmware/bootloader/memory.x` have it.
pub mod layout {
    /// The bootloader's state: the update and revert markers and swap progress.
    pub const BOOTLOADER_STATE: u32 = 0x0000_8000;
    pub const BOOTLOADER_STATE_LEN: u32 = 12 * 1024;
    /// The running image.
    pub const ACTIVE: u32 = 0x0000_B000;
    pub const ACTIVE_LEN: u32 = 720 * 1024;
    /// Where an update waits for the bootloader to swap it in.
    pub const DFU: u32 = 0x000B_F000;
    pub const DFU_LEN: u32 = 724 * 1024;
    /// The [`UpdateHeader`](crate::UpdateHeader) of an update written to [`DFU`] but not yet
    /// checked.
    pub const STAGED_UPDATE: u32 = 0x0017_A000;
    pub const STAGED_UPDATE_LEN: u32 = 4096;
    /// The header of the update this image was installed from, kept so the node can pass the
    /// image on to its neighbours.
    pub const ACTIVE_HEADER: u32 = 0x0017_9000;
    pub const ACTIVE_HEADER_LEN: u32 = 4096;
}

/// L2CAP channel for firmware updates between neighbours (architecture §13).
pub const UPDATE_SPSM: u16 = 0x0083;
/// Image bytes per [`UpdateMsg::Data`]: one SDU (245 bytes) with its framing, and whole 16-byte
/// RRAM lines.
pub const UPDATE_CHUNK: usize = 224;

/// One SDU of an update transfer. A node that runs an installed update offers it; a neighbour
/// that wants it says where to start, and the sender streams the image to the end. The receiver
/// answers `Done` once it has the image stored, and the sender keeps the channel open until then:
/// closing it drops whatever the receiver hasn't read yet.
///
/// The host sends a sink the same messages over its serial port ([`encode_downlink`]), with
/// [`DOWNLINK_CHUNK`]-byte `Data` and a CRC-32 each. The sink answers in [`Message::Update`], a `Want` after each
/// `Data` as well: the serial port has no flow control of its own, so the host sends the next
/// chunk only then, and resends one that got no answer.
///
/// Fixed layout, little-endian: a tag byte, then `Offer` the encoded [`UpdateHeader`], `Want`
/// the start offset, `Data` the offset and the bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateMsg<'a> {
    Offer(UpdateHeader),
    Want {
        from: u32,
    },
    Decline,
    Data {
        offset: u32,
        bytes: &'a [u8],
    },
    Done,
    /// A mesh record from the controller for the sink to inject (a desired-state record,
    /// `navgrid control`), postcard-encoded; the sink mirrors it back as [`Message::Record`].
    Record(&'a [u8]),
}

impl<'a> UpdateMsg<'a> {
    /// The encoded message; `buf` needs [`UPDATE_HEADER_LEN`] + 1 bytes, or 5 more than the data.
    pub fn encode<'b>(&self, buf: &'b mut [u8]) -> &'b [u8] {
        let len = match *self {
            Self::Offer(h) => {
                buf[0] = 1;
                buf[1..1 + UPDATE_HEADER_LEN].copy_from_slice(&h.encode());
                1 + UPDATE_HEADER_LEN
            }
            Self::Want { from } => {
                buf[0] = 2;
                buf[1..5].copy_from_slice(&from.to_le_bytes());
                5
            }
            Self::Decline => {
                buf[0] = 3;
                1
            }
            Self::Data { offset, bytes } => {
                buf[0] = 4;
                buf[1..5].copy_from_slice(&offset.to_le_bytes());
                buf[5..5 + bytes.len()].copy_from_slice(bytes);
                5 + bytes.len()
            }
            Self::Done => {
                buf[0] = 5;
                1
            }
            Self::Record(bytes) => {
                buf[0] = 6;
                buf[1..1 + bytes.len()].copy_from_slice(bytes);
                1 + bytes.len()
            }
        };
        &buf[..len]
    }

    pub fn decode(b: &'a [u8]) -> Option<Self> {
        let word = |i: usize| Some(u32::from_le_bytes(b.get(i..i + 4)?.try_into().ok()?));
        Some(match *b.first()? {
            1 => Self::Offer(UpdateHeader::decode(&b[1..]).ok()?),
            2 => Self::Want { from: word(1)? },
            3 => Self::Decline,
            4 => Self::Data {
                offset: word(1)?,
                bytes: &b[5..],
            },
            5 => Self::Done,
            6 => Self::Record(&b[1..]),
            _ => return None,
        })
    }
}

/// What a signed update says about its image (D-35). `navgrid sign` writes it in front of the
/// image; a device keeps it in [`layout::STAGED_UPDATE`] until it has checked the image in the
/// update slot.
///
/// Fixed layout, little-endian: magic `ngup`, format (2), flags (bit 0 dirty), the [`Board`], a
/// reserved byte, version, commit, image length, the 64-byte Ed25519 signature over the image's SHA-512 digest
/// (as embassy-boot verifies it), and a CRC-32 of everything before it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UpdateHeader {
    pub firmware: Firmware,
    pub board: Board,
    pub len: u32,
    pub signature: [u8; 64],
}

pub const UPDATE_HEADER_LEN: usize = 88;
const UPDATE_MAGIC: [u8; 4] = *b"ngup";
const UPDATE_FORMAT: u8 = 2;

impl UpdateHeader {
    pub fn encode(&self) -> [u8; UPDATE_HEADER_LEN] {
        let mut b = [0u8; UPDATE_HEADER_LEN];
        b[..4].copy_from_slice(&UPDATE_MAGIC);
        b[4] = UPDATE_FORMAT;
        b[5] = u8::from(self.firmware.dirty);
        b[6] = self.board as u8;
        b[8..12].copy_from_slice(&self.firmware.version.to_le_bytes());
        b[12..16].copy_from_slice(&self.firmware.commit.to_le_bytes());
        b[16..20].copy_from_slice(&self.len.to_le_bytes());
        b[20..84].copy_from_slice(&self.signature);
        let crc = crc32(&b[..84]);
        b[84..88].copy_from_slice(&crc.to_le_bytes());
        b
    }

    /// Decodes a header, as `navgrid sign` writes it or as read from the staging page.
    pub fn decode(b: &[u8]) -> Result<Self, ProvisioningError> {
        let b = b.get(..UPDATE_HEADER_LEN).ok_or(ProvisioningError::Corrupt)?;
        if b.iter().all(|&x| x == 0xff) {
            return Err(ProvisioningError::Blank);
        }
        if b[..4] != UPDATE_MAGIC {
            return Err(ProvisioningError::Corrupt);
        }
        if b[4] != UPDATE_FORMAT {
            return Err(ProvisioningError::Format(b[4]));
        }
        let word = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        if crc32(&b[..84]) != word(84) {
            return Err(ProvisioningError::Corrupt);
        }
        let board = Board::from_u8(b[6]).ok_or(ProvisioningError::Corrupt)?;
        let mut signature = [0u8; 64];
        signature.copy_from_slice(&b[20..84]);
        Ok(Self {
            firmware: Firmware {
                version: word(8),
                commit: word(12),
                dirty: b[5] & 1 != 0,
            },
            board,
            len: word(16),
            signature,
        })
    }
}

/// Where the [`Provisioning`] page lives in RRAM: the second-to-last 4 KiB, which the firmware's
/// `memory.x` keeps out of the image. Flashing firmware leaves it alone.
pub const PROVISIONING_ADDRESS: u32 = 0x0017_B000;
/// The provisioning page's header: magic, format, payload length, CRC-32 of the payload.
const PROVISIONING_MAGIC: [u8; 4] = *b"ngpv";
const PROVISIONING_FORMAT: u8 = 5;
const PROVISIONING_HEADER: usize = 12;
/// Upper bound for an encoded provisioning page.
pub const MAX_PROVISIONING_LEN: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProvisioningError {
    /// Never provisioned: the page is erased.
    Blank,
    /// Something else is there, or it is cut short.
    Corrupt,
    /// Written by a newer `navgrid provision`, or not at all.
    Format(u8),
}

/// Encodes `p` as a provisioning page; returns the used part of `buf`.
pub fn encode_provisioning<'a>(p: &Provisioning, buf: &'a mut [u8]) -> Result<&'a [u8], Error> {
    if buf.len() < PROVISIONING_HEADER {
        return Err(Error::SerializeBufferFull);
    }
    let (header, rest) = buf.split_at_mut(PROVISIONING_HEADER);
    let len = postcard::to_slice(p, rest)?.len();
    header[..4].copy_from_slice(&PROVISIONING_MAGIC);
    header[4] = PROVISIONING_FORMAT;
    header[5] = 0;
    header[6..8].copy_from_slice(&(len as u16).to_le_bytes());
    header[8..12].copy_from_slice(&crc32(&rest[..len]).to_le_bytes());
    Ok(&buf[..PROVISIONING_HEADER + len])
}

/// Decodes a provisioning page, as read from [`PROVISIONING_ADDRESS`].
pub fn decode_provisioning(page: &[u8]) -> Result<Provisioning, ProvisioningError> {
    let header = page.get(..PROVISIONING_HEADER).ok_or(ProvisioningError::Corrupt)?;
    if header.iter().all(|&b| b == 0xff) {
        return Err(ProvisioningError::Blank);
    }
    if header[..4] != PROVISIONING_MAGIC {
        return Err(ProvisioningError::Corrupt);
    }
    let format = header[4];
    if format == 0 || format > PROVISIONING_FORMAT {
        return Err(ProvisioningError::Format(format));
    }
    let len = usize::from(u16::from_le_bytes([header[6], header[7]]));
    let crc = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
    let payload = page
        .get(PROVISIONING_HEADER..PROVISIONING_HEADER + len)
        .ok_or(ProvisioningError::Corrupt)?;
    if crc32(payload) != crc {
        return Err(ProvisioningError::Corrupt);
    }
    if format == 1 {
        let v1: ProvisioningV1 = postcard::from_bytes(payload).map_err(|_| ProvisioningError::Corrupt)?;
        return Ok(Provisioning {
            config: v1.config.into(),
            fleet_key: v1.fleet_key,
            peer: v1.peer,
            sink: v1.sink,
            number: None,
            signing_key: None,
            report_level: ReportLevel::default(),
        });
    }
    if format == 2 {
        let v2: ProvisioningV2 = postcard::from_bytes(payload).map_err(|_| ProvisioningError::Corrupt)?;
        return Ok(Provisioning {
            config: v2.config.into(),
            fleet_key: v2.fleet_key,
            peer: v2.peer,
            sink: v2.sink,
            number: v2.number,
            signing_key: None,
            report_level: ReportLevel::default(),
        });
    }
    if format == 3 {
        let v3: ProvisioningV3 = postcard::from_bytes(payload).map_err(|_| ProvisioningError::Corrupt)?;
        return Ok(Provisioning {
            config: v3.config.into(),
            fleet_key: v3.fleet_key,
            peer: v3.peer,
            sink: v3.sink,
            number: v3.number,
            signing_key: v3.signing_key,
            report_level: ReportLevel::default(),
        });
    }
    if format == 4 {
        let v4: ProvisioningV4 = postcard::from_bytes(payload).map_err(|_| ProvisioningError::Corrupt)?;
        return Ok(Provisioning {
            config: v4.config.into(),
            fleet_key: v4.fleet_key,
            peer: v4.peer,
            sink: v4.sink,
            number: v4.number,
            signing_key: v4.signing_key,
            report_level: v4.report_level,
        });
    }
    postcard::from_bytes(payload).map_err(|_| ProvisioningError::Corrupt)
}

/// CRC-32 (IEEE 802.3, as zlib).
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Company identifier in a [`Beacon`]'s manufacturer-specific data: 0xFFFF, which the Bluetooth
/// SIG reserves for tests, until we have our own.
pub const BEACON_COMPANY: u16 = 0xffff;
/// A beacon's length after the company identifier.
pub const BEACON_LEN: usize = 20;
const BEACON_FORMAT: u8 = 2;

/// Battery thresholds under radio load, for the LED status display and the host's low-battery
/// flag. Provisional until the CR2032 sag measurements (P0): at rest a CR2032 holds about 3.0 V
/// almost to the end.
pub const BATTERY_GOOD_MV: u16 = 2800;
pub const BATTERY_LOW_MV: u16 = 2600;
pub const BATTERY_CRITICAL_MV: u16 = 2400;

/// What a node advertises (FR-M4), so that a tag can choose nodes and get a height reference
/// without a directory, and the field tool can list it (FR-O6).
///
/// Layout after the company identifier, little-endian: format (2), role, flags (bit 0 attention,
/// bit 1 install mode, bit 2 sink, bit 3 routes only: no ranging with it), hops, free links, battery (20 mV steps above 2.0 V), pressure (0.1 Pa,
/// 3 bytes), fleet number (2 bytes), position x, y, z (decimetres, 2 bytes each), position σ
/// (decimetres), frame version (2 bytes). All-ones fields are unknown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Beacon {
    pub role: Role,
    /// It has a host link (it is a sink); its hop count is then 0.
    pub sink: bool,
    /// Its button was pressed in the last 30 s (FR-O5).
    pub attention: bool,
    /// It is in install mode (architecture §13).
    pub install: bool,
    /// It ranges: its links survey and fix positions. `false` for a node that only routes;
    /// decoders before the flag take every node as ranging.
    pub ranging: bool,
    /// Hops to the nearest sink; 255 with no path.
    pub hops: u8,
    /// Links it can still take (admission control, NFR-4).
    pub free_links: u8,
    pub battery_mv: Option<u16>,
    pub pressure_dpa: Option<u32>,
    pub number: Option<u16>,
    pub position: Option<BeaconPosition>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BeaconPosition {
    /// Decimetres, in the frame `frame` names.
    pub point_dm: [i16; 3],
    /// 1-sigma, decimetres, at most 254.
    pub sigma_dm: u8,
    /// Frame version (FR-A5).
    pub frame: u16,
}

impl Beacon {
    pub fn encode(&self) -> [u8; BEACON_LEN] {
        let mut b = [0xffu8; BEACON_LEN];
        b[0] = BEACON_FORMAT;
        b[1] = self.role as u8;
        b[2] = u8::from(self.attention)
            | (u8::from(self.install) << 1)
            | (u8::from(self.sink) << 2)
            | (u8::from(!self.ranging) << 3);
        b[3] = self.hops;
        b[4] = self.free_links;
        if let Some(mv) = self.battery_mv {
            b[5] = (mv.saturating_sub(2000) / 20).min(254) as u8;
        }
        if let Some(p) = self.pressure_dpa {
            b[6..9].copy_from_slice(&p.min(0xff_fffe).to_le_bytes()[..3]);
        }
        if let Some(n) = self.number {
            b[9..11].copy_from_slice(&n.min(0xfffe).to_le_bytes());
        }
        if let Some(pos) = self.position {
            for (i, v) in pos.point_dm.iter().enumerate() {
                b[11 + 2 * i..13 + 2 * i].copy_from_slice(&v.to_le_bytes());
            }
            b[17] = pos.sigma_dm.min(254);
            b[18..20].copy_from_slice(&pos.frame.to_le_bytes());
        }
        b
    }

    pub fn decode(b: &[u8]) -> Option<Self> {
        let b: &[u8; BEACON_LEN] = b.get(..BEACON_LEN)?.try_into().ok()?;
        if b[0] != BEACON_FORMAT {
            return None;
        }
        let (role, legacy_sink) = match b[1] {
            0 => (Role::Anchor, false),
            1 => (Role::Asset, false),
            2 => (Role::Mobile, false),
            // Firmware before the sink flag advertised the sink as a role.
            3 => (Role::Anchor, true),
            _ => return None,
        };
        let i16_at = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        let pressure = u32::from_le_bytes([b[6], b[7], b[8], 0]);
        let number = u16::from_le_bytes([b[9], b[10]]);
        Some(Self {
            role,
            sink: legacy_sink || b[2] & 4 != 0,
            attention: b[2] & 1 != 0,
            install: b[2] & 2 != 0,
            ranging: b[2] & 8 == 0,
            hops: b[3],
            free_links: b[4],
            battery_mv: (b[5] != 0xff).then(|| 2000 + 20 * u16::from(b[5])),
            pressure_dpa: (pressure != 0xff_ffff).then_some(pressure),
            number: (number != 0xffff).then_some(number),
            position: (b[17] != 0xff).then(|| BeaconPosition {
                point_dm: [i16_at(11), i16_at(13), i16_at(15)],
                sigma_dm: b[17],
                frame: u16::from_le_bytes([b[18], b[19]]),
            }),
        })
    }
}

/// Estimator that produced a [`Range`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangeMethod {
    PhaseSlope,
    Rtt,
    Fused,
}

/// Result of one CS procedure on a link.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Range {
    pub initiator: NodeId,
    pub reflector: NodeId,
    /// Initiator uptime when the procedure completed.
    pub uptime_ms: u32,
    pub distance_mm: u32,
    /// 1-sigma uncertainty.
    pub sigma_mm: u32,
    /// Estimator quality: 0 = unusable, 255 = best. Mapping is defined in `navgrid-ranging`.
    pub quality: u8,
    pub method: RangeMethod,
    /// How much the estimate rests on: for phase slope, the adjacent channel pairs in the fit.
    pub steps: u8,
    /// Antenna paths sounded: initiator antennas × reflector antennas, at most 4.
    pub antenna_paths: u8,
}

/// A device's estimate of its own position.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Position {
    pub node: NodeId,
    pub uptime_ms: u32,
    pub point: PointMm,
    /// 1-sigma per axis.
    pub sigma_mm: [u32; 3],
    pub pinned: bool,
}

/// Periodic device health for the monitoring system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Health {
    pub node: NodeId,
    pub uptime_s: u32,
    pub battery_mv: u16,
    /// Die temperature, 0.01 °C.
    pub temperature_cdeg: i16,
    pub firmware: Firmware,
    /// Telemetry records waiting for a contact toward a sink.
    pub queued: u16,
}

/// Environmental sample (TAG: BME688).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Environment {
    pub node: NodeId,
    pub uptime_s: u32,
    /// 0.01 °C.
    pub temperature_cdeg: i16,
    /// 0.01 %RH.
    pub humidity_cpct: u16,
    /// 0.01 Pa. Near sea level 1 Pa ≈ 8.5 cm of height, so this resolution is ample for
    /// differential barometry.
    pub pressure_cpa: u32,
    /// Gas-sensor resistance in ohms, when gas sampling is on.
    pub gas_ohm: Option<u32>,
}

/// Counters for one link, as seen from `node`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkStats {
    pub node: NodeId,
    pub peer: NodeId,
    pub procedures: u16,
    pub failures: u16,
    pub distance_mm: u32,
    pub quality: u8,
    pub rssi_dbm: i8,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Message {
    Hello {
        node: NodeId,
        role: Role,
        protocol: u16,
        firmware: Firmware,
        /// Antennas in use for Channel Sounding (1 when diversity is off).
        antennas: u8,
    },
    Range(Range),
    Position(Position),
    Health(Health),
    Environment(Environment),
    Link(LinkStats),
    /// A sink's answer to an update the host sends it (see [`UpdateMsg`]).
    Update(UpdateAnswer),
    Boot(Boot),
    Log(Log),
    /// One antenna path's raw tones from one procedure, a chunk of consecutive channels
    /// (diagnostics; `navgrid tones` runs the host estimators on them).
    Tones(Tones),
    /// A mesh-state record a sink holds (`navgrid-mesh`, architecture §12), as the sink mirrors
    /// its replica to the host: the record encoded with postcard, opaque here.
    Record(MeshRecord),
    /// A beacon a sink heard while scanning (the field tool's view, FR-O6): who, how loud, and
    /// what the beacon said.
    Heard(Heard),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Heard {
    /// The node that heard it.
    pub node: NodeId,
    pub of: NodeId,
    pub rssi_dbm: i8,
    pub role: Role,
    pub sink: bool,
    pub attention: bool,
    pub install: bool,
    pub hops: u8,
    pub free_links: u8,
    pub battery_mv: Option<u16>,
    pub number: Option<u16>,
}

/// Room for a mesh record on the wire: a node record with its eight neighbours is the largest.
pub const MESH_RECORD_LEN: usize = 144;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MeshRecord {
    pub bytes: heapless::Vec<u8, MESH_RECORD_LEN>,
}

/// One `defmt` frame from a device's log, as its logger wrote it: the string index, the timestamp
/// and the arguments. The host decodes it with the ELF of `firmware` for `board`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Log {
    pub node: NodeId,
    pub firmware: Firmware,
    pub board: Board,
    pub frame: heapless::Vec<u8, LOG_FRAME_LEN>,
}

/// The longest frame a device sends; longer ones stay in its RTT log only.
pub const LOG_FRAME_LEN: usize = 64;

/// Raw tones of one antenna path of one procedure, for the host's estimators: `pct` holds the
/// channels from `first` up, 3 bytes each in the controller's PCT layout (12-bit I and Q,
/// `navgrid_ranging::steps::pct`), all zero where the channel had no usable tone. A procedure
/// takes three of these ([`TONES_CHANNELS`] channels each). An initiator sends one path a second,
/// its paths in turn.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tones {
    pub node: NodeId,
    pub procedure: u16,
    pub path: u8,
    pub paths: u8,
    pub first: u8,
    pub pct: heapless::Vec<u8, TONES_PCT_LEN>,
}

/// Channels per [`Tones`] message, and the bytes they take.
pub const TONES_CHANNELS: usize = 33;
pub const TONES_PCT_LEN: usize = 3 * TONES_CHANNELS;

/// What a device reports once per boot: why it started, the state of its image, and how its
/// last run ended if that was a crash (architecture §13).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Boot {
    pub node: NodeId,
    pub firmware: Firmware,
    pub board: Board,
    /// Counts the device's boots.
    pub epoch: u32,
    /// The chip's `RESETREAS` bits; see [`reset_reasons`].
    pub reset_reason: u32,
    pub image: ImageState,
    pub crash: Option<Crash>,
}

/// The image a device booted, as the bootloader left it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImageState {
    Confirmed,
    /// Just swapped in; it confirms itself once the device is back in the mesh.
    Unconfirmed,
    /// The bootloader put this image back after one that didn't confirm, named if known.
    Reverted(Option<Firmware>),
}

/// How the device's last run ended, when it crashed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Crash {
    pub kind: CrashKind,
    /// A panic's source line.
    pub line: u32,
    /// A hard fault's faulting instruction.
    pub pc: u32,
    /// A panic's file name and the start of its message, as `update.rs: message`.
    pub text: heapless::String<CRASH_TEXT_LEN>,
}

pub const CRASH_TEXT_LEN: usize = 56;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrashKind {
    Panic,
    HardFault,
}

/// The nRF54L's reset reasons in `RESETREAS`, by bit. None set means power-on.
const RESET_REASONS: [&str; 14] = [
    "reset pin",
    "watchdog 0",
    "watchdog 1",
    "debugger soft reset",
    "debugger hard reset",
    "debugger pin reset",
    "soft reset",
    "CPU lockup",
    "wake from System OFF (GPIO)",
    "wake from System OFF (LPCOMP)",
    "debug interface",
    "wake from System OFF (GRTC)",
    "wake from System OFF (NFC)",
    "tamper",
];

/// The reset reasons set in `bits`, or `power-on`.
pub fn reset_reasons(bits: u32) -> impl Iterator<Item = &'static str> {
    let power_on = (bits == 0).then_some("power-on");
    RESET_REASONS
        .iter()
        .enumerate()
        .filter(move |(i, _)| bits & (1 << i) != 0)
        .map(|(_, &name)| name)
        .chain(power_on)
}

/// What a sink answers the host's [`UpdateMsg`]s with.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UpdateAnswer {
    /// Send on from this byte: after the offer, and after each chunk stored.
    Want {
        from: u32,
    },
    Decline,
    /// Stored: the sink installs it, or passes it to the nodes on its links.
    Done,
}

/// Image bytes per [`UpdateMsg::Data`] from the host to a sink: an RRAM page.
pub const DOWNLINK_CHUNK: usize = 4096;
/// The longest message from the host to a sink, with its CRC.
const MAX_DOWNLINK_MSG: usize = DOWNLINK_CHUNK + 5 + 4;
/// Upper bound for one frame from the host to a sink, COBS overhead and terminator included.
pub const MAX_DOWNLINK_FRAME: usize = cobs::max_encoding_length(MAX_DOWNLINK_MSG) + 1;

/// `msg` as one frame for a sink: the message and its CRC-32, COBS-framed, `0x00` terminator
/// included. `out` needs [`MAX_DOWNLINK_FRAME`] bytes.
pub fn encode_downlink<'b>(msg: &UpdateMsg, out: &'b mut [u8]) -> &'b [u8] {
    let mut raw = [0u8; MAX_DOWNLINK_MSG];
    let len = msg.encode(&mut raw).len();
    let crc = crc32(&raw[..len]);
    raw[len..len + 4].copy_from_slice(&crc.to_le_bytes());
    let n = cobs::encode(&raw[..len + 4], out);
    out[n] = 0;
    &out[..=n]
}

/// A frame from the host, without its terminator, decoded in place; `None` if it was damaged on
/// the way.
pub fn decode_downlink(frame: &mut [u8]) -> Option<UpdateMsg<'_>> {
    let n = cobs::decode_in_place(frame).ok()?;
    let (msg, crc) = frame[..n].split_at(n.checked_sub(4)?);
    if crc32(msg).to_le_bytes() != crc {
        return None;
    }
    UpdateMsg::decode(msg)
}

pub type Error = postcard::Error;

/// Encodes `msg` as one COBS frame including the `0x00` terminator; returns the used part of `buf`.
pub fn encode<'a>(msg: &Message, buf: &'a mut [u8]) -> Result<&'a mut [u8], Error> {
    postcard::to_slice_cobs(msg, buf)
}

/// Decodes one COBS frame in place. `frame` excludes the `0x00` terminator.
pub fn decode(frame: &mut [u8]) -> Result<Message, Error> {
    postcard::from_bytes_cobs(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn worst_case() -> [Message; 9] {
        let id = NodeId(u64::MAX);
        [
            Message::Hello {
                node: id,
                role: Role::Mobile,
                protocol: u16::MAX,
                firmware: Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true),
                antennas: u8::MAX,
            },
            Message::Range(Range {
                initiator: id,
                reflector: id,
                uptime_ms: u32::MAX,
                distance_mm: u32::MAX,
                sigma_mm: u32::MAX,
                quality: u8::MAX,
                method: RangeMethod::Fused,
                steps: u8::MAX,
                antenna_paths: u8::MAX,
            }),
            Message::Position(Position {
                node: id,
                uptime_ms: u32::MAX,
                point: PointMm {
                    x: i32::MIN,
                    y: i32::MIN,
                    z: i32::MIN,
                },
                sigma_mm: [u32::MAX; 3],
                pinned: true,
            }),
            Message::Health(Health {
                node: id,
                uptime_s: u32::MAX,
                battery_mv: u16::MAX,
                temperature_cdeg: i16::MIN,
                firmware: Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true),
                queued: u16::MAX,
            }),
            Message::Environment(Environment {
                node: id,
                uptime_s: u32::MAX,
                temperature_cdeg: i16::MIN,
                humidity_cpct: u16::MAX,
                pressure_cpa: u32::MAX,
                gas_ohm: Some(u32::MAX),
            }),
            Message::Link(LinkStats {
                node: id,
                peer: id,
                procedures: u16::MAX,
                failures: u16::MAX,
                distance_mm: u32::MAX,
                quality: u8::MAX,
                rssi_dbm: i8::MIN,
            }),
            Message::Update(UpdateAnswer::Want { from: u32::MAX }),
            Message::Boot(Boot {
                node: id,
                firmware: Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true),
                board: Board::Tag,
                epoch: u32::MAX,
                reset_reason: u32::MAX,
                image: ImageState::Reverted(Some(Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true))),
                crash: Some(Crash {
                    kind: CrashKind::HardFault,
                    line: u32::MAX,
                    pc: u32::MAX,
                    text: heapless::String::try_from("x".repeat(CRASH_TEXT_LEN).as_str()).unwrap(),
                }),
            }),
            Message::Log(Log {
                node: id,
                firmware: Firmware::new(u8::MAX, u8::MAX, u8::MAX, u32::MAX, true),
                board: Board::Tag,
                frame: heapless::Vec::from_slice(&[0xff; LOG_FRAME_LEN]).unwrap(),
            }),
        ]
    }

    #[test]
    fn reset_reasons_by_bit() {
        assert_eq!(reset_reasons(0).collect::<std::vec::Vec<_>>(), ["power-on"]);
        assert_eq!(
            reset_reasons(1 << 1 | 1 << 6).collect::<std::vec::Vec<_>>(),
            ["watchdog 0", "soft reset"]
        );
    }

    #[test]
    fn worst_case_roundtrips_within_max_frame() {
        for msg in worst_case() {
            let mut buf = [0u8; MAX_FRAME_LEN];
            let frame = encode(&msg, &mut buf).unwrap();
            let (terminator, body) = frame.split_last_mut().unwrap();
            assert_eq!(*terminator, 0);
            assert!(!body.contains(&0), "COBS body must not contain the delimiter");
            assert_eq!(decode(body).unwrap(), msg);
        }
    }

    #[test]
    fn provisioning_roundtrips_and_rejects_damage() {
        let p = Provisioning {
            config: NodeConfig {
                role: Role::Anchor,
                sink: true,
                pinned: Some(PointMm { x: 1, y: 2, z: 3 }),
                ..NodeConfig::default()
            },
            fleet_key: [0xa5; 16],
            peer: Some(NodeId(0xc0ff_ee12_3456)),
            sink: None,
            number: Some(42),
            signing_key: Some([0x5a; 32]),
            report_level: ReportLevel::Warn,
        };
        let mut buf = [0u8; MAX_PROVISIONING_LEN];
        let page = encode_provisioning(&p, &mut buf).unwrap().to_vec();
        assert_eq!(decode_provisioning(&page), Ok(p));
        // What follows the page on the device doesn't matter.
        let mut longer = page.clone();
        longer.extend_from_slice(&[0xff; 64]);
        assert_eq!(decode_provisioning(&longer), Ok(p));
        let mut flipped = page.clone();
        *flipped.last_mut().unwrap() ^= 1;
        assert_eq!(decode_provisioning(&flipped), Err(ProvisioningError::Corrupt));
        assert_eq!(decode_provisioning(&[0xff; 64]), Err(ProvisioningError::Blank));
        let mut newer = page.clone();
        newer[4] = 6;
        assert_eq!(decode_provisioning(&newer), Err(ProvisioningError::Format(6)));
        assert_eq!(
            decode_provisioning(&page[..page.len() - 1]),
            Err(ProvisioningError::Corrupt)
        );
    }

    /// A default `NodeConfig` as formats 1–4 stored it.
    fn legacy_config() -> NodeConfigV4 {
        NodeConfigV4 {
            role: RoleV4::Node,
            diversity: true,
            pinned: None,
            adv_interval_ms: 1000,
            env_interval_s: 60,
            gas: false,
        }
    }

    #[test]
    fn reads_format_1_pages() {
        #[derive(Serialize)]
        struct V1 {
            config: NodeConfigV4,
            fleet_key: [u8; 16],
            peer: Option<NodeId>,
            sink: Option<NodeId>,
        }
        let v1 = V1 {
            config: legacy_config(),
            fleet_key: [7; 16],
            peer: None,
            sink: Some(NodeId(0xc000_0000_0001)),
        };
        let mut page = [0u8; MAX_PROVISIONING_LEN];
        let len = postcard::to_slice(&v1, &mut page[PROVISIONING_HEADER..]).unwrap().len();
        let crc = crc32(&page[PROVISIONING_HEADER..PROVISIONING_HEADER + len]);
        page[..4].copy_from_slice(&PROVISIONING_MAGIC);
        page[4] = 1;
        page[6..8].copy_from_slice(&(len as u16).to_le_bytes());
        page[8..12].copy_from_slice(&crc.to_le_bytes());
        let p = decode_provisioning(&page).unwrap();
        assert_eq!((p.sink, p.number), (v1.sink, None));
    }

    #[test]
    fn reads_format_3_pages_at_error_level() {
        #[derive(Serialize)]
        struct V3 {
            config: NodeConfigV4,
            fleet_key: [u8; 16],
            peer: Option<NodeId>,
            sink: Option<NodeId>,
            number: Option<u16>,
            signing_key: Option<[u8; 32]>,
        }
        let v3 = V3 {
            config: legacy_config(),
            fleet_key: [7; 16],
            peer: None,
            sink: None,
            number: Some(3),
            signing_key: Some([9; 32]),
        };
        let mut page = [0u8; MAX_PROVISIONING_LEN];
        let len = postcard::to_slice(&v3, &mut page[PROVISIONING_HEADER..]).unwrap().len();
        let crc = crc32(&page[PROVISIONING_HEADER..PROVISIONING_HEADER + len]);
        page[..4].copy_from_slice(&PROVISIONING_MAGIC);
        page[4] = 3;
        page[6..8].copy_from_slice(&(len as u16).to_le_bytes());
        page[8..12].copy_from_slice(&crc.to_le_bytes());
        let p = decode_provisioning(&page).unwrap();
        assert_eq!(
            (p.number, p.signing_key, p.report_level),
            (Some(3), Some([9; 32]), ReportLevel::Error)
        );
    }

    #[test]
    fn node_id_from_ficr_matches_the_logged_id() {
        // The bench initiator TAG: FICR reads dfcc4c2d d1aece36; it logs ce36dfcc4c2d.
        let id = NodeId::from_ficr([0xdfcc_4c2d, 0xd1ae_ce36]);
        assert_eq!(id, NodeId(0xce36_dfcc_4c2d));
        assert_eq!(id.address(), [0x2d, 0x4c, 0xcc, 0xdf, 0x36, 0xce]);
    }

    #[test]
    fn firmware_displays_as_semver_plus_commit() {
        let fw = Firmware::new(0, 1, 2, 0x1950_d45d, true);
        assert_eq!(std::format!("{fw}"), "0.1.2+1950d45d-dirty");
        assert_eq!(fw.semver(), (0, 1, 2));
    }

    #[test]
    fn update_header_roundtrips_and_rejects_damage() {
        let h = UpdateHeader {
            firmware: Firmware::new(0, 1, 1, 0x0cf9_80a7, false),
            board: Board::Tag,
            len: 415_144,
            signature: [0x3c; 64],
        };
        let b = h.encode();
        assert_eq!(UpdateHeader::decode(&b), Ok(h));
        let mut bad = b;
        bad[30] ^= 1;
        assert_eq!(UpdateHeader::decode(&bad), Err(ProvisioningError::Corrupt));
        assert_eq!(UpdateHeader::decode(&[0xff; 4096]), Err(ProvisioningError::Blank));
        let mut format1 = b;
        format1[4] = 1;
        assert_eq!(UpdateHeader::decode(&format1), Err(ProvisioningError::Format(1)));
    }

    #[test]
    fn update_messages_roundtrip() {
        let header = UpdateHeader {
            firmware: Firmware::new(0, 1, 1, 7, false),
            board: Board::Dk,
            len: 1000,
            signature: [9; 64],
        };
        let data = [0x42u8; UPDATE_CHUNK];
        let mut buf = [0u8; UPDATE_CHUNK + 5];
        for msg in [
            UpdateMsg::Offer(header),
            UpdateMsg::Want { from: 4096 },
            UpdateMsg::Decline,
            UpdateMsg::Data {
                offset: 224,
                bytes: &data,
            },
            UpdateMsg::Done,
        ] {
            let encoded = msg.encode(&mut buf).to_vec();
            assert!(encoded.len() <= 245);
            assert_eq!(UpdateMsg::decode(&encoded), Some(msg));
        }
        assert_eq!(UpdateMsg::decode(&[9]), None);
    }

    #[test]
    fn downlink_frames_decode_and_fit() {
        let page = [0u8; DOWNLINK_CHUNK];
        let msg = UpdateMsg::Data {
            offset: 8192,
            bytes: &page,
        };
        let mut out = [0u8; MAX_DOWNLINK_FRAME];
        let frame = encode_downlink(&msg, &mut out).to_vec();
        assert_eq!(frame.iter().position(|&b| b == 0), Some(frame.len() - 1));
        let mut body = frame[..frame.len() - 1].to_vec();
        assert_eq!(decode_downlink(&mut body.clone()), Some(msg));
        // A flipped bit, or a byte lost on the way.
        body[1000] ^= 0x10;
        assert_eq!(decode_downlink(&mut body.clone()), None);
        body.remove(2000);
        assert_eq!(decode_downlink(&mut body), None);
    }

    #[test]
    fn boards_by_name() {
        for b in [Board::Dk, Board::Tag] {
            assert_eq!(Board::from_name(b.name()), Some(b));
            assert_eq!(Board::from_u8(b as u8), Some(b));
        }
        assert_eq!(Board::from_name("nrf52"), None);
    }

    #[test]
    fn crc32_matches_zlib() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn beacon_roundtrips_with_unknowns() {
        let full = Beacon {
            role: Role::Anchor,
            sink: false,
            attention: true,
            install: false,
            number: Some(42),
            hops: 3,
            free_links: 9,
            battery_mv: Some(2980),
            pressure_dpa: Some(1_013_250),
            ranging: false,
            position: Some(BeaconPosition {
                point_dm: [-120, 345, 27],
                sigma_dm: 3,
                frame: 7,
            }),
        };
        assert_eq!(Beacon::decode(&full.encode()), Some(full));
        let bare = Beacon {
            role: Role::Mobile,
            sink: true,
            attention: false,
            install: true,
            number: None,
            hops: 0,
            free_links: 14,
            battery_mv: None,
            pressure_dpa: None,
            ranging: true,
            position: None,
        };
        assert_eq!(Beacon::decode(&bare.encode()), Some(bare));
        assert_eq!(Beacon::decode(&[9; BEACON_LEN]), None);
        // A beacon from firmware that still advertised the sink as role 3.
        let mut legacy = bare.encode();
        legacy[1] = 3;
        legacy[2] = 0;
        let decoded = Beacon::decode(&legacy).unwrap();
        assert_eq!((decoded.role, decoded.sink), (Role::Anchor, true));
    }

    #[test]
    fn format_4_pages_decode_with_the_sink_as_a_flag() {
        let v4 = ProvisioningV4 {
            config: NodeConfigV4 {
                role: RoleV4::Sink,
                diversity: false,
                pinned: Some(PointMm { x: 1, y: 2, z: 3 }),
                adv_interval_ms: 500,
                env_interval_s: 0,
                gas: true,
            },
            fleet_key: [0xa5; 16],
            peer: None,
            sink: Some(NodeId(7)),
            number: Some(3),
            signing_key: None,
            report_level: ReportLevel::Info,
        };
        let mut page = [0xffu8; MAX_PROVISIONING_LEN];
        let mut payload = [0u8; MAX_PROVISIONING_LEN];
        let len = postcard::to_slice(&v4, &mut payload).unwrap().len();
        page[..4].copy_from_slice(&PROVISIONING_MAGIC);
        page[4] = 4;
        page[5] = 0;
        page[6..8].copy_from_slice(&(len as u16).to_le_bytes());
        page[8..12].copy_from_slice(&crc32(&payload[..len]).to_le_bytes());
        page[PROVISIONING_HEADER..PROVISIONING_HEADER + len].copy_from_slice(&payload[..len]);
        let p = decode_provisioning(&page).unwrap();
        assert_eq!(
            p.config,
            NodeConfig {
                role: Role::Anchor,
                sink: true,
                diversity: false,
                pinned: Some(PointMm { x: 1, y: 2, z: 3 }),
                adv_interval_ms: 500,
                env_interval_s: 0,
                gas: true,
            }
        );
        assert_eq!(
            (p.sink, p.number, p.report_level),
            (Some(NodeId(7)), Some(3), ReportLevel::Info)
        );
    }

    #[test]
    fn node_config_roundtrips() {
        let cfg = NodeConfig {
            diversity: false,
            pinned: Some(PointMm {
                x: 1200,
                y: -300,
                z: 2700,
            }),
            gas: true,
            ..NodeConfig::default()
        };
        let mut buf = [0u8; 32];
        let bytes = postcard::to_slice(&cfg, &mut buf).unwrap();
        assert_eq!(postcard::from_bytes::<NodeConfig>(bytes).unwrap(), cfg);
    }
}
