//! Wire messages shared by firmware and host.
//!
//! Messages are postcard-encoded. On byte streams (UART, RTT, capture files) each message is one
//! COBS frame terminated by `0x00`. Lengths and coordinates are integer millimetres so firmware
//! never formats floats.

#![no_std]

use serde::{Deserialize, Serialize};

/// Bumped on any incompatible change to [`Message`].
pub const PROTOCOL_VERSION: u16 = 1;

/// Upper bound for one encoded frame, including COBS overhead and the `0x00` terminator.
pub const MAX_FRAME_LEN: usize = 64;

/// L2CAP LE credit-based channel (SPSM) that carries telemetry toward a sink (D-6). Each SDU is
/// one encoded frame, terminator included.
pub const TELEMETRY_SPSM: u16 = 0x0081;

/// Device identity: the 48-bit BLE device address in the low bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NodeId(pub u64);

/// The controller (the host) as the origin of mesh records. No device has this ID: static random
/// addresses have their top two bits set.
pub const CONTROLLER: NodeId = NodeId(0);

/// Numbered as in a [`Beacon`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Role {
    /// Static mesh member: ranges with neighbors, serves as a reference for tags.
    Node,
    /// Tag on a thing (keys): still most of the time; fixes and publishes its position when it settles.
    Asset,
    /// Tag worn by a person with a HUD: tracked continuously while moving, with heading.
    Person,
    /// USB-powered gateway that collects telemetry for the host.
    Sink,
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
            role: Role::Node,
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
}

/// Where the [`Provisioning`] page lives in RRAM: the second-to-last 4 KiB, which the firmware's
/// `memory.x` keeps out of the image. Flashing firmware leaves it alone.
pub const PROVISIONING_ADDRESS: u32 = 0x0017_B000;
/// The provisioning page's header: magic, format, payload length, CRC-32 of the payload.
const PROVISIONING_MAGIC: [u8; 4] = *b"ngpv";
const PROVISIONING_FORMAT: u8 = 1;
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
    /// Written by a newer `navgrid provision`.
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
    if header[4] != PROVISIONING_FORMAT {
        return Err(ProvisioningError::Format(header[4]));
    }
    let len = usize::from(u16::from_le_bytes([header[6], header[7]]));
    let crc = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
    let payload = page
        .get(PROVISIONING_HEADER..PROVISIONING_HEADER + len)
        .ok_or(ProvisioningError::Corrupt)?;
    if crc32(payload) != crc {
        return Err(ProvisioningError::Corrupt);
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
pub const BEACON_LEN: usize = 17;
const BEACON_FORMAT: u8 = 1;

/// What a node advertises (FR-M4), so that a tag can choose nodes and get a height reference
/// without a directory.
///
/// Layout after the company identifier, little-endian: format (1), role, hops, free links,
/// battery (20 mV steps above 2.0 V), pressure (0.1 Pa, 3 bytes), position x, y, z (decimetres,
/// 2 bytes each), position σ (decimetres), frame version (2 bytes). All-ones fields are unknown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Beacon {
    pub role: Role,
    /// Hops to the nearest sink; 255 with no path.
    pub hops: u8,
    /// Links it can still take (admission control, NFR-4).
    pub free_links: u8,
    pub battery_mv: Option<u16>,
    pub pressure_dpa: Option<u32>,
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
        b[2] = self.hops;
        b[3] = self.free_links;
        if let Some(mv) = self.battery_mv {
            b[4] = (mv.saturating_sub(2000) / 20).min(254) as u8;
        }
        if let Some(p) = self.pressure_dpa {
            b[5..8].copy_from_slice(&p.min(0xff_fffe).to_le_bytes()[..3]);
        }
        if let Some(pos) = self.position {
            for (i, v) in pos.point_dm.iter().enumerate() {
                b[8 + 2 * i..10 + 2 * i].copy_from_slice(&v.to_le_bytes());
            }
            b[14] = pos.sigma_dm.min(254);
            b[15..17].copy_from_slice(&pos.frame.to_le_bytes());
        }
        b
    }

    pub fn decode(b: &[u8]) -> Option<Self> {
        let b: &[u8; BEACON_LEN] = b.get(..BEACON_LEN)?.try_into().ok()?;
        if b[0] != BEACON_FORMAT {
            return None;
        }
        let role = match b[1] {
            0 => Role::Node,
            1 => Role::Asset,
            2 => Role::Person,
            3 => Role::Sink,
            _ => return None,
        };
        let i16_at = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        let pressure = u32::from_le_bytes([b[5], b[6], b[7], 0]);
        Some(Self {
            role,
            hops: b[2],
            free_links: b[3],
            battery_mv: (b[4] != 0xff).then(|| 2000 + 20 * u16::from(b[4])),
            pressure_dpa: (pressure != 0xff_ffff).then_some(pressure),
            position: (b[14] != 0xff).then(|| BeaconPosition {
                point_dm: [i16_at(8), i16_at(10), i16_at(12)],
                sigma_dm: b[14],
                frame: u16::from_le_bytes([b[15], b[16]]),
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
    pub firmware: u32,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Message {
    Hello {
        node: NodeId,
        role: Role,
        protocol: u16,
        /// Antennas in use for Channel Sounding (1 when diversity is off).
        antennas: u8,
    },
    Range(Range),
    Position(Position),
    Health(Health),
    Environment(Environment),
    Link(LinkStats),
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

    fn worst_case() -> [Message; 6] {
        let id = NodeId(u64::MAX);
        [
            Message::Hello {
                node: id,
                role: Role::Person,
                protocol: u16::MAX,
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
                firmware: u32::MAX,
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
        ]
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
                role: Role::Sink,
                pinned: Some(PointMm { x: 1, y: 2, z: 3 }),
                ..NodeConfig::default()
            },
            fleet_key: [0xa5; 16],
            peer: Some(NodeId(0xc0ff_ee12_3456)),
            sink: None,
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
        newer[4] = 2;
        assert_eq!(decode_provisioning(&newer), Err(ProvisioningError::Format(2)));
        assert_eq!(
            decode_provisioning(&page[..page.len() - 1]),
            Err(ProvisioningError::Corrupt)
        );
    }

    #[test]
    fn crc32_matches_zlib() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn beacon_roundtrips_with_unknowns() {
        let full = Beacon {
            role: Role::Node,
            hops: 3,
            free_links: 9,
            battery_mv: Some(2980),
            pressure_dpa: Some(1_013_250),
            position: Some(BeaconPosition {
                point_dm: [-120, 345, 27],
                sigma_dm: 3,
                frame: 7,
            }),
        };
        assert_eq!(Beacon::decode(&full.encode()), Some(full));
        let bare = Beacon {
            role: Role::Sink,
            hops: 0,
            free_links: 14,
            battery_mv: None,
            pressure_dpa: None,
            position: None,
        };
        assert_eq!(Beacon::decode(&bare.encode()), Some(bare));
        assert_eq!(Beacon::decode(&[2; BEACON_LEN]), None);
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
