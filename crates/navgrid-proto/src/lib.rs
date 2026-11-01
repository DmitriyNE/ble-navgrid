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

/// Device identity: the 48-bit BLE device address in the low bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NodeId(pub u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
