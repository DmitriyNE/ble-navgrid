//! CS step data as controllers report it in the LE CS Subevent Result and Subevent Result Continue
//! events (Core 6.0, Vol 4, Part E, 7.7.65.44–45): per step a mode, a channel, a length, and
//! data whose layout depends on the mode and on which side reported it.
//!
//! With several antenna paths, a step's tones come in the order its antenna permutation index
//! selects (Core 6.0, Vol 6, Part H, Tables 4.13–4.15); [`Tones::path`] undoes that.

use crate::Complex;

/// Most antenna paths a CS configuration can have.
pub const MAX_ANTENNA_PATHS: u8 = 4;

/// Which side reported the step. Mode-0 data differs between them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Role {
    Initiator,
    Reflector,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepError {
    /// The step bytes end inside a step.
    Truncated,
    /// The data length doesn't fit the mode, role and antenna path count.
    Length { mode: u8, len: usize },
    /// A mode outside 0–3.
    Mode(u8),
    /// More antenna paths than CS allows.
    Paths(u8),
}

/// One step as the event frames it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawStep<'a> {
    pub mode: u8,
    pub channel: u8,
    pub data: &'a [u8],
}

/// Splits step bytes (`mode, channel, length, data` per step) into steps. Bytes from consecutive
/// subevents of one procedure can be concatenated first: each step carries its own length.
pub fn split(bytes: &[u8]) -> Split<'_> {
    Split(bytes)
}

pub struct Split<'a>(&'a [u8]);

impl<'a> Iterator for Split<'a> {
    type Item = Result<RawStep<'a>, StepError>;

    fn next(&mut self) -> Option<Self::Item> {
        let bytes = core::mem::take(&mut self.0);
        if bytes.is_empty() {
            return None;
        }
        let [mode, channel, len, rest @ ..] = bytes else {
            return Some(Err(StepError::Truncated));
        };
        let Some((data, tail)) = rest.split_at_checked(usize::from(*len)) else {
            return Some(Err(StepError::Truncated));
        };
        self.0 = tail;
        Some(Ok(RawStep {
            mode: *mode,
            channel: *channel,
            data,
        }))
    }
}

/// Tone quality as the controller grades it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ToneQuality {
    High,
    Medium,
    Low,
    Unavailable,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tone {
    /// Phase correction term: the received tone as I/Q, 12 bits each.
    pub pct: Complex,
    pub quality: ToneQuality,
    /// 0: not the tone extension slot; 1: extension slot, no tone expected; 2: extension slot,
    /// tone expected.
    pub extension: u8,
}

/// The tones of one mode-2 or mode-3 step: one per antenna path, then the tone extension slot.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tones {
    pub permutation: u8,
    paths: u8,
    tones: [Tone; MAX_ANTENNA_PATHS as usize + 1],
}

impl Tones {
    pub fn paths(&self) -> u8 {
        self.paths
    }

    /// The tone measured on antenna path `path` (0-based). `None` if `path` is out of range or
    /// the permutation index is invalid for the path count.
    pub fn path(&self, path: u8) -> Option<Tone> {
        (0..self.paths)
            .find(|&i| antenna_path(self.paths, self.permutation, i) == Some(path))
            .map(|i| self.tones[usize::from(i)])
    }

    /// The tone in the extension slot, after the antenna-path tones.
    pub fn extension_slot(&self) -> Tone {
        self.tones[usize::from(self.paths)]
    }
}

/// Antenna path permutations, Core 6.0, Vol 6, Part H, Tables 4.13–4.15: row = permutation
/// index, column = tone position in the step, value = 0-based antenna path. The last column is
/// the tone extension slot. One path has the single permutation `[0, 0]`.
const PERMUTATIONS_2: [[u8; 3]; 2] = [[0, 1, 1], [1, 0, 0]];
const PERMUTATIONS_3: [[u8; 4]; 6] = [
    [0, 1, 2, 2],
    [1, 0, 2, 2],
    [0, 2, 1, 1],
    [2, 0, 1, 1],
    [2, 1, 0, 0],
    [1, 2, 0, 0],
];
const PERMUTATIONS_4: [[u8; 5]; 24] = [
    [0, 1, 2, 3, 3],
    [1, 0, 2, 3, 3],
    [0, 2, 1, 3, 3],
    [2, 0, 1, 3, 3],
    [2, 1, 0, 3, 3],
    [1, 2, 0, 3, 3],
    [0, 1, 3, 2, 2],
    [1, 0, 3, 2, 2],
    [0, 3, 1, 2, 2],
    [3, 0, 1, 2, 2],
    [3, 1, 0, 2, 2],
    [1, 3, 0, 2, 2],
    [0, 3, 2, 1, 1],
    [3, 0, 2, 1, 1],
    [0, 2, 3, 1, 1],
    [2, 0, 3, 1, 1],
    [2, 3, 0, 1, 1],
    [3, 2, 0, 1, 1],
    [3, 1, 2, 0, 0],
    [1, 3, 2, 0, 0],
    [3, 2, 1, 0, 0],
    [2, 3, 1, 0, 0],
    [2, 1, 3, 0, 0],
    [1, 2, 3, 0, 0],
];

/// The antenna path of the tone at `tone` (0-based position in the step, `paths` being the
/// extension slot) for a step with `paths` antenna paths and antenna permutation index
/// `permutation`. `None` for an invalid combination.
pub fn antenna_path(paths: u8, permutation: u8, tone: u8) -> Option<u8> {
    let (p, t) = (usize::from(permutation), usize::from(tone));
    match paths {
        1 => (p == 0 && t <= 1).then_some(0),
        2 => PERMUTATIONS_2.get(p)?.get(t).copied(),
        3 => PERMUTATIONS_3.get(p)?.get(t).copied(),
        4 => PERMUTATIONS_4.get(p)?.get(t).copied(),
        _ => None,
    }
}

/// Mode-1 (RTT) measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rtt {
    /// Bits 0–3: CS access address check; bits 4–7: bit errors.
    pub quality: u8,
    /// Normalized attack detector metric; 0xFF when unavailable.
    pub nadm: u8,
    pub rssi_dbm: Option<i8>,
    /// The initiator's ToA−ToD or the reflector's ToD−ToA, in 0.5 ns.
    pub toa_tod: i16,
    pub antenna: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Step {
    /// Frequency calibration. `freq_offset` (initiator only) is in 0.01 ppm.
    Mode0 {
        quality: u8,
        rssi_dbm: Option<i8>,
        antenna: u8,
        freq_offset: Option<i16>,
    },
    Mode1(Rtt),
    Mode2(Tones),
    Mode3(Rtt, Tones),
}

/// Decodes one step's data. `paths` is the subevent's antenna path count.
pub fn decode(role: Role, paths: u8, step: RawStep<'_>) -> Result<Step, StepError> {
    if !(1..=MAX_ANTENNA_PATHS).contains(&paths) {
        return Err(StepError::Paths(paths));
    }
    let d = step.data;
    let bad_length = StepError::Length {
        mode: step.mode,
        len: d.len(),
    };
    let tones_len = 1 + 4 * (usize::from(paths) + 1);
    match step.mode {
        0 => match (role, d) {
            (Role::Initiator, &[quality, rssi, antenna, f0, f1]) => Ok(Step::Mode0 {
                quality,
                rssi_dbm: rssi_dbm(rssi),
                antenna,
                freq_offset: Some(sign15(u16::from_le_bytes([f0, f1]))),
            }),
            (Role::Reflector, &[quality, rssi, antenna]) => Ok(Step::Mode0 {
                quality,
                rssi_dbm: rssi_dbm(rssi),
                antenna,
                freq_offset: None,
            }),
            _ => Err(bad_length),
        },
        1 => rtt(d).ok_or(bad_length).map(Step::Mode1),
        2 => tones(paths, d).ok_or(bad_length).map(Step::Mode2),
        3 => {
            let split = d.len().checked_sub(tones_len).ok_or(bad_length)?;
            let (r, t) = d.split_at(split);
            Ok(Step::Mode3(
                rtt(r).ok_or(bad_length)?,
                tones(paths, t).ok_or(bad_length)?,
            ))
        }
        mode => Err(StepError::Mode(mode)),
    }
}

/// The RTT fields, with or without the two sounding-sequence PCTs (6 or 14 octets).
fn rtt(d: &[u8]) -> Option<Rtt> {
    let (&[quality, nadm, rssi, t0, t1, antenna], rest) = d.split_first_chunk::<6>()?;
    (rest.is_empty() || rest.len() == 8).then(|| Rtt {
        quality,
        nadm,
        rssi_dbm: rssi_dbm(rssi),
        toa_tod: i16::from_le_bytes([t0, t1]),
        antenna,
    })
}

fn tones(paths: u8, d: &[u8]) -> Option<Tones> {
    let (&permutation, rest) = d.split_first()?;
    if rest.len() != 4 * (usize::from(paths) + 1) {
        return None;
    }
    let mut tones = [Tone {
        pct: Complex::ZERO,
        quality: ToneQuality::Unavailable,
        extension: 0,
    }; MAX_ANTENNA_PATHS as usize + 1];
    for (tone, b) in tones.iter_mut().zip(rest.chunks_exact(4)) {
        *tone = Tone {
            pct: pct([b[0], b[1], b[2]]),
            quality: match b[3] & 0x0f {
                0 => ToneQuality::High,
                1 => ToneQuality::Medium,
                2 => ToneQuality::Low,
                _ => ToneQuality::Unavailable,
            },
            extension: b[3] >> 4,
        };
    }
    Some(Tones {
        permutation,
        paths,
        tones,
    })
}

/// Decodes a phase correction term: I in bits 0–11, Q in bits 12–23, both 12-bit two's
/// complement.
pub fn pct(bytes: [u8; 3]) -> Complex {
    let raw = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0]);
    let i = ((raw & 0xfff) as i16) << 4 >> 4;
    let q = ((raw >> 12) as i16) << 4 >> 4;
    Complex::new(f32::from(i), f32::from(q))
}

/// Encodes a phase correction term, rounding and clamping I and Q to 12 bits. For simulations
/// and tests.
pub fn encode_pct(c: Complex) -> [u8; 3] {
    let quantize = |v: f32| (libm::roundf(v).clamp(-2048.0, 2047.0) as i32 as u32) & 0xfff;
    let raw = quantize(c.re) | quantize(c.im) << 12;
    let [b0, b1, b2, _] = raw.to_le_bytes();
    [b0, b1, b2]
}

/// A frequency offset or compensation (15-bit two's complement, 0.01 ppm) in ppm; `None` for the
/// "not available" value 0xC000.
pub fn freq_ppm(raw: u16) -> Option<f32> {
    (raw != 0xc000).then(|| f32::from(sign15(raw)) / 100.0)
}

fn sign15(raw: u16) -> i16 {
    ((raw << 1) as i16) >> 1
}

fn rssi_dbm(raw: u8) -> Option<i8> {
    (raw != 0x7f).then_some(raw as i8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_concatenated_steps() {
        let mut steps = split(&[0, 10, 3, 1, 2, 3, 2, 11, 1, 9]);
        let step = |mode, channel, data| Some(Ok(RawStep { mode, channel, data }));
        assert_eq!(steps.next(), step(0, 10, &[1, 2, 3]));
        assert_eq!(steps.next(), step(2, 11, &[9]));
        assert_eq!(steps.next(), None);
        let mut truncated = split(&[2, 11, 5, 0]);
        assert_eq!(truncated.next(), Some(Err(StepError::Truncated)));
        assert_eq!(truncated.next(), None);
        assert_eq!(split(&[2, 11]).next(), Some(Err(StepError::Truncated)));
    }

    #[test]
    fn pct_round_trips_and_sign_extends() {
        // I = −1 (0xfff), Q = 2047 (0x7ff): raw 0x7ff_fff.
        assert_eq!(pct([0xff, 0xff, 0x7f]), Complex::new(-1.0, 2047.0));
        assert_eq!(pct([0x00, 0x08, 0x80]), Complex::new(-2048.0, -2048.0));
        for c in [
            Complex::new(512.4, -300.6),
            Complex::new(-2047.0, 0.0),
            Complex::new(5000.0, -5000.0),
        ] {
            let back = pct(encode_pct(c));
            assert!(
                libm::fabsf(back.re - c.re.clamp(-2048.0, 2047.0)) <= 0.5,
                "{c:?} {back:?}"
            );
            assert!(
                libm::fabsf(back.im - c.im.clamp(-2048.0, 2047.0)) <= 0.5,
                "{c:?} {back:?}"
            );
        }
    }

    #[test]
    fn decodes_each_mode() {
        let raw = |mode, data| RawStep {
            mode,
            channel: 30,
            data,
        };
        // Mode 0: the initiator adds the frequency offset (−6.06 ppm = 0x7da2 as 15 bits).
        assert_eq!(
            decode(Role::Initiator, 1, raw(0, &[0, 0xe2, 0, 0xa2, 0x7d])),
            Ok(Step::Mode0 {
                quality: 0,
                rssi_dbm: Some(-30),
                antenna: 0,
                freq_offset: Some(-606)
            })
        );
        assert!(matches!(
            decode(Role::Reflector, 1, raw(0, &[0, 0x7f, 1])),
            Ok(Step::Mode0 {
                rssi_dbm: None,
                freq_offset: None,
                ..
            })
        ));
        assert_eq!(
            decode(Role::Reflector, 1, raw(0, &[0, 0, 0, 0, 0])),
            Err(StepError::Length { mode: 0, len: 5 })
        );
        // Mode 1, with and without the sounding-sequence PCTs.
        let mode1 = [0, 0xff, 0xd8, 0x10, 0x01, 0];
        assert!(matches!(
            decode(Role::Initiator, 1, raw(1, &mode1)),
            Ok(Step::Mode1(Rtt { toa_tod: 272, .. }))
        ));
        let mut long = [0u8; 14];
        long[..6].copy_from_slice(&mode1);
        assert!(decode(Role::Initiator, 1, raw(1, &long)).is_ok());
        assert!(decode(Role::Initiator, 1, raw(1, &mode1[..5])).is_err());
        // Mode 2, one path: permutation, path tone, extension slot.
        let mut mode2 = [0u8; 9];
        mode2[1..4].copy_from_slice(&encode_pct(Complex::new(100.0, -200.0)));
        mode2[4] = 1; // medium
        mode2[8] = 0x13; // extension slot, no tone expected; unavailable
        let Ok(Step::Mode2(t)) = decode(Role::Reflector, 1, raw(2, &mode2)) else {
            panic!("mode 2");
        };
        assert_eq!(
            t.path(0).map(|t| (t.pct, t.quality)),
            Some((Complex::new(100.0, -200.0), ToneQuality::Medium))
        );
        assert_eq!(t.path(1), None);
        assert_eq!(t.extension_slot().extension, 1);
        assert_eq!(t.extension_slot().quality, ToneQuality::Unavailable);
        // Four paths, permutation 7 ([A2, A1, A4, A3]): tone n carries a marker n + 1.
        let mut four = [0u8; 21];
        four[0] = 7;
        for n in 0..5 {
            four[1 + 4 * n..4 + 4 * n].copy_from_slice(&encode_pct(Complex::new(n as f32 + 1.0, 0.0)));
        }
        let step = |data| RawStep {
            mode: 2,
            channel: 30,
            data,
        };
        let Ok(Step::Mode2(t)) = decode(Role::Initiator, 4, step(&four)) else {
            panic!("mode 2, four paths");
        };
        let marker = |path| t.path(path).map(|t| t.pct.re);
        assert_eq!(
            [marker(0), marker(1), marker(2), marker(3), marker(4)],
            [Some(2.0), Some(1.0), Some(4.0), Some(3.0), None]
        );
        // An index beyond the table maps nothing.
        let mut bad = four;
        bad[0] = 24;
        let Ok(Step::Mode2(t)) = decode(Role::Initiator, 4, step(&bad)) else {
            panic!("mode 2, bad permutation");
        };
        assert_eq!(t.path(0), None);
        // Mode 3 = mode 1 then mode 2.
        let mut mode3 = [0u8; 15];
        mode3[..6].copy_from_slice(&mode1);
        mode3[6..].copy_from_slice(&mode2);
        assert!(matches!(
            decode(Role::Initiator, 1, raw(3, &mode3)),
            Ok(Step::Mode3(Rtt { toa_tod: 272, .. }, _))
        ));
        assert_eq!(decode(Role::Initiator, 1, raw(4, &[])), Err(StepError::Mode(4)));
        assert_eq!(decode(Role::Initiator, 5, raw(2, &[])), Err(StepError::Paths(5)));
    }

    #[test]
    fn permutation_tables_are_permutations() {
        for paths in 1..=MAX_ANTENNA_PATHS {
            let rows = (0..).take_while(|&p| antenna_path(paths, p, 0).is_some());
            let mut count = 0;
            for p in rows {
                count += 1;
                let mut row = [0u8; MAX_ANTENNA_PATHS as usize + 1];
                for t in 0..=paths {
                    row[usize::from(t)] = antenna_path(paths, p, t).unwrap();
                }
                // Each path once among the first `paths` tones.
                let seen = row[..usize::from(paths)].iter().fold(0u8, |m, &a| m | 1 << a);
                assert_eq!(seen, (1u8 << paths) - 1, "paths {paths}, row {p}");
                // The extension slot repeats the last path's antenna.
                assert_eq!(row[usize::from(paths)], row[usize::from(paths) - 1]);
                assert_eq!(antenna_path(paths, p, paths + 1), None);
            }
            // N_AP! permutations.
            assert_eq!(count, (1..=usize::from(paths)).product::<usize>(), "paths {paths}");
        }
    }

    #[test]
    fn frequency_values() {
        assert_eq!(freq_ppm(0xc000), None);
        assert_eq!(freq_ppm(0x7da2), Some(-6.06));
        assert_eq!(freq_ppm(0x2710), Some(100.0));
        assert_eq!(freq_ppm(0x58f0), Some(-100.0));
    }
}
