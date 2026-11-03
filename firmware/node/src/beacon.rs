//! The FR-M4 beacon in this node's advertisements (`navgrid_proto::Beacon`), rebuilt from the
//! node's status whenever advertising starts. A button press restarts advertising, fast for the
//! attention period, so a field tool nearby sees it at once (architecture §13).
//!
//! Advertising uses the extended commands, as connecting and scanning do: the SDC refuses to mix
//! legacy and extended ones (DEF-50), and a reflector or a sink also connects as central. The
//! set is connectable and not scannable, so the beacon carries no name.

use defmt::unwrap;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use navgrid_proto::{BEACON_COMPANY, Beacon, BeaconPosition, NodeConfig, Role};
use nrf_sdc::SoftdeviceController;
use trouble_host::prelude::*;

use crate::status;

const FAST: Duration = Duration::from_millis(100);

pub struct Beacons {
    cfg: NodeConfig,
    number: Option<u16>,
    max_links: u8,
}

impl Beacons {
    pub fn new(cfg: NodeConfig, number: Option<u16>, max_links: u8) -> Self {
        Self { cfg, number, max_links }
    }

    /// The beacon as of now. Until the mesh state runs on devices only a sink knows its hop
    /// count, and pressure isn't measured yet.
    pub fn beacon(&self) -> Beacon {
        let dm = |mm: i32| (mm / 100).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
        Beacon {
            role: self.cfg.role,
            attention: status::attention_left_s() > 0,
            install: false,
            hops: if self.cfg.role == Role::Sink { 0 } else { u8::MAX },
            free_links: self.max_links.saturating_sub(status::links()),
            battery_mv: status::battery_mv(),
            pressure_dpa: None,
            number: self.number,
            position: self.cfg.pinned.map(|p| BeaconPosition {
                point_dm: [dm(p.x), dm(p.y), dm(p.z)],
                sigma_dm: 1,
                frame: 0,
            }),
        }
    }

    /// Advertises until a central connects, starting over with fresh data whenever the beacon
    /// changes, and at 100 ms while the attention flag is up.
    pub async fn advertise<'d>(
        &self,
        peripheral: &mut Peripheral<'d, SoftdeviceController<'_>, DefaultPacketPool>,
        slow: &AdvertisementParameters,
    ) -> Connection<'d, DefaultPacketPool> {
        loop {
            let beacon = self.beacon().encode();
            let mut adv_data = [0; 31];
            let adv_len = unwrap!(AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ManufacturerSpecificData {
                        company_identifier: BEACON_COMPANY,
                        payload: &beacon,
                    },
                ],
                &mut adv_data[..],
            ));
            let attention = status::attention_left_s();
            let fast = AdvertisementParameters {
                interval_min: FAST,
                interval_max: FAST,
                ..*slow
            };
            let params = if attention > 0 { &fast } else { slow };
            status::BEACON_CHANGED.reset();
            let sets = [AdvertisementSet {
                params: *params,
                data: Advertisement::ExtConnectableNonscannableUndirected {
                    adv_data: &adv_data[..adv_len],
                },
                address: None,
            }];
            let mut handles = AdvertisementSet::handles(&sets);
            let advertiser = unwrap!(peripheral.advertise_ext(&sets, &mut handles).await);
            let changed = async {
                if attention > 0 {
                    select(status::BEACON_CHANGED.wait(), Timer::after_secs(u64::from(attention))).await;
                } else {
                    status::BEACON_CHANGED.wait().await;
                }
            };
            // Dropping the advertiser stops advertising.
            if let Either::First(conn) = select(advertiser.accept(), changed).await {
                return unwrap!(conn);
            }
        }
    }
}
