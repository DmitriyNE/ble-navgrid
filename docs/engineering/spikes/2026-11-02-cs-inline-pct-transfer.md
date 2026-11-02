# CS inline PCT transfer (IPT) on the TAGs

**Question.** Can the initiator range on its own with inline PCT transfer? With IPT the reflector
returns each tone with the phase it measured added, so the initiator's own PCTs carry the round
trip. No reflector data has to travel back over the ACL.

## Setup

- **Boards:** the TAG pair and DKs of
  [the subevent-aborts spike](2026-11-02-cs-subevent-aborts.md), about 90 cm apart.
- **Firmware:** `4e4c3c8` plus:
  - the SDC extended feature set, as NCS's IPT samples enable it (SDC memory 95,448 → 99,072 B);
  - `CS_Enhancements_1` bit 0 in Create Config;
  - an on-device estimate on the initiator: the mode-2 PCTs of each procedure go through
    `navgrid_ranging::phase_slope`.
- **Host check:** `navgrid cs --ipt` runs the same estimate on the initiator's capture.
- **Reference:** Nordic's `ipt_initiator` / `ipt_reflector` (NCS v3.4.1, SDC v3.4.1).

## Results

| Initiator | Reflector | Initiator-only estimate | Notes |
|---|---|---|---|
| ours, SDC v3.4.0 | ours, SDC v3.4.0 | coherence 0.17–0.34, 0.2–148 m | Both sides echo IPT in the config. The two-sided product still gives a clean 3.38 m. |
| Nordic | Nordic | 3.40 m, a new estimate every 30 ms | IFFT, as the sample computes it. |
| ours, SDC v3.4.0 | Nordic | coherence 0.97, median 5.73 m | Our initiator side works. |
| Nordic | ours, SDC v3.4.0 | median 2.98 m, p10 0.51 m | Our reflector applies the IPT phase wrongly. Its v2 capabilities advertise IPT exactly like Nordic's. |
| ours, SDC v3.4.1 | ours, SDC v3.4.1 | **median 3.88 m** (p10–p90 3.83–3.95), coherence 0.986 | The on-device value equals the host `--ipt` value. |

## Conclusion

- **The v3.4.0 reflector is at fault.** It applies the IPT phase wrongly when several antenna
  paths are configured (ours: 2 antennas, 4 paths). This is DRGN-29459, fixed in SDC v3.4.1.
- **Our side is fine.** With v3.4.1 on both TAGs, the initiator ranges alone. The on-device
  decoder and estimator match the host exactly.
- **Offsets may differ per mode:** IPT read about 0.5 m above the two-sided estimate at the same
  placement, so calibration should be per mode.

The nrf-sdc fork has a local branch `update-nrfxlib-3.4.1` (`e6da399`), not pushed. Until the pin
moves, `INLINE_PCT` stays off in main (DEF-24).
