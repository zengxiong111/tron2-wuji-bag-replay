# Hardware Identity, Joint Mapping, and Limits

简体中文：[HARDWARE.md](HARDWARE.md)。

This document defines the hardware inventory read by the bag-replay tooling. The current inventory blocks unconfirmed physical playback; it is not a device configuration or commissioning record.

## Contents

- [Status and conclusion](#status-and-conclusion)
- [Facts confirmed by local evidence](#facts-confirmed-by-local-evidence)
- [Device information still required](#device-information-still-required)
- [Configuration files](#configuration-files)
- [Admission rules](#admission-rules)
- [Official references](#official-references)

## Status and conclusion

**The actual device revisions, firmware, serial numbers, complete physical joint mappings, and per-axis hardware limits cannot currently be confirmed.** The delivery contains the retimed trajectory, model files, and source bag metadata; the original bag is not in the current runtime bundle. It has no device identity record, revision-bound parameter table, or operator confirmation. Do not treat URDF/USD model limits as physical ratings.

| Device information | Current status | Evidence boundary |
| --- | --- | --- |
| TRON2 body | The bag has 16 `/motor/state` `q` channels; the message declares no names | Association with model joints remains a candidate mapping |
| Head channels | The local model/bag candidate order is yaw, pitch; the TRON2 manual numbers 14=pitch and 15=yaw | Conflict remains; the preceding targets were zero; this file contains their actual feedback, which still does not confirm the mapping |
| Wuji Hand2 | The bag has 40 named position channels; `left_`/`right_` prefixes correspond to model `l_`/`r_` names | Only bag-name normalization is confirmed; mapping to the present devices and SDK S1–S4/actuator numbers is unconfirmed |
| Position/speed limits | Configuration includes delivered model angle and speed limits | These are model/documentation values, not confirmed physical limits |
| Acceleration, rated current, torque, temperature | No per-axis values bound to the installed device revision are available | Cannot be used for hardware admission |

The public Wuji 2.2 Beta control material gives `effort` in A and an allowed hard ceiling of 2.0 A. This is a control boundary, not a default command or per-axis rated current. 1000 Hz is a control/communication frequency, not a mechanical speed limit. TRON2 documentation uses rad, rad/s, and estimated N·m for `q`, `dq`, and `tau`; manual endpoint speed/acceleration specifications do not replace per-axis motor limits.

## Facts confirmed by local evidence

| Item | Known fact | Source |
| --- | --- | --- |
| Input bag | Source `2026-10-08-17-54-13.bag` (not distributed in the current bundle); SHA-256 `2af6c090b36e26231aff0b2b102768c7224044cd260b8bc07486aa4167fe55bd` | `configs/bag_info.json` |
| Body channels | 16-dimensional `/motor/state q`, with no joint names in the message | `configs/bag_info.json` |
| Hand channels | 40-dimensional `/wuji/hand/state hand_state.pos`, with source names | `configs/bag_info.json` |
| Bag device label | All 44,058 hand-state messages declare `hand_type=wuji/hand2`; nested frames are left/right | Original bag; `configs/hardware_inventory.json`. Recorded labels only; Beta revision, firmware and current devices are unconfirmed |
| Model limits | Angle and speed data listed for 56 model joints | `evidence/motor_limits/joint_limits.csv` |

## Device information still required

Before admitting any hardware player, the device owner must provide and review:

1. Exact product variants, hardware revisions, firmware versions, and asset IDs/serial numbers for TRON2 and each Wuji hand.
2. A row-by-row mapping from bag channels and model joints to physical SDK joint names/actuator numbers. Confirm the 16-channel TRON2 body order, resolve the separate head 14/15 conflict, and confirm the relation between the 40 named hand joints and SDK S1–S4/actuators.
3. Per-axis position, speed, acceleration, current, and torque limits applicable to those revisions, with units, continuous/peak definitions, source version, and document page.
4. A confirmation record signed by an authorized device owner or stored in a controlled configuration repository; then recompute trajectory admission against the hardware limits.

No device was connected, and no SDK query, enable command, or movement was performed for this work. The available bag and public material alone cannot provide these confirmations.

## Configuration files

| File | Purpose |
| --- | --- |
| `configs/hardware_inventory.json` | Device identity, evidence status, public-document boundaries, and required admission fields; `admission_ready` is currently `false` |
| `configs/joint_mapping.csv` | Bag raw channel, device group, and pending physical/SDK mapping for all 56 model joints. Unconfirmed fields are blank and the status column says candidate or unconfirmed |
| `configs/hardware_limits.csv` | Model angle/speed values and blank physical-limit columns for all 56 joints. `hardware_limit_status` is `unconfirmed` for every row |

`bag_hand_index` and `bag_pos_index` identify the nested hand and position indices in the bag; `bag_raw_index` is the flattened 40-channel index. Physical direction, zero offset, unit and evidence source remain blank independently and cannot be inferred from matching names. Left and right hands have separate device identity records; temperature and sustained-load conditions must also be supplied.

Offline check (repository root; the current expected exit code is `2` because device evidence is unconfirmed):

```bash
python3 scripts/check_hardware_admission.py
```

This command checks offline inventory and trajectory evidence completeness only. It neither connects to devices nor authorizes hardware execution. Live commissioning is separately reported as `live_commissioning_verified=false`. Run `python3 -m unittest discover -s tests -v` to check rejection of unknown values, invalid numbers, incorrect joint sets, wrong device assignments and channel indices.

A blank physical limit means unknown, not unlimited or zero. Player admission must treat blank values as a rejection condition; it must not fall back to model limits, general manual performance, or the 2.0 A control ceiling.

## Admission rules

The player may enter a separately authorized hardware workflow only after all of these conditions hold: device identity and mappings are confirmed; every commanded physical channel has revision-bound per-axis limits and units; the trajectory is rechecked against those hardware limits; static checks and controlled low-speed commissioning records pass separately; and the device owner approves the run. Any missing item must block enabling and playback.

This configuration establishes unknown fields and admission boundaries only. It does not prove continuous collision avoidance, payload capability, thermal behavior, closed-loop task success, or Sim2Real.

## Official references

- [Wuji control guide](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/)
- [Wuji SDK reference](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [Wuji user notice](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/)
- [TRON2 SDK guide](https://www.limxdynamics.com/zh/documents/847884267345285120)
- [TRON2 user manual](https://www.limxdynamics.com/zh/documents/844648486841487360)
