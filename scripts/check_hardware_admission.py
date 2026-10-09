"""Validate offline evidence completeness without connecting to any device."""
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def known(value):
    return str(value or '').strip().lower() not in {'', 'unknown', 'n/a', 'na', 'none', 'null', 'pending', 'unconfirmed'}


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def evaluate(root):
    config = root / 'configs'
    inventory = json.loads((config / 'hardware_inventory.json').read_text())
    with (config / 'joint_mapping.csv').open(newline='') as stream:
        mapping = list(csv.DictReader(stream))
    with (config / 'hardware_limits.csv').open(newline='') as stream:
        limits = list(csv.DictReader(stream))
    reasons = []
    expected = {joint.attrib['name'] for joint in ET.parse(root / 'outputs/robot/tron2_wuji_hand2.urdf').getroot().findall('joint') if joint.attrib['type'] == 'revolute'}
    for label, rows in [('mapping', mapping), ('limits', limits)]:
        names = [row['model_joint'] for row in rows]
        if len(names) != 56 or len(set(names)) != 56 or set(names) != expected:
            reasons.append(f'{label}: requires the exact 56 unique model revolute joints')
    if {row['model_joint'] for row in mapping} != {row['model_joint'] for row in limits}:
        reasons.append('mapping and limit joint sets differ')
    devices = inventory.get('devices', [])
    if {device.get('device_id') for device in devices} != {'tron2', 'wuji_left', 'wuji_right'}:
        reasons.append('requires separate TRON2, left Wuji and right Wuji identities')
    for device in devices:
        fields = ['model_variant', 'hardware_revision', 'firmware', 'serial_or_asset_id', 'identity_evidence']
        missing = [field for field in fields if not known(device.get(field))]
        if missing or device.get('identity_status') != 'confirmed':
            reasons.append(f"{device.get('device_id')}: identity unconfirmed ({', '.join(missing)})")
    mapping_fields = ['hardware_joint_name', 'hardware_sdk_axis', 'hardware_device_id',
                      'hardware_direction_sign', 'hardware_zero_offset_rad',
                      'hardware_position_unit', 'hardware_evidence_source']
    limit_fields = ['hardware_position_min_rad', 'hardware_position_max_rad',
                    'hardware_speed_limit_rad_s', 'hardware_acceleration_limit_rad_s2',
                    'hardware_current_limit_A', 'hardware_torque_limit_Nm',
                    'hardware_temperature_limit_C', 'hardware_duty_cycle_constraint',
                    'hardware_limit_evidence_revision']
    for label, rows, fields, status in [('mapping', mapping, mapping_fields, 'mapping_status'),
                                       ('limits', limits, limit_fields, 'hardware_limit_status')]:
        incomplete = [row['model_joint'] for row in rows
                      if row.get(status) != 'confirmed' or any(not known(row.get(field)) for field in fields)]
        if incomplete:
            reasons.append(f'{label}: {len(incomplete)} joints lack confirmed evidence')
    body_rows = [row for row in mapping if not row['model_joint'].startswith(('l_', 'r_'))]
    hand_rows = [row for row in mapping if row['model_joint'].startswith(('l_', 'r_'))]
    try:
        body_indices = sorted(int(row['bag_raw_index']) for row in body_rows)
        hand_indices = sorted(int(row['bag_raw_index']) for row in hand_rows)
        nested_ok = all(int(row['bag_raw_index']) == 20 * int(row['bag_hand_index']) + int(row['bag_pos_index'])
                        and int(row['bag_hand_index']) == (0 if row['model_joint'].startswith('l_') else 1)
                        and 0 <= int(row['bag_pos_index']) < 20 for row in hand_rows)
        if body_indices != list(range(16)) or hand_indices != list(range(40)) or not nested_ok:
            reasons.append('bag channel indices must cover 16 body and 40 nested hand positions exactly')
    except (KeyError, TypeError, ValueError):
        reasons.append('invalid bag channel indices')
    device_ids = {device.get('device_id') for device in devices}
    for row in mapping:
        if row['mapping_status'] != 'confirmed':
            continue
        valid = (row.get('hardware_device_id') in device_ids
                 and row.get('hardware_device_id') == ('wuji_left' if row['model_joint'].startswith('l_') else 'wuji_right' if row['model_joint'].startswith('r_') else 'tron2')
                 and finite(row.get('hardware_direction_sign'))
                 and float(row['hardware_direction_sign']) in {-1., 1.}
                 and finite(row.get('hardware_zero_offset_rad'))
                 and row.get('hardware_position_unit') == 'rad')
        if not valid:
            reasons.append(f"{row['model_joint']}: invalid SDK device/direction/zero/unit")
    confirmed = [row for row in mapping if row['mapping_status'] == 'confirmed']
    axes = [(row.get('hardware_device_id'), row.get('hardware_sdk_axis')) for row in confirmed]
    if len(axes) != len(set(axes)):
        reasons.append('SDK axis assignments repeat within a device')
    for row in limits:
        if row['hardware_limit_status'] != 'confirmed':
            continue
        numeric = limit_fields[:7] + ['hardware_temperature_limit_C']
        valid = all(finite(row.get(field)) for field in numeric)
        if valid:
            valid = (float(row['hardware_position_min_rad']) < float(row['hardware_position_max_rad'])
                     and all(float(row[field]) > 0 for field in numeric[2:]))
        if not valid:
            reasons.append(f"{row['model_joint']}: invalid physical numeric limits")
    # Offline completeness covers inventory and trajectory evidence only.
    # Live commissioning is not evaluated here and can never be authorized here.
    record = inventory.get('trajectory_admission', {})
    if record.get('status') != 'passed' or not record.get('trajectory_sha256') or not record.get('report_sha256'):
        reasons.append('revision-bound hardware trajectory admission report missing')
    return {'offline_evidence_complete': not reasons, 'hardware_execution_authorized': False, 'live_commissioning_verified': False,
            'reasons': reasons, 'boundary': 'Offline evidence completeness only; no SDK or hardware I/O; not a player or safety certificate'}


def main():
    report = evaluate(Path(__file__).resolve().parents[1])
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report['offline_evidence_complete'] else 2)


if __name__ == '__main__':
    main()
