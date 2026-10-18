"""Extract block settings and connection groups without running MATLAB."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'models/Mulag_FME_500_with_cylinder.slx'
SYSTEMS = ['743', '162', '178', '188', '196', '1103', '609', '616', '602', '1107']
FIELDS = [
    'displacement', 'omega_nominal', 'constant', 'p_set_differential', 'p_range',
    'capacity', 'dead_volume', 'p_precharge', 'k_sh', 'rho_L_atm', 'beta_L_atm',
    'nu_atm', 'air_fraction', 'bulk_modulus_model', 'pressure_spec',
    'reservoir_pressure', 'Unit', 'FilteringAndDerivatives', 'UdotUserProvided',
    'InputFilterTimeConstant', 'piston_area_A', 'piston_area_B', 'stroke',
    'dead_volume_A', 'dead_volume_B', 'p0_A', 'p0_B', 'x0', 'displacement_spec',
    'dynamic_compressibility', 'friction_spec', 'leakage_spec',
    'cushion_A_spec', 'cushion_B_spec', 'hardstop_model', 'stiff_coeff',
    'damping_coeff', 'area_spec', 'valve_spec', 'area_max', 'area_leak',
    'del_S_max', 'Cd', 'Re_c', 'smoothing_factor', 'opening_dynamics',
    'S_max_PA', 'S_max_PB', 'S_max_AT', 'S_max_BT',
    'open_orifices_pos', 'open_orifices_neu', 'open_orifices_neg',
    'brkwy_frc', 'brkwy_vel', 'Col_frc', 'visc_coef',
    'TorqueActuationMode', 'MotionActuationMode', 'LowerLimitSpecify',
    'UpperLimitSpecify', 'LowerLimitBound', 'UpperLimitBound',
    'DampingCoefficient', 'GravityVector', 'Port', 'Side',
]


def extract():
    result = {'model': str(MODEL.relative_to(ROOT)),
              'sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest(),
              'method': 'Static XML inspection; no compiled or executed model.',
              'systems': {}}
    with zipfile.ZipFile(MODEL) as archive:
        for system in SYSTEMS:
            member = f'simulink/systems/system_{system}.xml'
            node = ET.fromstring(archive.read(member))
            blocks = []
            for block in node.findall('Block'):
                props = {p.get('Name'): p.text for p in block.findall('P')}
                instance = block.find('InstanceData')
                if instance is not None:
                    props.update({p.get('Name'): p.text for p in instance.findall('P')})
                if not (props.get('SourceBlock') or block.get('BlockType') == 'PMIOPort'):
                    continue
                parameters = {}
                for name in FIELDS:
                    if name not in props:
                        continue
                    parameters[name] = {'value': props[name]}
                    for suffix in ['_unit', 'Units']:
                        if name+suffix in props:
                            parameters[name]['unit'] = props[name+suffix]
                blocks.append({'sid': block.get('SID'), 'name': block.get('Name'),
                               'source': props.get('SourceBlock'),
                               'commented': props.get('Commented', 'off'),
                               'parameters': parameters})
            lines = [{'name': line.findtext("P[@Name='Name']"),
                      'endpoints': [p.text for p in line.iter('P')
                                    if p.get('Name') in ['Src', 'Dst']]}
                     for line in node.findall('Line')]
            result['systems'][member] = {'blocks': blocks, 'connection_groups': lines}
    return result


if __name__ == '__main__':
    output = ROOT/'docs/model-specification.json'
    output.write_text(json.dumps(extract(), indent=2)+'\n')
    print(output)
