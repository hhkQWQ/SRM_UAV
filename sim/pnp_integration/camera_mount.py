"""Build and verify the X500 camera mount from the original OpenMV extrinsics."""
import argparse
import math
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET


FRD_TO_FLU = ((1., 0., 0.), (0., -1., 0.), (0., 0., -1.))
OPTICAL_TO_SENSOR = ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.))
BASE_IN_MODEL = (0., 0., .24)


def transpose(matrix):
    return tuple(zip(*matrix))


def mv(matrix, vector):
    return tuple(sum(a*b for a, b in zip(row, vector)) for row in matrix)


def mm(first, second):
    return tuple(tuple(sum(a*b for a, b in zip(row, col))
                       for col in transpose(second)) for row in first)


def close(first, second):
    return len(first) == len(second) and all(abs(a-b) < 1e-7 for a, b in zip(first, second))


def rotation_rpy(roll, pitch, yaw):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def rpy_rotation(rotation):
    pitch = math.asin(max(-1., min(1., -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return roll, pitch, yaw


def pose(element):
    if element is None:
        return (0.,)*6
    if element.get('rotation_format', 'euler_rpy') != 'euler_rpy' or element.get('degrees', 'false') != 'false':
        raise ValueError('Camera mount requires radian Euler SDF poses')
    result = tuple(float(x) for x in (element.text or '0 0 0 0 0 0').split())
    if len(result) != 6 or not all(math.isfinite(x) for x in result):
        raise ValueError('Invalid model pose')
    return result


def validate_extrinsics(rotation, translation):
    rotation = tuple(tuple(float(x) for x in row) for row in rotation)
    translation = tuple(float(x) for x in translation)
    if (len(rotation) != 3 or any(len(row) != 3 for row in rotation)
            or len(translation) != 3
            or not all(math.isfinite(x) for row in rotation for x in row)
            or not all(math.isfinite(x) for x in translation)):
        raise ValueError('Invalid original camera extrinsics')
    identity = mm(rotation, transpose(rotation))
    expected = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    determinant = sum(rotation[0][i]*(rotation[1][(i+1)%3]*rotation[2][(i+2)%3]
                                   -rotation[1][(i+2)%3]*rotation[2][(i+1)%3]) for i in range(3))
    if not all(close(a, b) for a, b in zip(identity, expected)) or abs(determinant-1.) > 1e-7:
        raise ValueError('Original R_BC is not a proper rotation')
    return rotation, translation


def model_elements(px4, model_path=None):
    models = Path(px4)/'Tools/simulation/gz/models'
    tree = ET.parse(model_path or models/'x500_mono_cam_down/model.sdf')
    top = tree.find('model')
    matches = [x for x in top.findall('include') if x.findtext('uri') == 'model://mono_cam']
    if len(matches) != 1:
        raise ValueError('Expected one merged mono_cam include')
    include = matches[0]
    if include.get('merge') != 'true':
        raise ValueError('Camera include must merge into the X500 model')
    base = ET.parse(models/'x500_base/model.sdf').find('model')
    if not close(pose(base.find('pose')), BASE_IN_MODEL+(0., 0., 0.)):
        raise ValueError('X500 base reference changed; update camera mount geometry')
    imager = ET.parse(models/'mono_cam/model.sdf').find('model')
    for element in (imager.find('pose'), imager.find('./link/pose'), imager.find('./link/sensor/pose')):
        if not close(pose(element), (0.,)*6):
            raise ValueError('mono_cam has an additional local pose; update camera mount geometry')
    joint = top.find("joint[@name='CameraJoint']")
    if (joint is None or joint.get('type') != 'fixed'
            or joint.findtext('parent') != 'base_link' or joint.findtext('child') != 'camera_link'):
        raise ValueError('Expected a fixed base_link to camera_link joint')
    return tree, include, joint


def read_camera_mount(px4, rotation, translation, model_path=None):
    """Verify actual SDF placement against R_BC/T_BC; return physical model pose."""
    rotation, translation = validate_extrinsics(rotation, translation)
    _, include, joint = model_elements(px4, model_path)
    placement = include.find('pose')
    if placement is not None and placement.get('relative_to', '') not in ('', '__model__'):
        raise ValueError('Camera include pose must be expressed in the model frame')
    camera = pose(placement)
    camera_rotation = rotation_rpy(*camera[3:])
    actual_rotation = mm(mm(FRD_TO_FLU, camera_rotation), OPTICAL_TO_SENSOR)
    actual_translation = mv(FRD_TO_FLU, tuple(camera[i]-BASE_IN_MODEL[i] for i in range(3)))
    if (not all(close(a, b) for a, b in zip(actual_rotation, rotation))
            or not close(actual_translation, translation)):
        raise ValueError('Gazebo camera mount does not match original R_BC/T_BC; restart with start_simulation.sh')
    anchor = joint.find('pose')
    expected_anchor = tuple(camera[i]-BASE_IN_MODEL[i] for i in range(3))
    if (anchor is None or anchor.get('relative_to') != 'base_link'
            or not close(pose(anchor)[:3], expected_anchor)
            or not all(close(a, b) for a, b in zip(rotation_rpy(*pose(anchor)[3:]), camera_rotation))):
        raise ValueError('Camera fixed-joint pose differs from the calibrated mount')
    return camera[:3], camera_rotation


def build_camera_mount(px4, rotation, translation, output, model_path=None):
    rotation, translation = validate_extrinsics(rotation, translation)
    tree, include, joint = model_elements(px4, model_path)
    camera_rotation = mm(mm(FRD_TO_FLU, rotation), transpose(OPTICAL_TO_SENSOR))
    camera_offset = mv(FRD_TO_FLU, translation)
    camera_position = tuple(BASE_IN_MODEL[i]+camera_offset[i] for i in range(3))
    angles = rpy_rotation(camera_rotation)
    for parent, values, reference in ((include, camera_position+angles, None),
                                      (joint, camera_offset+angles, 'base_link')):
        element = parent.find('pose')
        if element is None:
            element = ET.SubElement(parent, 'pose')
        element.attrib.clear()
        if reference:
            element.set('relative_to', reference)
        element.text = ' '.join(format(0. if abs(x) < 1e-15 else x, '.17g') for x in values)
    ET.indent(tree, space='  ')
    data = ET.tostring(tree.getroot(), encoding='utf-8', xml_declaration=True)+b'\n'
    destination = Path(output)
    if not destination.exists() or destination.read_bytes() != data:
        temporary = destination.with_name(destination.name+'.mount-tmp')
        temporary.write_bytes(data)
        temporary.chmod(0o644)
        temporary.replace(destination)
    return read_camera_mount(px4, rotation, translation, destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', default='/home/hhkqwq/PX4-Autopilot')
    parser.add_argument('--algorithm-dir', default=str(Path(__file__).resolve().parents[2]/'PnP'))
    parser.add_argument('--output')
    options, _ = parser.parse_known_args()
    config = runpy.run_path(str(Path(options.algorithm_dir)/'config.py'))
    if not config.get('EXTRINSICS_CONFIRMED'):
        raise ValueError('Original OpenMV extrinsics are not confirmed')
    output = options.output or str(Path(options.px4_dir)/'Tools/simulation/gz/models/x500_mono_cam_down/model.sdf')
    build_camera_mount(options.px4_dir, config['R_BC'], config['T_BC_M'], output)
    print(f'Camera mount matches original OpenMV configuration: T_BC_M={config["T_BC_M"]}; R_BC={config["R_BC"]}')


if __name__ == '__main__':
    main()
