"""H1 standalone scenes: explicit ground and 19 actuators, with mesh-foot support."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from h1_deploy.robots.h1 import H1_XML, H1_JOINT_NAMES, stiffness, damping, effort_limit


def scene_xml(torque=False):
    root = ET.parse(H1_XML).getroot()
    compiler = root.find('compiler')
    compiler.set('meshdir', str(H1_XML.parent / compiler.get('meshdir', '')))
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('timestep', '0.001' if torque else '0.005')
    option.set('integrator', 'implicitfast')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'headlight', active='1', diffuse='0.6 0.6 0.6',
                  ambient='0.3 0.3 0.3', specular='0 0 0')
    # Match the blue checkerboard ground in h1_data/scene.xml.
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', type='skybox', name='skybox', builtin='flat',
                  rgb1='0.3 0.5 0.7', rgb2='0.3 0.5 0.7', width='512', height='3072')
    ET.SubElement(asset, 'texture', type='2d', name='groundplane', builtin='checker', mark='edge',
                  rgb1='0.2 0.3 0.4', rgb2='0.1 0.2 0.3', markrgb='0.8 0.8 0.8', width='300', height='300')
    ET.SubElement(asset, 'material', name='groundplane', texture='groundplane', texuniform='true',
                  texrepeat='5 5', reflectance='0.2')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', name='scene_light', pos='0 0 3', dir='0 0 -1',
                  directional='true', active='true', castshadow='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='0 0 0.05',
                  material='groundplane', friction='1 0.005 0.0001', condim='3')
    old = root.find('actuator')
    if old is not None:
        root.remove(old)
    actuators = ET.SubElement(root, 'actuator')
    for name, kp, kd, effort in zip(H1_JOINT_NAMES, stiffness, damping, effort_limit):
        attrs = dict(name=name + '_actuator', joint=name, forcelimited='true', forcerange=f'{-effort} {effort}')
        if torque:
            ET.SubElement(actuators, 'motor', **attrs, ctrllimited='true', ctrlrange=f'{-effort} {effort}')
        else:
            ET.SubElement(actuators, 'position', **attrs, kp=str(kp), kv=str(kd), ctrllimited='false')
    # The source MJCF comments out its sensors, but the DDS simulator publishes
    # the torso IMU. The controller then converts that measurement to the pelvis.
    sensors = root.find('sensor')
    if sensors is None:
        sensors = ET.SubElement(root, 'sensor')
    names = {sensor.get('name') for sensor in sensors}
    for kind, name, attrs in (
        ('framequat', 'imu_quat', dict(objtype='site', objname='imu')),
        ('gyro', 'imu_gyro', dict(site='imu')),
        ('accelerometer', 'imu_acc', dict(site='imu')),
    ):
        if name not in names:
            ET.SubElement(sensors, kind, name=name, **attrs)
    return ET.tostring(root, encoding='unicode')


def load_scene(path=None, lock=False, torque=False):
    path = Path(path) if path else None
    if path and path.is_file():
        spec = mujoco.MjSpec.from_file(str(path))
        if not list(spec.actuators):
            raise ValueError(f'{path}: scene has no actuators; use the built-in H1 scene or a complete exported scene')
    else:
        # Only the standard optional export paths fall back to the bundled H1 scene.
        if path and path.name not in ('h1_train_scene.xml', 'h1_torque_scene.xml'):
            raise FileNotFoundError(path)
        spec = mujoco.MjSpec.from_string(scene_xml(torque))
    if lock:
        for actuator in list(spec.actuators):
            if (actuator.target or '').split('/')[-1] == 'torso_joint':
                spec.delete(actuator)
        for key in list(spec.keys):
            spec.delete(key)
        for sensor in list(spec.sensors):
            if (sensor.objname or '').split('/')[-1] == 'torso_joint':
                spec.delete(sensor)
        for joint in list(spec.joints):
            if (joint.name or '').split('/')[-1] == 'torso_joint':
                spec.delete(joint)
    return spec.compile()


def foot_height(model, data):
    heights = []
    for g in range(model.ngeom):
        body = (model.body(model.geom_bodyid[g]).name or '').split('/')[-1]
        if body not in ('left_ankle_link', 'right_ankle_link') or not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            first, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[first:first + count]
            heights.append(float(np.min(vertices @ data.geom_xmat[g].reshape(3, 3)[2] + data.geom_xpos[g, 2])))
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
            heights.append(float(data.geom_xpos[g, 2] - model.geom_size[g, 0]))
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            heights.append(float(data.geom_xpos[g, 2] - abs(data.geom_xmat[g].reshape(3, 3)[2, 2]) * model.geom_size[g, 1] - model.geom_size[g, 0]))
    if not heights:
        raise ValueError('H1 scene has no supported foot collision geometry')
    return min(heights)
