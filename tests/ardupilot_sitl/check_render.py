#!/usr/bin/env python3
"""Offline render validation of the x500 SDF templates for the ArduPilot backend.

Renders the x500 jinja template exactly like the drone spawner's
JinjaTemplateManager does (same jinja2 environment, same spawner_args shape),
both with and without the --use-ardupilot keyword, and asserts the structure
of the produced SDF. Runs without ROS, Gazebo or SITL - this is the first
gate before touching the simulator.

Usage:
    ./check_render.py            # source the workspace first (ament_index not needed)

Exit code 0 = all checks passed.
"""

import math
import os
import sys
import xml.dom.minidom

import jinja2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
MODELS_PATH = os.path.join(PACKAGE_DIR, 'models')

PROP_RADIUS = 0.1751
FORCE_CONSTANT = 0.000042
MOMENT_CONSTANT = 0.06
SERVO_MIN, SERVO_MAX = 1100, 1900
MAX_ROT_VELOCITY = 780
P_GAIN = 0.02

failures = []


def check(condition, message):
    status = 'PASS' if condition else 'FAIL'
    print(f'  [{status}] {message}')
    if not condition:
        failures.append(message)


def render_x500(spawner_args):
    '''Render the x500 template with the same jinja env as JinjaTemplateManager.'''
    env = jinja2.Environment(loader=jinja2.FileSystemLoader([MODELS_PATH]), autoescape=False)
    env.globals['math'] = math
    template = env.get_template('mrs_robots_description/sdf/drones/x500.sdf.jinja')
    return template.render({'spawner_args': spawner_args})


def plugins_of(root, filename=None):
    found = []
    for plugin in root.getElementsByTagName('plugin'):
        if filename is None or plugin.getAttribute('filename') == filename:
            found.append(plugin)
    return found


def child_text(elem, tag):
    nodes = elem.getElementsByTagName(tag)
    return nodes[0].firstChild.nodeValue.strip() if nodes and nodes[0].firstChild else None


def check_px4_variant():
    print('Variant: default (PX4 motor model)')
    rendered = render_x500({
        'name': 'uav1',
        'model': 'x500',
    })
    root = xml.dom.minidom.parseString(rendered)

    check(len(plugins_of(root, 'MrsGazeboCommonResources_MulticopterMotorModel')) == 4,
          'x500 PX4 variant has 4 MrsMulticopterMotorModel plugins')
    check(len(plugins_of(root, 'ArduPilotPlugin')) == 0, 'PX4 variant contains no ArduPilotPlugin')
    check(len(plugins_of(root, 'gz-sim-lift-drag-system')) == 0,
          'PX4 variant contains no LiftDrag plugins (motor model handles aero)')


def check_ardupilot_variant():
    print('Variant: --use-ardupilot')
    rendered = render_x500({
        'name': 'uav1',
        'model': 'x500',
        'use-ardupilot': {},
        'ardupilot_config': {
            'fdm_port_in': 9002,
            'gcs_tcp_port': 5760,
            'qgc_udp_port': 14560,
            'sysid': 1,
            'home': '47.397743,8.545594,340,0',
            'fcu_url': 'tcp://127.0.0.1:5760@',
        },
    })

    # dump the rendered SDF next to this script for manual inspection
    dump_path = os.path.join(SCRIPT_DIR, 'last_rendered_x500_ardupilot.sdf')
    with open(dump_path, 'w') as f:
        f.write(rendered)

    root = xml.dom.minidom.parseString(rendered)

    # the PX4 motor model must be fully replaced by the ArduPilot pipeline
    check(len(plugins_of(root, 'MrsGazeboCommonResources_MulticopterMotorModel')) == 0,
          'x500 ArduPilot variant has NO MrsMulticopterMotorModel plugins')

    # ArduPilotPlugin block
    ap_plugins = plugins_of(root, 'ArduPilotPlugin')
    check(len(ap_plugins) == 1, 'exactly one ArduPilotPlugin')
    ap = ap_plugins[0]
    check(child_text(ap, 'fdm_port_in') == '9002',
          f'fdm_port_in matches SITL JSON backend port 9002+10*ID (got {child_text(ap, "fdm_port_in")})')
    check(child_text(ap, 'imuName') == 'base_link::imu_sensor',
          f'imuName is model-relative (got {child_text(ap, "imuName")})')
    check(child_text(ap, 'lock_step') == '0', 'lock_step is 0 (robust SITL/gazebo startup)')

    # control channels
    controls = ap.getElementsByTagName('control')
    check(len(controls) == 4, f'4 control channels (got {len(controls)})')
    expected_multipliers = [MAX_ROT_VELOCITY, MAX_ROT_VELOCITY, -MAX_ROT_VELOCITY, -MAX_ROT_VELOCITY]
    for i, control in enumerate(controls):
        check(control.getAttribute('channel') == str(i), f'control {i} uses servo channel {i}')
        check(child_text(control, 'jointName') == f'prop_{i}_joint',
              f'channel {i} drives prop_{i}_joint')
        check(float(child_text(control, 'multiplier')) == expected_multipliers[i],
              f'channel {i} multiplier is {expected_multipliers[i]} '
              f'(CW rotors spin with negative joint velocity)')
        check(float(child_text(control, 'servo_min')) == SERVO_MIN
              and float(child_text(control, 'servo_max')) == SERVO_MAX,
              f'channel {i} servo range {SERVO_MIN}..{SERVO_MAX} matches SITL MOT_PWM_MIN/MAX')
        check(float(child_text(control, 'p_gain')) == P_GAIN,
              f'channel {i} p_gain {P_GAIN} (discrete-time stable at 200 Hz physics)')

    # parametric propeller thrust plugins (replacing LiftDrag)
    props = plugins_of(root, 'ArduPilotPropeller')
    check(len(props) == 4, f'4 ArduPilotPropeller plugins, one per propeller (got {len(props)})')

    check(len(plugins_of(root, 'gz-sim-lift-drag-system')) == 0,
          'no LiftDrag plugins (unsuitable at 200 Hz physics rate)')
    check(len(plugins_of(root, 'gz-sim-apply-joint-force-system')) == 0,
          'no ApplyJointForce systems (unused by the ArduPilot backend)')

    expected_dirs = {0: 'ccw', 1: 'ccw', 2: 'cw', 3: 'cw'}
    by_joint = {child_text(p, 'jointName'): p for p in props}
    for motor in range(4):
        key = f'prop_{motor}_joint'
        p = by_joint.get(key)
        check(p is not None, f'{key}: ArduPilotPropeller plugin present')
        if p is None:
            continue
        check(child_text(p, 'linkName') == f'prop_{motor}_link',
              f'{key}: drives prop_{motor}_link')
        check(child_text(p, 'turningDirection') == expected_dirs[motor],
              f'{key}: turningDirection {expected_dirs[motor]}')
        check(abs(float(child_text(p, 'force_constant')) - FORCE_CONSTANT) < 1e-12,
              f'{key}: force_constant {FORCE_CONSTANT} (MRS force curve)')
        check(abs(float(child_text(p, 'moment_constant')) - MOMENT_CONSTANT) < 1e-12,
              f'{key}: moment_constant {MOMENT_CONSTANT} (MRS reaction torque)')

    # joints: damping/friction that settle the idle props without starving the PID
    joints = [j for j in root.getElementsByTagName('joint') if j.getAttribute('name').startswith('prop_')]
    check(len(joints) == 4, '4 propeller joints')
    for joint in joints:
        check(child_text(joint, 'damping') == '0.002' and child_text(joint, 'friction') == '0.001',
              f'{joint.getAttribute("name")}: damping 0.002 / friction 0.001')

    check(len(plugins_of(root, 'gz-sim-joint-state-publisher-system')) == 1,
          'JointStatePublisher present (prop velocity probing)')

    print(f'  rendered SDF dumped to {dump_path}')


def main():
    print('=== ArduPilot x500 template render checks ===')
    try:
        check_px4_variant()
        check_ardupilot_variant()
    except xml.dom.minidom.ExpatError as e:
        print(f'  [FAIL] rendered SDF is not well-formed XML: {e}')
        failures.append('XML well-formedness')
    except Exception as e:
        print(f'  [FAIL] rendering crashed: {e}')
        failures.append('rendering')

    print()
    if failures:
        print(f'{len(failures)} check(s) FAILED')
        return 1
    print('All render checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
