#!/usr/bin/env python3
"""Force & reaction-torque symmetry verification of the x500 propeller models.

The concern this answers (raised after the ArduPilot mixer kept allocating
unequal CW/CCW pwm): are the CW and CCW propeller models truly mirror images,
i.e. do they produce identical force and reaction torque at identical |omega|?

Measurement trick: spawn the x500 on a frictionless VIRTUAL RAIL
(prismatic-z + revolute-yaw world joints, gravity off). The rail removes all
ground-contact masking, so
  * IMU az is a direct force gauge: force = mass * az,
  * yaw angular acceleration is a direct reaction-torque gauge:
    torque = Izz * d(yaw_rate)/dt.

Checks (fake servo packets drive the ArduPilotPlugin, no SITL needed):
  1. all-four symmetric pwm -> total force matches LiftDrag theory
     (F_rotor = force_constant * omega^2, force_constant = 4.2e-5), and the
     net reaction torque cancels (yaw slope ~ 0).
  2. per-rotor: all four rotors produce equal force (catches a mis-mirrored
     CW/CCW blade), and CW vs CCW reaction torques are EQUAL in magnitude and
     OPPOSITE in sign (momentum-to-rpm symmetry).
  3. CCW pair vs CW pair: equal pair force; opposite, equal-magnitude
     pair reaction torques.

Self-sourced, headless, cleans up gz/ROS processes itself.

Usage: ./check_force_symmetry.py
Exit 0 = all checks passed.
"""

import json
import os
import re
import sys
import time
import xml.dom.minidom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RUN_DIR = os.path.join(SCRIPT_DIR, '.run')
sys.path.insert(0, SCRIPT_DIR)

import check_motor_physics as phys  # noqa: E402

FDM_PORT = 9002
PHYS_STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.005
RHO = 1.2041
CP = 2.0 / 3.0 * 0.1751           # blade center of pressure radius
AREA = 0.002                      # per-blade LiftDrag area (calibrated)
CL = 4.25 * 0.3                   # cla * a0 in the hover alpha regime
CD = 0.10 * 0.3                   # cda * alpha
FORCE_CONSTANT = 4.2e-5           # N/(rad/s)^2 per rotor incl. 2 blades
MASS = phys.DRONE_MASS

# whole-vehicle inertia about z [kg m^2]: body disc (radius 0.25) + prop
# spin inertia + prop offset inertia. Only used for torque-vs-theory, the
# CW/CCW equality checks do not depend on it.
IZZ_TOTAL = 0.0659

failures = []
records = {}


def check(cond, msg, val=None):
    tag = 'PASS' if cond else 'FAIL'
    print(f'  [{tag}] {msg}' + (f'  (got {val})' if val is not None else ''))
    if not cond:
        failures.append(msg)


def rotor_force_theory(omega):
    return FORCE_CONSTANT * omega * omega


def rotor_torque_theory(omega):
    # 2 blades * cp * drag; drag/lift ratio per blade = cd/cl (alpha cancels)
    force = rotor_force_theory(omega)
    return CP * CD / CL * force


# ---------------------------------------------------------------------------

RAIL_XML = """
  <link name='rail_link'>
    <inertial>
      <mass>0.000001</mass>
      <inertia>
        <ixx>1e-9</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>1e-9</iyy><iyz>0</iyz><izz>1e-9</izz>
      </inertia>
    </inertial>
    <visual name='rail_vis'>
      <geometry><box><size>0.01 0.01 0.01</size></box></geometry>
    </visual>
  </link>
  <joint name='rail_prismatic' type='prismatic'>
    <parent>world</parent>
    <child>rail_link</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
      <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
    </axis>
  </joint>
  <joint name='rail_revolute' type='revolute'>
    <parent>rail_link</parent>
    <child>base_link</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
      <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
    </axis>
  </joint>
"""


def render_model_xml():
    rendered = phys.render_x500({
        'name': phys.UAV_NAME, 'model': 'x500', 'use-ardupilot': {},
        'ardupilot_config': {'fdm_port_in': FDM_PORT},
    })
    doc = xml.dom.minidom.parseString(rendered)
    # inject the rail link/joints INTO the uav model (intra-model scoping);
    # cross-model joint references are not resolvable in gz-sim8 world files
    for extra in xml.dom.minidom.parseString(f'<wrapper>{RAIL_XML}</wrapper>').documentElement.childNodes:
        doc.getElementsByTagName('model')[0].appendChild(doc.importNode(extra, True))
    return doc.getElementsByTagName('model')[0].toxml()


def build_rail_world():
    '''x500 constrained to vertical translation + yaw on a frictionless rail,
    gravity off -> az = F/m and yaw angular acceleration = torque/Izz.'''
    model_xml = render_model_xml()
    return f"""<?xml version='1.0'?>
<sdf version='1.10'>
  <world name='default'>
    <gravity>0 0 0</gravity>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.005</max_step_size>
      <real_time_update_rate>200</real_time_update_rate>
    </physics>
    {model_xml}
  </world>
</sdf>""".replace('0.005', str(PHYS_STEP)).replace('200</real_time_update_rate>', f'{1.0 / PHYS_STEP}></real_time_update_rate>')


V3 = r'\{\s*x: ([-0-9.eE+]+)\s+y: ([-0-9.eE+]+)\s+z: ([-0-9.eE+]+)\s*\}'


def parse_imu_series(raw):
    '''Parse (sec, az, wz) triples in chronological order from topic echo output.'''
    series = []
    # each message: header stamp ... linear_acceleration {...} and angular_velocity {...}
    for chunk in re.split(r'\ndata: true\n|\nheader \{', raw)[1:]:
        sm = re.search(r'stamp \{\s*sec: ([-0-9]+)\s+nsec: ([0-9]+)', chunk)
        am = re.search(r'linear_acceleration ' + V3, chunk)
        wm = re.search(r'angular_velocity ' + V3, chunk)
        if sm and am and wm:
            sec = int(sm.group(1)) + int(sm.group(2)) / 1e9
            series.append((sec, float(am.group(3)), float(wm.group(3))))
    return series


def imu_stats(raw):
    '''Return mean az and yaw-slope (rad/s^2) over the series.'''
    series = parse_imu_series(raw)
    if len(series) < 10:
        return None, None
    n = len(series)
    t0 = series[0][0]
    ts = [s[0] - t0 for s in series]
    azs = [s[1] for s in series]
    wzs = [s[2] for s in series]
    mean_az = sum(azs) / n
    # least squares slope of wz vs t
    mt, mw = sum(ts) / n, sum(wzs) / n
    num = sum((t - mt) * (w - mw) for t, w in zip(ts, wzs))
    den = sum((t - mt) ** 2 for t in ts)
    slope = num / den if den > 1e-9 else 0.0
    return mean_az, slope


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    env = phys.sourced_env()
    # a static world load resolves model:// URIs at parse time, unlike the
    # spawn service - put the mrs robotics models on the gz resource path
    models_parent = __import__('subprocess').check_output(
        ['bash', '-c',
         'source /home/nekovfra/workspaces/claude_ws/install/setup.bash >/dev/null 2>&1; '
         'echo $(ros2 pkg prefix mrs_uav_gazebo_simulator)'
         '/share/mrs_uav_gazebo_simulator/models'],
        text=True).strip()
    env['GZ_SIM_RESOURCE_PATH'] = models_parent + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')

    world_path = os.path.join(RUN_DIR, 'rail_world.sdf')
    with open(world_path, 'w') as f:
        f.write(build_rail_world())

    print('=== startup: gz rail-world (no gravity, frictionless yaw/vertical) ===')
    phys.cleanup_leftovers()
    time.sleep(2)

    gz_proc = None
    driver = phys.ServoDriver(port=FDM_PORT)
    try:
        gz_proc = phys.start_gz(env, world=world_path)
        # model is part of the world; wait for its topics
        phys.spawn_uav.__wrapped__ if False else None
        for _ in range(60):
            try:
                topics = __import__('subprocess').check_output(
                    ['gz', 'topic', '-l'], env=env, text=True,
                    stderr=__import__('subprocess').DEVNULL)
                if phys.JOINT_STATE_TOPIC in topics:
                    break
            except Exception:
                pass
            time.sleep(0.5)
        __import__('time').sleep(5)  # let plugin + imu settle
        print('  [info] model present in rail world')

        driver.start()

        def stats_for(seconds):
            raw = phys.sample_gz(phys.IMU_TOPIC, seconds, env)
            return imu_stats(raw)

        # ---- phase 0: baseline (no thrust, free fall -> az ~ 0) ----
        print('Phase 0: idle baseline')
        driver.set_all(phys.SERVO_MIN)
        time.sleep(2)
        om = phys.measure(1.0, env)
        az0, s0 = stats_for(2)
        check(az0 is not None, 'IMU series readable on rail')
        if az0 is not None:
            check(abs(az0) < 0.05, 'idle on rail: az ~ 0 (free float, gravity off)', az0)

        # ---- phase 1: all-four symmetric, force + torque-cancellation ----
        pwm_mid = 1600
        print(f'Phase 1: symmetric all-four (pwm = {pwm_mid})')
        driver.set_all(pwm_mid)
        time.sleep(3)
        om = phys.measure(1.5, env)
        az, slope = stats_for(3)
        driver.set_all(phys.SERVO_MIN)
        oms = [abs(om['omega'][f'prop_{k}_joint']) for k in range(4)]
        om_mean = sum(oms) / 4
        f_meas = MASS * az
        f_theory = 4 * rotor_force_theory(om_mean)
        records['symmetric'] = dict(omegas=om['omega'], az=az, yaw_slope=slope,
                                    force_meas=f_meas, force_theory=f_theory)
        print(f'  om={oms} az={az:.2f} slope={slope:.4f}')
        check(abs(f_meas - f_theory) < 0.15 * f_theory,
              'symmetric: total force matches theory F=4*4.2e-5*om^2 +/-15%',
              f'{f_meas:.1f}N vs {f_theory:.1f}N')
        check(abs(slope) < 0.08,
              'symmetric: net reaction torque ~ 0 (CW/CCW cancel)', slope)

        # ---- phase 2: per-rotor force & reaction-torque equality ----
        pwm_one = 1550
        print(f'Phase 2: per-rotor singles (pwm = {pwm_one} on one channel)')
        forces, torques, signs = [], [], []
        expected_sign = [ -1, -1, 1, 1]  # CCW props react -z, CW react +z
        for k in range(4):
            time.sleep(1.5)
            driver.set_pwm(k, pwm_one)
            time.sleep(2.5)
            om = phys.measure(1.0, env)
            az, slope = stats_for(3)
            driver.set_pwm(k, phys.SERVO_MIN)
            o = om['omega'][f'prop_{k}_joint']
            f_k = MASS * az
            tau_k = IZZ_TOTAL * slope
            forces.append(f_k)
            torques.append(tau_k)
            signs.append(1 if slope > 0 else -1)
            records[f'single_{k}'] = dict(omega=o, az=az, force=f_k,
                                          yaw_slope=slope, torque=tau_k)
            print(f'  prop_{k}: om={o:.0f} az={az:.2f} F={f_k:.2f}N '
                  f'yaw_slope={slope:.4f} tau={tau_k:.4f}Nm')

        f_mean = sum(forces) / 4
        check(all(abs(f - f_mean) < 0.12 * f_mean for f in forces),
              'per-rotor: all four rotors produce equal force +/-12% '
              '(no mis-mirrored CW/CCW blade)', [round(f, 2) for f in forces])
        t_mean = sum(abs(t) for t in torques) / 4
        check(all(abs(abs(t) - t_mean) < 0.25 * t_mean for t in torques),
              'per-rotor: CW vs CCW reaction torque EQUAL magnitude +/-25%',
              [round(t, 4) for t in torques])
        check(signs == expected_sign,
              'per-rotor: reaction torque sign matches spin direction '
              '(CCW->-z, CW->+z)', signs)

        # ---- phase 3: CCW pair vs CW pair ----
        pwm_pair = 1550
        print(f'Phase 3: pair comparison (CCW pair, then CW pair, pwm = {pwm_pair})')
        pair = {}
        for name, chs in (('ccw_pair', (0, 1)), ('cw_pair', (2, 3))):
            for c in chs:
                driver.set_pwm(c, pwm_pair)
            time.sleep(2.5)
            om = phys.measure(1.0, env)
            az, slope = stats_for(3)
            driver.set_all(phys.SERVO_MIN)
            pair[name] = dict(az=az, force=MASS * az, yaw_slope=slope,
                              torque=IZZ_TOTAL * slope)
            records[name] = pair[name]
            print(f'  {name}: az={az:.2f} F={MASS*az:.2f}N slope={slope:.4f}')
        check(abs(pair['ccw_pair']['force'] - pair['cw_pair']['force'])
              < 0.1 * pair['ccw_pair']['force'],
              'pairs: CCW and CW pairs produce equal force +/-10%',
              (round(pair['ccw_pair']['force'], 2), round(pair['cw_pair']['force'], 2)))
        check(abs(abs(pair['ccw_pair']['torque']) - abs(pair['cw_pair']['torque']))
              < 0.2 * abs(pair['ccw_pair']['torque']),
              'pairs: CCW and CW reaction torques equal magnitude +/-20%, opposite sign',
              (round(pair['ccw_pair']['torque'], 4), round(pair['cw_pair']['torque'], 4)))
        check(pair['ccw_pair']['torque'] * pair['cw_pair']['torque'] < 0,
              'pairs: reaction torques opposite sign')

    finally:
        driver.stop()
        if gz_proc and gz_proc.poll() is None:
            gz_proc.terminate()
        phys.cleanup_leftovers()
        print('=== cleanup done ===')

    out = os.path.join(RUN_DIR, f'force_symmetry_{time.strftime("%Y%m%d_%H%M%S")}.json')
    with open(out, 'w') as f:
        json.dump(records, f, indent=2, default=str)
    print(f'\nrecords -> {out}')
    if failures:
        print(f'\n{len(failures)} check(s) FAILED')
        for f_ in failures:
            print('  -', f_)
        return 1
    print('\nAll force symmetry checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
