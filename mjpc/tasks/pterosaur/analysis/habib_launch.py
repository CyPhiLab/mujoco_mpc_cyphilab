#!/usr/bin/env python3
"""Quad launch calculator: Python port of Mike Habib's launch model.

Port of the 'Ballistic Model (4)' sheet of launch_version_4.0.xlsx (M.
Habib), a quadrupedal launch calculator for pterosaurs with soaring
estimators. The water launch sheet is not ported. Formulas are reproduced
as in the sheet, including its constants (g = 9.8 in most cells, 9.81 and
pi = 3.142 in the acceleration cells, air density 1.23 in some cells);
each quantity notes its source cell. test_habib_launch.py checks every
numeric cell against the sheet's cached values.

Model outline:
  - required ballistic speed: stall speed minus the airspeed the first
    (climb-out) wingbeat adds at quarter span, divided by cos(launch angle)
  - push-off: constant acceleration over 0.9 x shoulder/hip height; peak
    power = mass x speed x acceleration (Habib version), or split into
    horizontal and vertical terms (Palmer version)
  - available power: forelimb and hindlimb muscle mass x specific power,
    blending aerobic (150 W/kg) and anaerobic (390 W/kg) output
  - required preload = required / available power: the power amplification
    needed from elastic storage and counter-movement (2.5-3.5 in average
    leapers, up to ~7 in dedicated jumpers; Biewener 2003)

Usage:
  habib_launch.py [--mass 40 --height 1 --span 4.5 --aspect 14 ...]
"""
import argparse
import math
from dataclasses import dataclass, field

LAUNCH_ANGLES_DEG = [15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
PALMER_ANGLES_DEG = [15, 20, 25, 30, 35, 40, 45]


@dataclass
class LimbSystem:
  available: float = 1.0          # H: 1 if used for ground locomotion
  mass_fraction: float = 0.3      # I: proportion of body mass
  anaerobic_fraction: float = 0.9  # K: anaerobic proportion

  def muscle_mass(self, body_mass):   # J
    return self.mass_fraction * body_mass

  def power(self, body_mass):         # L: W
    k = self.anaerobic_fraction
    return (150 * (1 - k) + 390 * k) * self.muscle_mass(body_mass)


@dataclass
class Launcher:
  mass: float = 40.0              # B2: kg
  height: float = 1.0             # B3: shoulder/hip height, m
  span: float = 4.5               # B4: m
  aspect_ratio: float = 14.0      # B5
  cl: float = 1.5                 # B6: lift coefficient
  air_density: float = 1.22       # B16
  density_coefficient: float = 1.0  # B17 (not used by any formula)
  slotting_factor: float = 0.0    # B18
  flap_angle_deg: float = 60.0    # B14
  cl_best_glide: float = 1.0      # D10: CL at best glide speed
  lift_to_drag: float = 12.0      # B50
  forelimb: LimbSystem = field(default_factory=lambda: LimbSystem(1, 0.3, 0.9))
  hindlimb: LimbSystem = field(default_factory=lambda: LimbSystem(1, 0.3, 0.3))


def aerodynamics(a):
  """Planform, stall speed, flapping and climb-out wingbeat (cells B-E)."""
  r = {}
  r['weight'] = a.mass * 9.8                                   # D2
  r['wing_area'] = a.span ** 2 / a.aspect_ratio                # B7
  r['wing_loading'] = a.mass * 9.8 / r['wing_area']            # B8
  r['aero_aspect_ratio'] = a.aspect_ratio + a.slotting_factor * a.aspect_ratio  # B9
  r['stall_speed'] = (2 * r['wing_loading'] / (a.air_density * a.cl)) ** 0.5  # B10
  r['D3'] = r['weight'] / (0.5 * 1.23 * r['wing_area'])        # W/(0.5 rho S)
  r['D4'] = r['D3'] ** 0.5
  r['induced_drag_coefficient'] = (a.cl_best_glide ** 2 /
                                   (math.pi * r['aero_aspect_ratio'] * 1))  # D9
  r['D5'] = r['D4'] * (r['induced_drag_coefficient'] * 1.5)
  r['min_sink'] = r['D5'] / 1.5 ** 1.5                         # D8: m/s
  flap_angle = math.radians(a.flap_angle_deg)                  # B14
  r['flap_amplitude'] = flap_angle * a.span * 0.5              # B15: m
  r['flap_time'] = 1 / (a.mass ** (3 / 8) * 9.8 ** 0.5 * a.span ** (-23 / 24) *
                        r['wing_area'] ** (-1 / 3) *
                        a.air_density ** (-3 / 8))             # B13: s
  r['strouhal'] = (r['flap_amplitude'] * (1 / r['flap_time']) /
                   r['stall_speed'])                           # B12
  # climb-out (first) wingbeat
  r['climbout_flap_angle'] = flap_angle * 2.5                  # D12: rad
  r['climbout_flap_time'] = r['flap_time'] * 0.75              # D13: s
  r['climbout_amplitude'] = r['climbout_flap_angle'] * a.span * 0.5  # D14: m
  r['tip_speed'] = r['climbout_amplitude'] / r['climbout_flap_time']  # D15
  r['quarter_span_speed'] = (r['climbout_amplitude'] * 0.25 /
                             r['climbout_flap_time'] / 0.6)    # D16: m/s
  r['required_glenoid_height'] = r['climbout_amplitude'] * 0.6  # D17: m
  r['initial_upstroke_time'] = 0.5 / (
      a.mass ** (3 / 8) * 9.8 ** 0.5 * a.height ** (-23 / 24) *
      (r['wing_area'] * 0.25) ** (-1 / 3) * 1.23 ** (-3 / 8))  # D18: s
  r['stroke_length'] = 0.9 * a.height                          # D19: m
  return r


def available_power(a):
  """Launch muscle power available (L3*H3 + L4*H4), W."""
  return (a.forelimb.power(a.mass) * a.forelimb.available +
          a.hindlimb.power(a.mass) * a.hindlimb.available)


def launch_row(a, aero, angle_deg):
  """Habib version, one launch angle (row 23-32 columns A-O)."""
  th = math.radians(angle_deg)                                  # B
  v = (aero['stall_speed'] - aero['quarter_span_speed']) / math.cos(th)  # C
  height_gain = v ** 2 * math.sin(th) ** 2 / (2 * 9.8)          # D
  glenoid = height_gain + a.height                              # E
  vy = v * math.sin(th)                                         # G
  push_time = 2 * aero['stroke_length'] / vy                    # H
  accel = v / push_time                                         # I
  accel_g = (accel ** 2 + 9.81 ** 2 -
             2 * accel * 9.81 * math.cos(0.5 * 3.142 + th)) ** 0.5  # J
  power = a.mass * v * accel_g                                  # K
  return {
      'angle_deg': angle_deg, 'angle_rad': th,
      'required_speed': v, 'height_gain': height_gain,
      'glenoid_height': glenoid,
      'clearance': glenoid - aero['required_glenoid_height'],   # F
      'vertical_speed': vy, 'push_time': push_time,
      'acceleration': accel, 'acceleration_with_gravity': accel_g,
      'required_power': power,
      'required_preload': power / available_power(a),           # L
      'ground_force': accel * a.mass,                           # M: N
      'ground_force_g': accel * a.mass / (9.8 * a.mass),        # N
      'time_to_peak': 0.5 * ((v ** 2 * math.sin(2 * th) / 9.8) /
                             (v * math.cos(th))),               # O: s
  }


def push_requirements(mass, speed, angle_deg, stroke):
  """Habib-version push-off requirements for a given takeoff speed and
  angle (instead of the stall-speed-derived speed), e.g. for a robot."""
  th = math.radians(angle_deg)
  vy = speed * math.sin(th)
  push_time = 2 * stroke / vy
  accel = speed / push_time
  accel_g = (accel ** 2 + 9.81 ** 2 -
             2 * accel * 9.81 * math.cos(0.5 * 3.142 + th)) ** 0.5
  return {'speed': speed, 'angle_deg': angle_deg, 'stroke': stroke,
          'push_time': push_time, 'acceleration': accel,
          'ground_force_g': accel / 9.8,
          'peak_power': mass * speed * accel_g,
          'kinetic_energy': 0.5 * mass * speed ** 2}


def palmer_row(a, habib_row):
  """Palmer version: horizontal and vertical power (rows 37-43)."""
  v, th, t = (habib_row['required_speed'], habib_row['angle_rad'],
              habib_row['push_time'])
  vx, vy = v * math.cos(th), v * math.sin(th)                   # C, D
  ax, ay = vx / t, vy / t                                       # H, I
  px = a.mass * ax * vx                                         # J
  py = a.mass * (ay + 9.81) * vy                                # K
  return {'angle_deg': habib_row['angle_deg'],
          'horizontal_speed': vx, 'vertical_speed': vy,
          'horizontal_acceleration': ax, 'vertical_acceleration': ay,
          'horizontal_power': px, 'vertical_power': py,
          'total_power': px + py,
          'required_preload': (px + py) / available_power(a)}  # M


def sustained_flight(a, aero):
  """Rows 49-52: level flight power and muscle ratios."""
  power = (a.mass * 9.8 / a.lift_to_drag *
           (2 * aero['wing_loading'] / (a.air_density * a.cl_best_glide)) ** 0.5)
  fore_mass = a.forelimb.muscle_mass(a.mass)
  return {'power': power, 'continuous_ratio': fore_mass * 175 / power,
          'burst_ratio': fore_mass * 410 / power}


def solve(a):
  aero = aerodynamics(a)
  habib = [launch_row(a, aero, deg) for deg in LAUNCH_ANGLES_DEG]
  palmer = [palmer_row(a, row) for row in habib
            if row['angle_deg'] in PALMER_ANGLES_DEG]
  return {'aero': aero, 'available_power': available_power(a),
          'habib': habib, 'palmer': palmer,
          'sustained': sustained_flight(a, aero)}


def cells(a):
  """Results keyed by the spreadsheet's cell references."""
  s = solve(a)
  ae = s['aero']
  c = {'B2': a.mass, 'B3': a.height, 'B4': a.span, 'B5': a.aspect_ratio,
       'B6': a.cl, 'B7': ae['wing_area'], 'B8': ae['wing_loading'],
       'B9': ae['aero_aspect_ratio'], 'B10': ae['stall_speed'],
       'B12': ae['strouhal'], 'B13': ae['flap_time'],
       'B14': math.radians(a.flap_angle_deg), 'B15': ae['flap_amplitude'],
       'B16': a.air_density, 'B17': a.density_coefficient,
       'B18': a.slotting_factor, 'D2': ae['weight'], 'D3': ae['D3'],
       'D4': ae['D4'], 'D5': ae['D5'], 'D8': ae['min_sink'],
       'D9': ae['induced_drag_coefficient'], 'D10': a.cl_best_glide,
       'D12': ae['climbout_flap_angle'], 'D13': ae['climbout_flap_time'],
       'D14': ae['climbout_amplitude'], 'D15': ae['tip_speed'],
       'D16': ae['quarter_span_speed'], 'D17': ae['required_glenoid_height'],
       'D18': ae['initial_upstroke_time'], 'D19': ae['stroke_length']}
  for row, limb in (('3', a.forelimb), ('4', a.hindlimb)):
    c['H' + row] = limb.available
    c['I' + row] = limb.mass_fraction
    c['J' + row] = limb.muscle_mass(a.mass)
    c['K' + row] = limb.anaerobic_fraction
    c['L' + row] = limb.power(a.mass)
  c['I5'] = a.forelimb.mass_fraction + a.hindlimb.mass_fraction
  c['J5'] = c['J3'] + c['J4']
  c['L5'] = c['L3'] + c['L4']
  keys = ['angle_deg', 'angle_rad', 'required_speed', 'height_gain',
          'glenoid_height', 'clearance', 'vertical_speed', 'push_time',
          'acceleration', 'acceleration_with_gravity', 'required_power',
          'required_preload', 'ground_force', 'ground_force_g', 'time_to_peak']
  for i, row in enumerate(s['habib']):
    for col, key in zip('ABCDEFGHIJKLMNO', keys):
      c[f'{col}{23 + i}'] = row[key]
  pkeys = [('A', 'angle_deg'), ('C', 'horizontal_speed'),
           ('D', 'vertical_speed'), ('H', 'horizontal_acceleration'),
           ('I', 'vertical_acceleration'), ('J', 'horizontal_power'),
           ('K', 'vertical_power'), ('L', 'total_power'),
           ('M', 'required_preload')]
  for i, row in enumerate(s['palmer']):
    for col, key in pkeys:
      c[f'{col}{37 + i}'] = row[key]
  sf = s['sustained']
  c.update({'B49': sf['power'], 'B50': a.lift_to_drag,
            'B51': sf['continuous_ratio'], 'B52': sf['burst_ratio']})
  return c


def main():
  p = argparse.ArgumentParser()
  d = Launcher()
  p.add_argument('--mass', type=float, default=d.mass)
  p.add_argument('--height', type=float, default=d.height)
  p.add_argument('--span', type=float, default=d.span)
  p.add_argument('--aspect', type=float, default=d.aspect_ratio)
  p.add_argument('--cl', type=float, default=d.cl)
  args = p.parse_args()
  a = Launcher(mass=args.mass, height=args.height, span=args.span,
               aspect_ratio=args.aspect, cl=args.cl)
  s = solve(a)
  ae = s['aero']
  print(f"stall speed {ae['stall_speed']:.2f} m/s, climb-out wingbeat adds "
        f"{ae['quarter_span_speed']:.2f} m/s, available power "
        f"{s['available_power']:.0f} W")
  print('angle  speed  push_t  accel   GRF(g)  power_W  preload  clearance')
  for r in s['habib']:
    print(f"{r['angle_deg']:5.0f} {r['required_speed']:6.2f} {r['push_time']:7.3f} "
          f"{r['acceleration']:7.1f} {r['ground_force_g']:7.2f} "
          f"{r['required_power']:8.0f} {r['required_preload']:7.2f} "
          f"{r['clearance']:9.2f}")


if __name__ == '__main__':
  main()
