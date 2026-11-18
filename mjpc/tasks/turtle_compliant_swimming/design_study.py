#!/usr/bin/env python3
"""Design study: which motor / transmission / articulation changes make the turtle faster?

  python design_study.py --turtle-sim ../../../../turtle_simulator list
  python design_study.py --turtle-sim ... search ak45_serial --budget 200
  python design_study.py --turtle-sim ... sweep ak45_serial --param g_roll --values 1 1.5 2 2.5 3
  python design_study.py --turtle-sim ... export ak45_serial     # model + gait for turtle_trajopt
  python design_study.py --turtle-sim ... table                  # summary of all saved searches

Everything is parametric: each design is a copy of turtle_compliant_mjpc.xml (the baseline
model is never modified) with

  * the front motors swapped (mass and inertia of the motor bodies, reflected rotor inertia as
    joint armature), optionally moved inboard into the shell;
  * an extra external reduction per joint (actuator gear, efficiency eta_ext);
  * a different articulation: a yaw/roll differential (two fixed tendons, one motor on each),
    a passive twist joint (motor removed, the compliant blade twists under load), or twist
    coupled to the stroke by a linkage (joint equality);
  * a motor torque-speed envelope (see `clamp_ctrl`), applied identically here and in
    turtle_trajopt's MPC rollouts (motor parameters are stored in the model as numerics).

The inner loop is the same open-loop, attitude-locked gait study as gait_study.py (the shell can
only translate, so the result is the thrust-limited speed, not limited by pitch stability).
CMA-ES re-optimizes the gait (stroke rate, yaw / roll / twist amplitude, twist timing) for every
design, plus the design's own continuous knobs (reduction ratios, coupling ratio). Finalists are
confirmed with MPC on the free body (turtle_trajopt --task_xml=designs/<name>/task.xml).
"""
import argparse
import dataclasses
import json
import os
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gait_study as gs  # noqa: E402  (biological_table, gait_table, N, BASE_PERIOD)

ACTUATORS = gs.ACTUATORS  # baseline actuator order = joint order of the gait tables
JOINTS = ACTUATORS
BODY_LENGTH = 0.65  # m, shell plus head
G0 = 9.81
# TURTLE_DESIGN_BASE: directory with an alternative turtle_compliant_mjpc.xml + assets/ (e.g.
# another blade n_segments, prepared in a scratch copy); results then go to a separate folder
BASE_DIR = Path(os.environ.get("TURTLE_DESIGN_BASE", HERE))
OUT = HERE / "results" / "design_study" if BASE_DIR == HERE else BASE_DIR / "results"
DESIGNS_DIR = HERE / "designs"
ROOT = "fr13_s105k"

# ----------------------------------------------------------------------------------------------
# Motors. All values at the motor's output shaft (after its built-in gearbox), before any
# external reduction.
#   tau_peak   current-limited peak torque [N m]
#   tau_stall  voltage-limited stall torque: the torque-speed line tau = tau_stall (1 - w / w0)
#   w0         no-load speed [rad/s]
#   km         motor constant at the output, tau / sqrt(copper loss) [N m / sqrt(W)]
#   tau_rated  continuous (thermal) torque [N m], None if not published
#   mass [kg], size, rotor inertia reflected to the output [kg m^2]
# ----------------------------------------------------------------------------------------------
RPM = 2 * np.pi / 60


def _xw540(volt, stall, amps, kt, rpm):
  r = volt / amps  # terminal resistance from the stall point
  return dict(label=f"XW540-T260 @ {volt:g} V", tau_peak=stall, tau_stall=stall, w0=rpm * RPM,
              km=kt / np.sqrt(r), tau_rated=None, mass=0.185, dims=(0.0335, 0.0585, 0.0459),
              # rotor inertia not published: ~0.8 g cm^2 (coreless 17 mm class) x 260.6^2
              rotor=0.8e-7 * 260.6**2, volt=volt)


MOTORS = {
    # ROBOTIS e-manual XW540-T260: gear 260.6:1, 185 g, 33.5 x 58.5 x 45.9 mm, IP68 (1 m, 24 h)
    "XW540-11.1V": _xw540(11.1, 8.8, 4.5, 1.956, 37),
    "XW540-12V": _xw540(12.0, 9.5, 4.9, 1.939, 40),
    "XW540-14.8V": _xw540(14.8, 11.2, 5.9, 1.898, 48),
    # CubeMars AK45-10 V3.0 KV75 product page (Nov 2026): 24 V rated, 10:1, 262 g, 7 N m peak,
    # 2.5 N m rated at 120 rpm, 180 rpm no-load, Kt 0.127 N m/A and Km 0.0858 N m/sqrt(W) at the
    # motor, phase resistance 2.2 ohm, rotor 157.33 g cm^2, backlash 18 arcmin, no IP rating.
    # The voltage-limited line goes through no-load (180 rpm) and the rated point (2.5 N m at
    # 120 rpm), i.e. tau_stall = 2.5 / (1 - 120/180) = 7.5 N m: above 7 N m only below 12 rpm,
    # so the AK45-10 at 24 V behaves almost like a straight torque-speed line, not a flat QDD.
    "AK45-24V": dict(label="AK45-10 KV75 @ 24 V", tau_peak=7.0, tau_stall=7.5, w0=180 * RPM,
                     km=0.0858 * 10, tau_rated=2.5, mass=0.262, dims=(0.053, 0.053, 0.0452),
                     rotor=157.33e-7 * 10**2, volt=24.0),
    # unlimited reference (the previous session's placeholder servo, +-10 N m, no speed limit)
    "ideal": dict(label="ideal servo +-10 N m", tau_peak=10.0, tau_stall=1e9, w0=1e9, km=1e9,
                  tau_rated=None, mass=0.185, dims=(0.0335, 0.0585, 0.0459), rotor=0.0, volt=0),
}


def motor_iso_inertia(mot):
  """Own inertia of a motor about its COM, isotropic approximation (box / short cylinder)."""
  a, b, c = mot["dims"]
  return mot["mass"] * ((a * a + b * b) + (b * b + c * c) + (a * a + c * c)) / 36.0


# ----------------------------------------------------------------------------------------------
# Designs
# ----------------------------------------------------------------------------------------------
@dataclasses.dataclass
class Design:
  name: str
  label: str
  front: str = "AK45-24V"        # MOTORS key for the 6 front motors
  rear: str = "XW540-12V"        # rear hips (trim only; zero in the locked study)
  arch: str = "serial"           # serial: one motor per joint; diff: yaw/roll differential
  twist_drive: str = "motor"     # motor | passive | coupled
  g_yaw: float = 1.0             # external reductions (serial)
  g_roll: float = 1.0            # serial: roll motor; diff: both differential motors
  g_twist: float = 1.0
  eta_ext: float = 0.95          # efficiency of an external stage (belt / spur), when g != 1
  eta_diff: float = 0.90         # bevel / cable differential (always on for arch=diff)
  eta_cable: float = 0.90        # cable / belt drive of a motor moved into the shell
  inboard: bool = False          # motors 2 and 3 moved into the shell (cable / belt drive)
  blade_scale: float = 1.0       # blade torsional stiffness multiplier (= G / 0.2 MPa)
  coupling: float = 0.0          # twist = coupling * roll (twist_drive = coupled)
  twist_range: float = 1.2       # joint-3 travel [rad], +-
  flipper_scale: float = 1.0     # blade span and chord scale about the flipper root (thin plate)
  kp: float = 100.0              # joint-space servo stiffness [N m/rad] (as the previous study)
  kv: float = 1.0                # joint-space servo damping [N m s/rad]
  armature: bool = True          # reflected rotor inertia
  search: tuple = ()             # extra continuous knobs for CMA-ES: (field, lo, hi)
  max_cadence: float = 6.0


DESIGNS = {}


def _d(**kw):
  d = Design(**kw)
  DESIGNS[d.name] = d
  return d


# baseline (the robot as built: XW540-T260 on a 3S pack, serial 3-DOF shoulders)
_d(name="xw540_3s", label="baseline: XW540-T260, 3S (11.1 V), serial", front="XW540-11.1V",
   rear="XW540-11.1V")
_d(name="xw540_12v", label="XW540-T260 @ 12 V, serial", front="XW540-12V")
_d(name="xw540_14v8", label="XW540-T260 @ 14.8 V (4S), serial", front="XW540-14.8V",
   rear="XW540-14.8V")
# articulation with the existing motors
_d(name="xw540_3s_diff", label="XW540 3S, yaw/roll differential", front="XW540-11.1V",
   rear="XW540-11.1V", arch="diff")
_d(name="xw540_3s_diff_passive", label="XW540 3S, differential + passive twist",
   front="XW540-11.1V", rear="XW540-11.1V", arch="diff", twist_drive="passive")
_d(name="xw540_3s_red", label="XW540 3S, serial, optimized gear (speed-up allowed)",
   front="XW540-11.1V", rear="XW540-11.1V", search=(("g_roll", 0.4, 1.5), ("g_twist", 0.4, 1.5)))
_d(name="xw540_3s_diff_red", label="XW540 3S, differential with speed-up gear",
   front="XW540-11.1V", rear="XW540-11.1V", arch="diff", search=(("g_roll", 0.3, 1.5), ("g_twist", 0.4, 1.5)))
_d(name="xw540_3s_bigflip", label="XW540 3S, serial, flipper x1.2 (span and chord)",
   front="XW540-11.1V", rear="XW540-11.1V", flipper_scale=1.2)
_d(name="xw540_3s_flip", label="XW540 3S, serial, flipper scale optimized",
   front="XW540-11.1V", rear="XW540-11.1V", search=(("flipper_scale", 0.8, 1.5),))
# AK45-10 drop-in and with an external reduction
_d(name="ak45_serial", label="AK45-10, serial, direct (drop-in)")
_d(name="ak45_serial_red", label="AK45-10, serial, optimized reductions",
   search=(("g_roll", 1.0, 3.5), ("g_twist", 0.5, 3.0)))
_d(name="ak45_diff", label="AK45-10, yaw/roll differential, direct", arch="diff")
_d(name="ak45_diff_red", label="AK45-10, differential, optimized reductions",
   arch="diff", search=(("g_roll", 0.5, 3.0), ("g_twist", 0.5, 3.0)))
_d(name="ak45_diff_inboard", label="AK45-10, differential, twist motor inboard (cable)",
   arch="diff", inboard=True, search=(("g_roll", 0.7, 3.0), ("g_twist", 0.5, 3.0)))
_d(name="ak45_diff_passive", label="AK45-10, differential + passive twist (4 motors)",
   arch="diff", twist_drive="passive", search=(("g_roll", 0.7, 3.0),))
_d(name="ak45_diff_coupled", label="AK45-10, differential + twist linkage (4 motors)",
   arch="diff", twist_drive="coupled", search=(("g_roll", 0.7, 3.0), ("coupling", -1.0, 1.0)))

# combined: both stroke motors in the shell (differential), twist motor inboard on a cable,
# reductions optimized; sprint (peak-limited) and cruise (RMS <= rated, search --thermal)
_d(name="ak45_combined", label="AK45-10, differential + inboard twist, optimized gears",
   arch="diff", inboard=True, search=(("g_roll", 0.7, 3.0), ("g_twist", 0.5, 3.0), ("flipper_scale", 0.8, 1.4)))
_d(name="ak45_diff_bigflip", label="AK45-10, differential, flipper scale optimized",
   arch="diff", search=(("g_roll", 0.7, 3.0), ("g_twist", 0.5, 3.0), ("flipper_scale", 0.8, 1.4)))


def variant(base, **kw):
  """A copy of a registered design with some fields changed (sweeps)."""
  d = dataclasses.replace(DESIGNS[base], **kw)
  return d


# ----------------------------------------------------------------------------------------------
# Model generation
# ----------------------------------------------------------------------------------------------
_BASE_XML = None
_BASE_MODEL = None


def base_xml():
  global _BASE_XML
  if _BASE_XML is None:
    _BASE_XML = (BASE_DIR / "turtle_compliant_mjpc.xml").read_text()
  return _BASE_XML


def compile_xml(xml, locked=False):
  import mujoco
  if locked:
    xml, n = re.subn(r'<joint name="world_frame_to_base" type="free"[^>]*/>',
                     '<joint name="slide_x" type="slide" axis="1 0 0"/>'
                     '<joint name="slide_y" type="slide" axis="0 1 0"/>'
                     '<joint name="slide_z" type="slide" axis="0 0 1"/>', xml)
    assert n == 1
  tmp = BASE_DIR / f"_design_{os.getpid()}.xml"  # next to the original so meshdir resolves
  tmp.write_text(xml)
  try:
    return mujoco.MjModel.from_xml_path(str(tmp))
  finally:
    tmp.unlink()


def base_model():
  global _BASE_MODEL
  if _BASE_MODEL is None:
    _BASE_MODEL = compile_xml(base_xml())
  return _BASE_MODEL


def _quat2mat(q):
  import mujoco
  m = np.zeros(9)
  mujoco.mju_quat2Mat(m, np.asarray(q, float))
  return m.reshape(3, 3)


def _shift(d):
  d = np.asarray(d, float)
  return d @ d * np.eye(3) - np.outer(d, d)


def edit_inertial(xml, model, body, parts):
  """Add (mass > 0) or remove (mass < 0) point-like parts with isotropic own inertia.
  parts: list of (mass, pos in body frame, own isotropic inertia). Rewrites the body's
  <inertial> as pos + fullinertia in the body frame."""
  b = model.body(body).id
  m0 = model.body_mass[b]
  c0 = model.body_ipos[b].copy()
  r = _quat2mat(model.body_iquat[b])
  i0 = r @ np.diag(model.body_inertia[b]) @ r.T
  mass = m0 + sum(p[0] for p in parts)
  com = (m0 * c0 + sum(p[0] * np.asarray(p[1]) for p in parts)) / mass
  inertia = i0 + m0 * _shift(c0 - com)
  for mp, pos, own in parts:
    own = own * np.eye(3) if np.isscalar(own) else np.asarray(own)
    inertia += np.sign(mp) * own + mp * _shift(np.asarray(pos) - com)
  assert mass > 0, f"{body}: non-positive mass after edit"
  # removing a motor that is most of a light link can leave a slightly non-physical tensor
  # (the CAD's motor model differs from the 185 g mesh estimate): add the smallest isotropic
  # term that restores positive definiteness and the triangle inequality
  a, b, c = np.sort(np.linalg.eigvalsh(inertia))
  delta = max(0.0, c - a - b, 1e-7 - a)
  if delta > 0:
    inertia += (delta + 1e-8) * np.eye(3)
  new = (f'<inertial pos="{com[0]:.6g} {com[1]:.6g} {com[2]:.6g}" mass="{mass:.6g}" '
         f'fullinertia="{inertia[0,0]:.6g} {inertia[1,1]:.6g} {inertia[2,2]:.6g} '
         f'{inertia[0,1]:.6g} {inertia[0,2]:.6g} {inertia[1,2]:.6g}"/>')
  pat = re.compile(r'(<body name="' + re.escape(body) + r'"[^>]*>(?:\s*<!--[^>]*-->)*\s*'
                   r'(?:<joint[^>]*/>\s*)*)<inertial[^>]*/>')
  xml, n = pat.subn(lambda mm: mm.group(1) + new, xml)
  assert n == 1, f"inertial of {body} not found"
  return xml, mass - m0


_XW_PRINCIPAL = None


def _xw540_tensor(model, body):
  """Inertia tensor (about its own COM, body frame) of the XW540 mesh in this body, for a
  185 g motor: the compiled mesh geom frame is the mesh's principal frame."""
  import mujoco
  global _XW_PRINCIPAL
  if _XW_PRINCIPAL is None:
    mini = compile_xml(re.sub(r"<worldbody>.*</worldbody>",
                              '<worldbody><body name="m"><geom type="mesh" mesh="lfs_xw540_n105_i101_stp" '
                              f'mass="{MOTORS["XW540-12V"]["mass"]}"/></body></worldbody>',
                              re.sub(r"<(actuator|tendon|contact|equality|sensor|keyframe)>.*?</\1>", "",
                                     base_xml(), flags=re.S), flags=re.S))
    _XW_PRINCIPAL = mini.body_inertia[mini.body("m").id].copy()
  for g in range(model.ngeom):
    if model.geom_bodyid[g] != model.body(body).id or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
      continue
    mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
    if "xw540" in mesh:
      r = _quat2mat(model.geom_quat[g])
      return r @ np.diag(_XW_PRINCIPAL) @ r.T
  raise KeyError(body)


def _xw540_com(model, body):
  """COM of the XW540 mesh geom in this body (compiled mesh geoms sit at their COM)."""
  import mujoco
  for g in range(model.ngeom):
    if model.geom_bodyid[g] != model.body(body).id or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
      continue
    mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
    if "xw540" in mesh:
      return model.geom_pos[g].copy()
  raise KeyError(f"no XW540 geom in {body}")


def _axis_in_parent(model, joint):
  """Point and direction of a joint axis in its body's parent frame."""
  j = model.joint(joint).id
  b = model.jnt_bodyid[j]
  r = _quat2mat(model.body_quat[b])
  p = model.body_pos[b] + r @ model.jnt_pos[j]
  return p, r @ model.jnt_axis[j]


def _project(point, axis_p, axis_d):
  return axis_p + np.dot(point - axis_p, axis_d) * axis_d


SIDES = {
    "L": dict(link1="left_front_link_1", link2="left_front_link_2", j=("shoulderL1", "shoulderL2", "shoulderL3"),
              shell_motor_body=ROOT),
    "R": dict(link1="right_front_link_1", link2="right_front_link_2", j=("shoulderR1", "shoulderR2", "shoulderR3"),
              shell_motor_body=ROOT),
}


def build(d):
  """Design -> (model xml text, actuator table). Each actuator entry: name, motor key, gear,
  eta, joint-space coefficients (ctrl = coef . q_joint_targets), kp, kv."""
  model = base_model()
  xml = base_xml()
  fm, rm = MOTORS[d.front], MOTORS[d.rear]
  xw = MOTORS["XW540-12V"]  # geometry / mass of the motor currently in the CAD
  xw_iso = motor_iso_inertia(xw)
  fm_iso = motor_iso_inertia(fm)
  shell_parts = []
  notes = []

  for side, s in SIDES.items():
    j1, j2, j3 = s["j"]
    # motor 1 sits in the shell and drives j1; motor 2 sits on link1 and drives j2 (roll);
    # motor 3 sits on link2 and drives j3 (twist)
    p1 = _xw540_com(model, ROOT if side == "L" else ROOT)
    # the root body has two front-shoulder XW540s; pick the one nearest this side's link1
    p_link1 = model.body_pos[model.body(s["link1"]).id]
    cands = []
    import mujoco
    for g in range(model.ngeom):
      if model.geom_bodyid[g] == model.body(ROOT).id and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
        if "xw540" in mesh:
          cands.append(model.geom_pos[g].copy())
    p1 = min(cands, key=lambda p: np.linalg.norm(p - p_link1))
    p2 = _xw540_com(model, s["link1"])
    p3 = _xw540_com(model, s["link2"])
    a1 = _axis_in_parent(model, j1)
    a2 = _axis_in_parent(model, j2)
    a3 = _axis_in_parent(model, j3)
    # new motors coaxial with the joint they drive (pancake motor), at the projection of the
    # old motor's COM onto that axis
    q1, q2, q3 = _project(p1, *a1), _project(p2, *a2), _project(p3, *a3)

    swap = not d.front.startswith("XW540")  # all XW540 voltages: same part
    has_m3 = d.twist_drive == "motor"
    # motor 1 (shell)
    if swap:
      shell_parts += [(-xw["mass"], p1, xw_iso), (fm["mass"], q1, fm_iso)]
    # motor 2: on link1 (serial), or in the shell next to motor 1 (differential: both motors
    # drive the carrier from the base; link1 keeps the bevel gears, ~40 g)
    xw2, xw3 = _xw540_tensor(model, s["link1"]), _xw540_tensor(model, s["link2"])
    link1_parts = []
    if d.arch == "diff" or d.inboard:
      link1_parts += [(-xw["mass"], p2, xw2), (0.040, q2, 0.0)]
      shell_parts += [(fm["mass"], q1 + 0.06 * (q1 - p_link1) / max(np.linalg.norm(q1 - p_link1), 1e-9), fm_iso)]
    elif swap:
      link1_parts += [(-xw["mass"], p2, xw2), (fm["mass"], q2, fm_iso)]
    if link1_parts:
      xml, _ = edit_inertial(xml, model, s["link1"], link1_parts)
    # motor 3: on link2 (twist), in the shell (inboard, cable), or gone (passive / coupled)
    link2_parts = []
    if not has_m3:
      link2_parts += [(-xw["mass"], p3, xw3)]
      if d.twist_drive == "coupled":
        link2_parts += [(0.030, q3, 0.0)]  # linkage
    elif d.inboard:
      link2_parts += [(-xw["mass"], p3, xw3), (0.030, q3, 0.0)]  # pulley
      shell_parts += [(fm["mass"], q1 + 0.12 * (q1 - p_link1) / max(np.linalg.norm(q1 - p_link1), 1e-9), fm_iso)]
    elif swap:
      link2_parts += [(-xw["mass"], p3, xw3), (fm["mass"], q3, fm_iso)]
    if link2_parts:
      xml, _ = edit_inertial(xml, model, s["link2"], link2_parts)

  if shell_parts:
    xml, _ = edit_inertial(xml, model, ROOT, shell_parts)

  # ---- blade stiffness (passive twist / shear modulus) ----
  if d.blade_scale != 1.0:
    def scale(mm):
      k = float(mm.group(2)) * d.blade_scale
      c = float(mm.group(4)) * d.blade_scale
      return f'{mm.group(1)}stiffness="{k:.6g}"{mm.group(3)}damping="{c:.6g}"'
    xml, n = re.subn(r'(<(?:joint name="shoulder[LR]3_twist\d"[^>]*?|fixed name="shoulder[LR]3_tip_spring" ))'
                     r'stiffness="([^"]+)"(\s+)damping="([^"]+)"', scale, xml)
    assert n >= 6 and n % 2 == 0, n  # 2 x (n_segments + 1)

  # ---- flipper size: scale blade meshes (fluid ellipsoids follow) about the flipper root;
  # thin plate of unchanged thickness: mass ~ s^2, COM ~ s, inertia ~ s^4 ----
  if d.flipper_scale != 1.0:
    sc = d.flipper_scale
    xml, n = re.subn(r'<mesh file="([lr]cf_blade_(?:tip|seg\d)\.stl)"/>',
                     rf'<mesh file="\1" scale="{sc:g} {sc:g} {sc:g}"/>', xml)
    assert n >= 6 and n % 2 == 0, n  # 2 x (n_segments + 1)

    def scale_inertial(mm):
      pos = np.array(mm.group(2).split(), float) * sc
      mass = float(mm.group(3)) * sc**2
      fi = np.array(mm.group(4).split(), float) * sc**4
      return (f'{mm.group(1)}<inertial pos="{pos[0]:.6g} {pos[1]:.6g} {pos[2]:.6g}" mass="{mass:.6g}" '
              f'fullinertia="{" ".join(f"{v:.6g}" for v in fi)}"/>')
    xml, n = re.subn(r'(<body name="(?:shoulder[LR]3_blade\d|flipper|flipper_2)"[^>]*>(?:\s*<!--[^>]*-->)*\s*'
                     r'(?:<joint[^>]*/>\s*)*)<inertial pos="([^"]+)" mass="([^"]+)" fullinertia="([^"]+)"/>',
                     scale_inertial, xml)
    assert n >= 6 and n % 2 == 0, n  # 2 x (n_segments + 1)

  # ---- armature (reflected rotor inertia) ----
  def set_armature(joint, value):
    nonlocal xml
    pat = re.compile(r'(<joint (?:axis="0 0 1" )?name="' + joint + r'" type="hinge")( armature="[^"]*")?')
    xml, n = pat.subn(lambda mm: mm.group(1) + f' armature="{value:.6g}"', xml)
    assert n == 1, joint

  def rotor(m):
    return MOTORS[m]["rotor"] if d.armature else 0.0

  # ---- actuators ----
  acts = []
  jidx = {n: i for i, n in enumerate(JOINTS)}

  def coef(**kw):
    c = np.zeros(len(JOINTS))
    for k, v in kw.items():
      c[jidx[k]] = v
    return c

  rng = {"1": 1.2, "2": 1.6, "3": d.twist_range}
  for side, s in SIDES.items():
    j1, j2, j3 = s["j"]
    cable = d.eta_cable if d.inboard else 1.0
    if d.arch == "serial":
      for j, g in ((j1, d.g_yaw), (j2, d.g_roll)):
        eta = (d.eta_ext if abs(g - 1) > 1e-9 else 1.0)
        acts.append(dict(name=j, joint=j, motor=d.front, gear=g, eta=eta, coef=coef(**{j: g}),
                         kp=d.kp / g**2, kv=d.kv / g**2, range=rng[j[-1]] * g))
        set_armature(j, (0.0001 if j == j3 else 0.0) + g * g * rotor(d.front))
    else:
      g = d.g_roll
      eta = d.eta_diff * (d.eta_ext if abs(g - 1) > 1e-9 else 1.0)
      for suffix, sy in (("a", 1.0), ("b", -1.0)):
        acts.append(dict(name=f"motor{side}_{suffix}", tendon=f"diff{side}_{suffix}", motor=d.front,
                         gear=1.0, eta=eta, coef=coef(**{j2: g, j1: sy * g}),
                         kp=d.kp / (2 * g * g), kv=d.kv / (2 * g * g), range=(1.6 + 1.2) * g,
                         tendon_coef={j2: g, j1: sy * g}))
      # J^T diag(Jr) J with J = g [[1, 1], [1, -1]] is diagonal: 2 g^2 Jr on both joints
      set_armature(j1, 2 * g * g * rotor(d.front))
      set_armature(j2, 2 * g * g * rotor(d.front))
    if d.twist_drive == "motor":
      g = d.g_twist
      eta = (d.eta_ext if abs(g - 1) > 1e-9 else 1.0) * cable
      acts.append(dict(name=j3, joint=j3, motor=d.front, gear=g, eta=eta, coef=coef(**{j3: g}),
                       kp=d.kp / g**2, kv=d.kv / g**2, range=rng["3"] * g))
      set_armature(j3, max(0.0005, 0.0001 + g * g * rotor(d.front)))
  for h in ACTUATORS[6:]:
    acts.append(dict(name=h, joint=h, motor=d.rear, gear=1.0, eta=1.0, coef=coef(**{h: 1.0}),
                     kp=d.kp, kv=d.kv, range=1.05))
    set_armature(h, rotor(d.rear))

  lines = []
  for a in acts:
    m = MOTORS[a["motor"]]
    f = m["tau_peak"] * a["eta"]
    trn = f'joint="{a["joint"]}"' if "joint" in a else f'tendon="{a["tendon"]}"'
    gear = f' gear="{a["gear"]:.6g}"' if a["gear"] != 1.0 else ""
    lines.append(f'    <position name="{a["name"]}" {trn}{gear} kp="{a["kp"]:.6g}" kv="{a["kv"]:.6g}" '
                 f'forcerange="-{f:.6g} {f:.6g}" ctrlrange="-{a["range"]:.6g} {a["range"]:.6g}"/>')
  xml, n = re.subn(r"<actuator>.*?</actuator>", "<actuator>\n" + "\n".join(lines) + "\n  </actuator>",
                   xml, flags=re.S)
  assert n == 1

  tendons = []
  for a in acts:
    if "tendon" in a:
      body = "".join(f'\n      <joint joint="{j}" coef="{c:.6g}"/>' for j, c in a["tendon_coef"].items())
      tendons.append(f'    <fixed name="{a["tendon"]}">{body}\n    </fixed>')
  if tendons:
    xml = xml.replace("  <tendon>\n", "  <tendon>\n" + "\n".join(tendons) + "\n", 1)

  if d.twist_drive == "coupled":
    eq = "".join(f'\n    <joint joint1="{s["j"][2]}" joint2="{s["j"][1]}" polycoef="0 {d.coupling:.6g} 0 0 0" '
                 f'solref="0.01 1"/>' for s in SIDES.values())
    xml = xml.replace("  <actuator>", f"  <equality>{eq}\n  </equality>\n  <actuator>", 1)

  # motor parameters for the torque-speed clamp (read by turtle_trajopt too):
  # tau_peak tau_stall w0 eta km (at the motor output, before the external stage)
  nums = []
  for a in acts:
    m = MOTORS[a["motor"]]
    nums.append(f'    <numeric name="motor_{a["name"]}" data="{m["tau_peak"]:.6g} {min(m["tau_stall"], 1e9):.6g} '
                f'{min(m["w0"], 1e9):.6g} {a["eta"]:.6g} {min(m["km"], 1e9):.6g}"/>')
  xml = xml.replace("  <actuator>", "  <custom>\n" + "\n".join(nums) + "\n  </custom>\n  <actuator>", 1)
  header = f"<!-- Design variant '{d.name}' generated by design_study.py: {d.label} -->\n"
  return header + xml, acts


# ----------------------------------------------------------------------------------------------
# Motor envelope and simulation
# ----------------------------------------------------------------------------------------------
class Envelope:
  """Torque-speed clamp for position actuators: force = kp (ctrl - L) - kv v is limited to
     motoring:  |F| <= eta min(tau_peak, tau_stall (1 - |w| / w0))   (F and w same sign)
     braking:   |F| <= eta tau_peak
  where w = actuator velocity (motor side, i.e. after the external gear / tendon coefficients).
  MuJoCo cannot clip a position actuator's force by speed, so the clamp moves ctrl to the
  value that gives the limiting force. turtle_trajopt does the same in mjcb_control."""

  def __init__(self, model, acts):
    self.kp = model.actuator_gainprm[:, 0].copy()
    self.kv = -model.actuator_biasprm[:, 2].copy()
    p = np.array([[MOTORS[a["motor"]][k] for k in ("tau_peak", "tau_stall", "w0")] for a in acts])
    self.tp, self.ts, self.w0 = p.T
    self.eta = np.array([a["eta"] for a in acts])

  def limits(self, w):
    motoring = np.minimum(self.tp, self.ts * (1 - np.abs(w) / self.w0))
    hi = np.where(w >= 0, motoring, self.tp) * self.eta
    lo = -np.where(w <= 0, motoring, self.tp) * self.eta
    return lo, hi

  def __call__(self, model, data):
    length, w = data.actuator_length, data.actuator_velocity
    lo, hi = self.limits(w)
    f = self.kp * (data.ctrl - length) - self.kv * w
    f = np.clip(f, lo, hi)
    data.ctrl[:] = length + (f + self.kv * w) / self.kp


def joint_targets(table, d):
  """Joint-space gait table (N x 10) clipped to the joint ranges of this design."""
  t = table.copy()
  t[:, [0, 3]] = np.clip(t[:, [0, 3]], -1.2, 1.2)
  t[:, [1, 4]] = np.clip(t[:, [1, 4]], -1.6, 1.6)
  t[:, [2, 5]] = np.clip(t[:, [2, 5]], -d.twist_range, d.twist_range)
  return t


def simulate(model, acts, d, table, cadence, strokes=5, skip=2, log=False):
  """Open loop: ctrl = coef . joint targets(phase), envelope clamp every step. Returns metrics."""
  import mujoco
  data = mujoco.MjData(model)
  period = gs.BASE_PERIOD / cadence
  jt = joint_targets(table, d)
  cmat = np.array([a["coef"] for a in acts])  # nu x 10
  ctab = jt @ cmat.T  # N x nu
  env = Envelope(model, acts)
  eta = env.eta
  km = np.array([MOTORS[a["motor"]]["km"] for a in acts])
  root = model.body(ROOT).id
  jq = [model.joint(n).qposadr[0] for n in JOINTS]
  jv = [model.joint(n).dofadr[0] for n in JOINTS]
  for k, a in enumerate(JOINTS):
    if d.twist_drive != "motor" and a in ("shoulderL3", "shoulderR3"):
      continue
    data.qpos[jq[k]] = jt[0, k]
  if d.twist_drive == "coupled":
    data.qpos[jq[2]] = d.coupling * jt[0, 1]
    data.qpos[jq[5]] = d.coupling * jt[0, 4]
  mujoco.mj_forward(model, data)

  mujoco.set_mjcb_control(env)
  try:
    t_end = strokes * period
    t0 = skip * period
    dt = model.opt.timestep
    n = 0
    com0 = None
    acc = dict(mech=0.0, elec=0.0, cu=0.0)
    jwork = np.zeros(len(JOINTS))
    tau2 = np.zeros(model.nu)
    tmax = np.zeros(model.nu)
    wmax = np.zeros(model.nu)
    sat = np.zeros(model.nu)
    rows = []
    while data.time < t_end - 1e-9:
      x = (data.time / period % 1.0) * gs.N
      k = int(x)
      f = x - k
      data.ctrl[:] = (1 - f) * ctab[k % gs.N] + f * ctab[(k + 1) % gs.N]
      mujoco.mj_step(model, data)  # the callback clamps ctrl at the step's state
      if not np.isfinite(data.qpos).all():
        return None
      if data.time >= t0 - 1e-9:
        if com0 is None:
          com0 = data.subtree_com[root].copy()
          tc0 = data.time
        F = data.actuator_force
        w = data.actuator_velocity
        tm = F / eta  # motor torque (before the external stage)
        pm = np.where(F * w > 0, F * w / eta, F * w * eta)  # motor shaft power
        cu = (tm / km) ** 2
        acc["mech"] += np.clip(pm, 0, None).sum() * dt
        acc["cu"] += cu.sum() * dt
        acc["elec"] += np.clip(pm + cu, 0, None).sum() * dt
        jwork += np.clip(data.qfrc_actuator[jv] * data.qvel[jv], 0, None) * dt
        tau2 += tm * tm
        tmax = np.maximum(tmax, np.abs(tm))
        wmax = np.maximum(wmax, np.abs(w))
        lo, hi = env.limits(w)
        sat += (F >= hi / 1.0 - 1e-6) | (F <= lo + 1e-6)
        n += 1
        if log:
          rows.append(np.concatenate([[data.time], data.subtree_com[root], data.qpos[jq], F, w]))
  finally:
    mujoco.set_mjcb_control(None)
  disp = data.subtree_com[root] - com0
  T = data.time - tc0
  speed = -disp[1] / T
  dist = max(-disp[1], 1e-6)
  mass = float(model.body_subtreemass[root])
  names = [a["name"] for a in acts]
  front = slice(0, sum(1 for a in acts if a["name"] not in ACTUATORS[6:]))
  rated = np.array([MOTORS[a["motor"]]["tau_rated"] or np.nan for a in acts])
  rms = np.sqrt(tau2 / n)
  res = dict(
      speed=float(speed), bl_s=float(speed / BODY_LENGTH), climb=float(disp[2] / T),
      mech_power=acc["mech"] / T, elec_power=acc["elec"] / T, copper_power=acc["cu"] / T,
      mech_jpm=acc["mech"] / dist if speed > 0 else float("nan"),
      elec_jpm=acc["elec"] / dist if speed > 0 else float("nan"),
      cot_mech=acc["mech"] / (mass * G0 * dist) if speed > 0 else float("nan"),
      cot_elec=acc["elec"] / (mass * G0 * dist) if speed > 0 else float("nan"),
      mass=mass, period=period, cadence=cadence,
      actuators=names,
      tau_peak=tmax.tolist(), tau_rms=rms.tolist(), w_peak=wmax.tolist(), sat_frac=(sat / n).tolist(),
      rms_over_rated=float(np.nanmax(rms[front] / rated[front])) if np.isfinite(rated[front]).any() else None,
      joint_work=dict(zip(JOINTS, (jwork).tolist())),
  )
  if log:
    res["log"] = np.array(rows)
  return res


def make_model(d, locked=True):
  xml, acts = build(d)
  model = compile_xml(xml, locked=locked)
  for i in range(model.nu):
    model.actuator_ctrllimited[i] = 0  # targets are clipped to the joint ranges instead
  return model, acts


_BIO = None


def bio(turtle_sim):
  global _BIO
  if _BIO is None:
    _BIO = gs.biological_table(turtle_sim)
  return _BIO


def gait(turtle_sim, yaw, roll, twist, shift):
  b, jm, _ = bio(turtle_sim)
  return gs.gait_table(b, jm, yaw, roll, twist, shift)


# ----------------------------------------------------------------------------------------------
# Search
# ----------------------------------------------------------------------------------------------
GAIT_KNOBS = [("cadence", 0.5, None), ("yaw", 0.0, 1.2), ("roll", 0.3, 1.2), ("twist", 0.0, 2.0),
              ("shift", -0.25, 0.25)]


def evaluate(turtle_sim, d, x, cache, locked=True, strokes=5, log=False):
  """x: dict of gait knobs and design fields."""
  fields = {k: v for k, v in x.items() if k in {f.name for f in dataclasses.fields(Design)}}
  dd = dataclasses.replace(d, **fields)
  key = tuple(sorted((k, round(v, 3)) for k, v in fields.items())) + (locked,)
  if key not in cache:
    if len(cache) >= 2:  # compiled models are ~150 MB each; continuous knobs rarely repeat
      cache.clear()
    cache[key] = make_model(dd, locked)
  model, acts = cache[key]
  twist = x.get("twist", 0.0) if dd.twist_drive == "motor" else 0.0
  tab = gait(turtle_sim, x["yaw"], x["roll"], twist, x.get("shift", 0.0))
  res = simulate(model, acts, dd, tab, x["cadence"], strokes=strokes, log=log)
  return res, dd


def search(turtle_sim, d, budget, thermal=False, seed=1, x0=None, quiet=False, vmin=None):
  """vmin: minimize electrical energy per metre subject to speed >= vmin (epsilon-constraint
  point of the speed-efficiency Pareto front) instead of maximizing speed."""
  import cma
  knobs = [k for k in GAIT_KNOBS if not (d.twist_drive != "motor" and k[0] in ("twist", "shift"))]
  knobs = [(n, lo, d.max_cadence if hi is None else hi) for n, lo, hi in knobs] + list(d.search)
  lo = np.array([k[1] for k in knobs])
  hi = np.array([k[2] for k in knobs])
  start = dict(cadence=1.5, yaw=0.2, roll=1.0, twist=1.0, shift=0.0, g_roll=1.5, g_twist=1.0,
               coupling=0.3, blade_scale=1.0)
  for k in knobs:  # design knobs without a preset start at the design's own value
    if k[0] not in start and hasattr(d, k[0]):
      start[k[0]] = getattr(d, k[0])
  start["flipper_scale"] = d.flipper_scale
  if x0:
    start.update(x0)
  u0 = np.clip((np.array([start[k[0]] for k in knobs]) - lo) / (hi - lo), 0.02, 0.98)
  history = []
  cache = {}

  def unpack(u):
    v = lo + (hi - lo) * np.clip(u, 0, 1)
    return {k[0]: float(val) for k, val in zip(knobs, v)}

  def objective(u):
    x = unpack(u)
    res, _ = evaluate(turtle_sim, d, x, cache)
    if res is None or not np.isfinite(res["speed"]):
      return 1.0
    pen = 0.0
    if thermal and res["rms_over_rated"] is not None:
      pen = 0.5 * max(res["rms_over_rated"] - 1.0, 0.0)
    history.append(dict(x=x, **{k: v for k, v in res.items() if k != "log"}))
    if not quiet:
      print(" ".join(f"{k} {v:.3f}" for k, v in x.items()),
            f"-> {res['speed']:.4f} m/s, {res['elec_power']:.1f} W elec, rms/rated {res['rms_over_rated']}",
            flush=True)
    if vmin is None:
      return -res["speed"] + pen
    short = max(vmin - res["speed"], 0.0) / vmin
    jpm = res["elec_jpm"] if res["speed"] > 0 and np.isfinite(res["elec_jpm"]) else 1000.0
    return jpm / 100.0 + 20.0 * short + pen

  es = cma.CMAEvolutionStrategy(u0, 0.2, {"bounds": [0, 1], "maxfevals": budget, "popsize": 10,
                                          "verbose": -9, "seed": seed})
  while not es.stop():
    us = es.ask()
    es.tell(us, [objective(u) for u in us])
  ok = history
  if thermal:
    ok = [h for h in history if h["rms_over_rated"] is None or h["rms_over_rated"] <= 1.02]
  if vmin is None:
    best = max(ok, key=lambda h: h["speed"]) if ok else None
  else:
    ok = [h for h in ok if h["speed"] >= 0.99 * vmin and np.isfinite(h["elec_jpm"])]
    best = min(ok, key=lambda h: h["elec_jpm"]) if ok else None
  return best, history


def save(name, best, history, d, tag=""):
  OUT.mkdir(parents=True, exist_ok=True)
  path = OUT / f"{name}{tag}.json"
  # history: scalar metrics per evaluation only (the per-actuator lists are kept for the best)
  slim = [{k: v for k, v in h.items() if not isinstance(v, (list, dict)) or k == "x"} for h in history]
  path.write_text(json.dumps(dict(design=dataclasses.asdict(d), best=best, history=slim)))
  return path


# ----------------------------------------------------------------------------------------------
# Export for turtle_trajopt
# ----------------------------------------------------------------------------------------------
def export(turtle_sim, d, x, outdir):
  """Write designs/<name>/{model.xml, reference_gait.xml, task.xml} for the MPC runner."""
  fields = {k: v for k, v in x.items() if k in {f.name for f in dataclasses.fields(Design)}}
  dd = dataclasses.replace(d, **fields)
  twist = x.get("twist", 0.0) if dd.twist_drive == "motor" else 0.0
  tab = gait(turtle_sim, x["yaw"], x["roll"], twist, x.get("shift", 0.0))
  return export_table(dd, tab, x["cadence"], outdir, note=json.dumps({k: round(v, 4) for k, v in x.items()}))


def export_table(dd, table, cadence, outdir, note=""):
  """Write the design's model and a joint-space gait table (gs.N x 10, one stroke of the base
  period, played at `cadence`) as designs/<name>/ for turtle_trajopt."""
  x = dict(cadence=cadence)
  xml, acts = build(dd)
  xml = xml.replace('meshdir="assets"', 'meshdir="../../assets"')
  outdir.mkdir(parents=True, exist_ok=True)
  (outdir / "model.xml").write_text(re.sub(r"<\?xml[^?]*\?>\s*", "", xml))
  tab = joint_targets(table, dd)
  cmat = np.array([a["coef"] for a in acts])
  ctab = tab @ cmat.T
  fmt = lambda v: " ".join(f"{z:.6g}" for z in v)
  rows = "\n".join(f'    <numeric name="turtle_gait_{a["name"]}" data="{fmt(ctab[:, i])}"/>'
                   for i, a in enumerate(acts))
  # keyframe: free joint, then joints in model order
  model = compile_xml(xml.replace('meshdir="../../assets"', 'meshdir="assets"'))
  qpos = model.qpos0.copy()
  for k, n in enumerate(JOINTS):
    if dd.twist_drive != "motor" and n in ("shoulderL3", "shoulderR3"):
      continue
    qpos[model.joint(n).qposadr[0]] = tab[0, k]
  if dd.twist_drive == "coupled":
    qpos[model.joint("shoulderL3").qposadr[0]] = dd.coupling * tab[0, 1]
    qpos[model.joint("shoulderR3").qposadr[0]] = dd.coupling * tab[0, 4]
  (outdir / "reference_gait.xml").write_text(f"""<!-- Generated by design_study.py export for design '{dd.name}'.
     Gait in actuator (motor) coordinates, {gs.N} samples over one stroke of the base period;
     play at Cadence={x['cadence']:.4f}. {note} -->
<mujoco>
  <custom>
    <numeric name="turtle_gait_period" data="{gs.BASE_PERIOD:.6f}"/>
{rows}
  </custom>
  <keyframe>
    <key name="home" qpos="{fmt(qpos)}" ctrl="{fmt(ctab[0])}"/>
  </keyframe>
</mujoco>
""")
  task = (HERE / "task.xml").read_text()
  task = task.replace('<include file="../common.xml"/>', '<include file="../../../common.xml"/>')
  task = task.replace('<include file="turtle_compliant_mjpc.xml"/>', '<include file="model.xml"/>')
  task = task.replace('<user name="Gait" dim="10"', f'<user name="Gait" dim="{len(acts)}"')
  task = re.sub(r'(<numeric name="residual_Cadence" data=")[^"]*"', rf'\g<1>{x["cadence"]:.4f} 0.5 8"', task)
  (outdir / "task.xml").write_text(task)
  csv = outdir / "gait.csv"
  np.savetxt(csv, ctab, delimiter=",", header=",".join(a["name"] for a in acts), comments="", fmt="%.6f")
  return dd, acts


# ----------------------------------------------------------------------------------------------
def summary_row(name, rec):
  b = rec["best"]
  d = rec["design"]
  front = [i for i, a in enumerate(b["actuators"]) if a not in ACTUATORS[6:]]
  tp = max(b["tau_peak"][i] for i in front)
  tr = max(b["tau_rms"][i] for i in front)
  wp = max(b["w_peak"][i] for i in front)
  return (name, d["label"], b["speed"], b["bl_s"], b["mech_jpm"], b["elec_jpm"], b["elec_power"], tp, tr, wp,
          b["mass"], b["x"])


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--turtle-sim", required=True)
  sub = ap.add_subparsers(dest="cmd", required=True)
  sub.add_parser("list")
  s = sub.add_parser("search")
  s.add_argument("name")
  s.add_argument("--budget", type=int, default=200)
  s.add_argument("--thermal", action="store_true", help="penalize RMS motor torque above the rated torque")
  s.add_argument("--vmin", type=float, default=None,
                 help="minimize electrical energy per metre at speed >= vmin (Pareto point) instead of max speed")
  s.add_argument("--set", nargs="*", default=[], help="design field overrides, field=value")
  s.add_argument("--tag", default="")
  s.add_argument("--seed", type=int, default=1)
  w = sub.add_parser("sweep", help="search the gait at each value of one design field")
  w.add_argument("name")
  w.add_argument("--param", required=True)
  w.add_argument("--values", type=float, nargs="+", required=True)
  w.add_argument("--budget", type=int, default=150)
  w.add_argument("--thermal", action="store_true")
  e = sub.add_parser("eval", help="evaluate one gait on a design")
  e.add_argument("name")
  e.add_argument("--x", required=True, help='JSON dict of gait knobs / design fields')
  e.add_argument("--free", action="store_true", help="free body instead of attitude locked")
  e.add_argument("--strokes", type=int, default=6)
  x = sub.add_parser("export")
  x.add_argument("name")
  x.add_argument("--tag", default="")
  sub.add_parser("table")
  args = ap.parse_args()

  if args.cmd == "list":
    for n, d in DESIGNS.items():
      print(f"{n:24s} {d.label}")
    return

  if args.cmd == "search":
    over = {}
    for item in args.set:
      k, v = item.split("=")
      over[k] = type(getattr(DESIGNS[args.name], k))(v) if not isinstance(getattr(DESIGNS[args.name], k), bool) \
          else v.lower() in ("1", "true", "yes")
    d = variant(args.name, **over)
    best, hist = search(args.turtle_sim, d, args.budget, args.thermal, seed=args.seed, vmin=args.vmin)
    tag = args.tag or (("_thermal" if args.thermal else "") + (f"_e{args.vmin:g}" if args.vmin else ""))
    p = save(args.name, best, hist, d, tag)
    print("best:", json.dumps({k: v for k, v in best.items() if k in ("x", "speed", "bl_s", "elec_power",
                                                                      "elec_jpm", "rms_over_rated")}))
    print("saved", p)
    return

  if args.cmd == "sweep":
    for v in args.values:
      d = variant(args.name, **{args.param: v})
      d.search = tuple(k for k in d.search if k[0] != args.param)
      best, hist = search(args.turtle_sim, d, args.budget, args.thermal, quiet=True)
      tag = f"_{args.param}{v:g}" + ("_thermal" if args.thermal else "")
      save(args.name, best, hist, d, tag)
      print(f"{args.param}={v:g}: {best['speed']:.4f} m/s, x {json.dumps(best['x'])}, "
            f"elec {best['elec_power']:.1f} W, rms/rated {best['rms_over_rated']}", flush=True)
    return

  if args.cmd == "eval":
    d = DESIGNS[args.name]
    xx = json.loads(args.x)
    res, dd = evaluate(args.turtle_sim, d, xx, {}, locked=not args.free, strokes=args.strokes)
    print(json.dumps({k: v for k, v in res.items() if k != "log"}, indent=1))
    return

  if args.cmd == "export":
    rec = json.loads((OUT / f"{args.name}{args.tag}.json").read_text())
    d = Design(**{**rec["design"], "search": tuple(map(tuple, rec["design"]["search"]))})
    outdir = DESIGNS_DIR / (args.name + args.tag)
    dd, acts = export(args.turtle_sim, d, rec["best"]["x"], outdir)
    print(f"wrote {outdir}/: task.xml model.xml reference_gait.xml gait.csv; "
          f"run: turtle_trajopt --task_xml={outdir}/task.xml --param=Cadence={rec['best']['x']['cadence']:.4f}")
    return

  if args.cmd == "table":
    rows = []
    for p in sorted(OUT.glob("*.json")):
      rec = json.loads(p.read_text())
      if rec.get("best"):
        rows.append(summary_row(p.stem, rec))
    rows.sort(key=lambda r: -r[2])
    print("| run | design | speed m/s | BL/s | mech J/m | elec J/m | elec W | peak tau | rms tau | peak w | mass |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
      print(f"| {r[0]} | {r[1]} | {r[2]:.3f} | {r[3]:.2f} | {r[4]:.0f} | {r[5]:.0f} | {r[6]:.1f} | {r[7]:.1f} | "
            f"{r[8]:.2f} | {r[9]:.1f} | {r[10]:.2f} |")


if __name__ == "__main__":
  main()
