#!/usr/bin/env python3
"""
Patch script to modify robot_xdomain.xml for MJPC PaddleCrabSwimming task:
- Replace hfield_ground.xml include with noground.xml (pure swimming environment)
- Add base_cog and front_point sites for directional sensing
- Remove sensor section (MJPC defines sensors in task.xml)
- Widen the actuator ranges so the planner has room to build a stroke
- Give each servo dyntype="user" so the task can supply a control filter
"""
import sys
import xml.etree.ElementTree as ET

# ctrl is the joint setpoint the planner chooses, so these ranges bound the
# stroke it can produce.
#  _1 = proximal (fore-aft sweep about the vertical)
#  _2 = mid (up/down heave, the main stroke joint)
#  _3 = distal (rear paddle feather about the leg axis, or a knee)
# All within the Dynamixel class limit of +-3.14 rad.
ACTUATOR_RANGE_GROUPS = (
    (("R1_1", "R2_1", "R3_1", "L1_1", "L2_1", "L3_1"), "-1.20 1.20"),
    (("R1_2", "R2_2", "R3_2", "L1_2", "L2_2", "L3_2"), "-1.57 1.57"),
    (("R1_3", "R2_3", "L1_3", "L2_3"), "-1.05 1.05"),
    (("R3_3", "L3_3"), "-1.57 1.57"),
)


def apply_actuator_ctrlranges(root):
    actuator = root.find('actuator')
    if actuator is None:
        print("Warning: No actuator section found; skipping ctrlrange patch")
        return

    desired_ranges = {}
    for names, ctrlrange in ACTUATOR_RANGE_GROUPS:
        for name in names:
            desired_ranges[name] = ctrlrange

    changed = []
    missing = sorted(desired_ranges)
    for position in actuator.findall('position'):
        name = position.get('name')
        if name not in desired_ranges:
            continue
        missing.remove(name)
        position.set('ctrlrange', desired_ranges[name])
        position.set('ctrllimited', 'true')
        changed.append(f"  {name}: ctrlrange -> {desired_ranges[name]!r}")

    if changed:
        print("Applied actuator ctrlrange updates:")
        for line in changed:
            print(line)
    else:
        print("Warning: no named position actuators matched the ctrlrange update")

    if missing:
        print("Warning: Missing expected actuators:", ', '.join(missing))


def apply_control_smoothing(root, kp):
    """Rewrites <position> servos as <general> actuators with dyntype="user".

    The smoothing law itself lives in paddle_crab_swimming.cc, which installs
    mjcb_act_dyn and reads its time constant from residual_ControlSmoothingTau
    in task.xml. MuJoCo integrates the returned da/dt and drives the joint from
    the filtered value a instead of straight from ctrl, which is what <position>
    does internally: force = kp * (a - joint_angle).
    """
    actuator = root.find('actuator')
    if actuator is None:
        print("Error: no actuator section found")
        sys.exit(1)

    converted = 0
    for index, position in enumerate(list(actuator)):
        if position.tag != 'position':
            continue
        general = ET.Element('general')
        general.set('name', position.get('name'))
        general.set('joint', position.get('joint'))
        general.set('ctrllimited', 'true')
        general.set('ctrlrange', position.get('ctrlrange'))
        general.set('forcelimited', 'true')
        general.set('forcerange', '-3.1 3.1')
        general.set('dyntype', 'user')
        general.set('gaintype', 'fixed')
        general.set('gainprm', str(kp))
        general.set('biastype', 'affine')
        general.set('biasprm', f'0 {-kp} 0')
        actuator[index] = general
        converted += 1

    print(f"Converted {converted} servos to dyntype='user'; the smoothing time "
          f"constant comes from task.xml")

if len(sys.argv) != 3:
    print("Usage: patch_paddle_crab_robot.py <input.xml> <output.xml>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

ET.register_namespace('', '')
tree = ET.parse(input_file)
root = tree.getroot()

# Set swimming physics: zero gravity, water density/viscosity, implicit integrator
for option in root.findall('option'):
    root.remove(option)
swimming_option = ET.Element('option')
# swimming_option.set('integrator', 'Euler')
swimming_option.set('integrator', 'implicitfast')
swimming_option.set('jacobian', 'sparse')
swimming_option.set('timestep', '0.001')
swimming_option.set('gravity', '0 0 0')
swimming_option.set('density', '1000')
swimming_option.set('cone', 'elliptic')
# swimming_option.set('viscosity', '0.0009')

root.insert(0, swimming_option)
# print("Set swimming physics options (gravity=0, density=1000, viscosity=0.0009)")

# Replace hfield_ground.xml include with noground.xml for swimming
# for include in root.findall('include'):
#     if include.get('file') == 'hfield_ground.xml':
#         include.set('file', 'noground.xml')
#         print("Replaced hfield_ground.xml with noground.xml")

worldbody = root.find('worldbody')
if worldbody is None:
    print("Error: No worldbody found")
    sys.exit(1)

plate = None
for body in worldbody.findall('body'):
    if body.get('name') == 'plate':
        plate = body
        break

if plate is None:
    print("Error: No plate body found in worldbody")
    sys.exit(1)

# Add reference sites if not already present.
# base_cog is kept hidden and used for sensing; front_point is the single
# visible green marker used to define body-forward for Align.
# Swim-forward is -Y: the rear paddles row the body toward -Y. (Leg names
# front/back are legacy from the walking domain and point the other way.)
for site in plate.findall('site'):
    if site.get('name') == 'base_cog':
        site.set('pos', '0 0 0')
        site.set('size', '0.005')
        site.set('rgba', '1 0 0 0')
        site.set('type', 'sphere')
        print("base_cog site already exists, updated to hidden sensor marker")
        break
else:
    site_cog = ET.Element('site')
    site_cog.set('name', 'base_cog')
    site_cog.set('pos', '0 0 0')
    site_cog.set('size', '0.005')
    site_cog.set('rgba', '1 0 0 0')
    site_cog.set('type', 'sphere')
    plate.insert(0, site_cog)
    print("Added hidden base_cog site to plate body")

for site in plate.findall('site'):
    if site.get('name') == 'front_point':
        site.set('pos', '0 -0.20 0')
        site.set('size', '0.015')
        site.set('rgba', '0 1 0 1')
        site.set('type', 'sphere')
        print("front_point site already exists, updated to visible green forward marker")
        break
else:
    site_front = ET.Element('site')
    site_front.set('name', 'front_point')
    site_front.set('pos', '0 -0.20 0')
    site_front.set('size', '0.015')
    site_front.set('rgba', '0 1 0 1')
    site_front.set('type', 'sphere')
    plate.insert(1, site_front)
    print("Added visible green front_point site to plate body")

# Remove old debug axis markers so only the forward point remains visible.
removed_debug_markers = []
for site in list(plate.findall('site')):
    if site.get('name') in ('debug_body_x_axis', 'debug_body_y_axis', 'debug_body_z_axis'):
        plate.remove(site)
        removed_debug_markers.append(site.get('name'))
if removed_debug_markers:
    print("Removed debug markers:", ', '.join(removed_debug_markers))

# Remove unsupported 'projection' attribute from cameras (not supported in MuJoCo 3.x)
for camera in root.iter('camera'):
    if 'projection' in camera.attrib:
        del camera.attrib['projection']
        print(f"Removed unsupported 'projection' attribute from camera '{camera.get('name', '')}'")

# # Remove gravcomp attributes (not needed with zero gravity)
# for body in root.iter('body'):
#     if 'gravcomp' in body.attrib:
#         del body.attrib['gravcomp']
# print("Removed gravcomp attributes from all bodies")

# Disable robot self-collision for swimming so the planner cannot exploit
# leg-leg impacts as a propulsion strategy. Adjacent legs are mounted 0.2 m
# apart but span ~0.17 m, so they overlap at any ctrlrange; narrowing the
# ranges does not avoid it. Fluid forces are unaffected by contype.
for body in plate.iter('body'):
    for geom in body.findall('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
print("Disabled self-collision on robot limb geoms")

# Set actuator gains for diagnostic -paddle-dominant control.
kp = 50
for default in root.findall('.//default[@class="dynamixel"]'):
    position_elem = default.find('position')
    if position_elem is not None:
        position_elem.set('kp', str(kp))
        print(f"Set actuator kp to {kp} for rear-paddle diagnostic control")

apply_actuator_ctrlranges(root)
apply_control_smoothing(root, kp)

# Remove sensor section (MJPC task.xml defines its own sensors)
sensor = root.find('sensor')
if sensor is not None:
    root.remove(sensor)
    print("Removed sensor section")

tree.write(output_file, xml_declaration=True, encoding='unicode')
print(f"Written patched model to {output_file}")
