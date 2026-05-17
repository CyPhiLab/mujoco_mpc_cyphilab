#!/usr/bin/env python3
"""
Patch script to modify robot_xdomain.xml for MJPC PaddleCrabSwimming task:
- Replace hfield_ground.xml include with noground.xml (pure swimming environment)
- Add base_cog and front_point sites for directional sensing
- Remove sensor section (MJPC defines sensors in task.xml)
"""
import sys
import xml.etree.ElementTree as ET

ACTUATOR_RANGE_GROUPS = (
    (("R1_1", "R1_2", "R1_3", "L1_1", "L1_2", "L1_3"), "-0.9 0.9"),
    (("R2_1", "R2_2", "R2_3", "L2_1", "L2_2", "L2_3"), "-1.0 1.0"),
    (("R3_1", "R3_2", "L3_1", "L3_2"), "-1.2 1.2"),
    (("R3_3", "L3_3"), "-1.5 1.5"),
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

    changed_names = []
    missing_names = sorted(desired_ranges)
    for position in actuator.findall('position'):
        actuator_name = position.get('name')
        if actuator_name not in desired_ranges:
            continue

        missing_names.remove(actuator_name)
        desired_ctrlrange = desired_ranges[actuator_name]
        old_ctrlrange = position.get('ctrlrange')
        old_ctrllimited = position.get('ctrllimited')
        position.set('ctrlrange', desired_ctrlrange)
        position.set('ctrllimited', 'true')
        changed_names.append(
            f"  {actuator_name}: ctrlrange {old_ctrlrange!r} -> {desired_ctrlrange!r}, "
            f"ctrllimited {old_ctrllimited!r} -> 'true'")

    if changed_names:
        print("Applied actuator ctrlrange updates:")
        for line in changed_names:
            print(line)
    else:
        print("Warning: No named position actuators matched the requested ctrlrange updates")

    if missing_names:
        print("Warning: Missing expected actuators:", ', '.join(missing_names))

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
swimming_option.set('integrator', 'implicit')
swimming_option.set('jacobian', 'sparse')
swimming_option.set('timestep', '0.005')
swimming_option.set('gravity', '0 0 0')
swimming_option.set('density', '1000')
swimming_option.set('viscosity', '0.0009')
swimming_option.set('cone', 'elliptic')
root.insert(0, swimming_option)
print("Set swimming physics options (gravity=0, density=1000, viscosity=0.0009)")

# Replace hfield_ground.xml include with noground.xml for swimming
for include in root.findall('include'):
    if include.get('file') == 'hfield_ground.xml':
        include.set('file', 'noground.xml')
        print("Replaced hfield_ground.xml with noground.xml")

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
        site.set('pos', '0 0.20 0')
        site.set('size', '0.015')
        site.set('rgba', '0 1 0 1')
        site.set('type', 'sphere')
        print("front_point site already exists, updated to visible green forward marker")
        break
else:
    site_front = ET.Element('site')
    site_front.set('name', 'front_point')
    site_front.set('pos', '0 0.20 0')
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

# Remove gravcomp attributes (not needed with zero gravity)
for body in root.iter('body'):
    if 'gravcomp' in body.attrib:
        del body.attrib['gravcomp']
print("Removed gravcomp attributes from all bodies")

# Disable robot self-collision for swimming so the planner cannot exploit
# leg-leg impacts as a propulsion strategy.
for body in plate.iter('body'):
    for geom in body.findall('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
print("Disabled self-collision on robot limb geoms")

# Set actuator gains for diagnostic rear-paddle-dominant control.
for default in root.findall('.//default[@class="dynamixel"]'):
    position_elem = default.find('position')
    if position_elem is not None:
        position_elem.set('kp', '50')
        print("Set actuator kp to 50 for rear-paddle diagnostic control")

apply_actuator_ctrlranges(root)

# Remove sensor section (MJPC task.xml defines its own sensors)
sensor = root.find('sensor')
if sensor is not None:
    root.remove(sensor)
    print("Removed sensor section")

tree.write(output_file, xml_declaration=True, encoding='unicode')
print(f"Written patched model to {output_file}")
