#!/usr/bin/env python3
"""
Patch script to add base_cog site to crab Robot.xml
"""

import sys
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    print("Usage: patch_crab_robot.py <input.xml> <output.xml>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

# Parse the XML
tree = ET.parse(input_file)
root = tree.getroot()

# Find the worldbody
worldbody = root.find('worldbody')
if worldbody is None:
    print("Error: No worldbody found in Robot.xml")
    sys.exit(1)

# Find the plate body inside worldbody
plate = None
for body in worldbody.findall('body'):
    if body.get('name') == 'plate':
        plate = body
        break

if plate is None:
    print("Error: No plate body found in worldbody")
    sys.exit(1)

# Check if base_cog site already exists
for site in plate.findall('site'):
    if site.get('name') == 'base_cog':
        print("base_cog site already exists, skipping")
        tree.write(output_file)
        sys.exit(0)

# Add the base_cog site to the plate body
site = ET.Element('site')
site.set('name', 'base_cog')
site.set('pos', '0 0 0')
site.set('size', '0.01')
site.set('rgba', '1 0 0 0.3')
site.set('type', 'sphere')
plate.append(site)

# Find and remove the sensor section (we'll define our own in task.xml)
# This must be done because MJPC requires user sensors to come first
sensor = root.find('sensor')
if sensor is not None:
    root.remove(sensor)
    print("Removed sensor section from Robot.xml")

# Write the modified XML
tree.write(output_file)
print(f"Patched Robot.xml: added base_cog site and removed sensor section")

