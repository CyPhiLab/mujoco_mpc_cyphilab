#!/usr/bin/env python3
"""Prepare pterosaur model XMLs for MJPC inclusion.

Usage:
  prepare_model.py robot <src> <dst>
    - Normalizes line endings (CRLF -> LF)
    - Strips <?xml?> declaration and onshape generator comments
      (MuJoCo's <include> cannot handle XML declarations)

  prepare_model.py scene <src> <dst>
    - Normalizes line endings
    - Updates model name and include reference to pterosaur_modified.xml
"""
import re
import sys

mode, src, dst = sys.argv[1], sys.argv[2], sys.argv[3]

with open(src, 'rb') as f:
    content = f.read().decode('utf-8').replace('\r\n', '\n').replace('\r', '\n')

if mode == 'robot':
    # Remove <?xml ... ?> declaration
    content = re.sub(r'<\?xml[^?]*\?>\s*\n?', '', content)
    # Remove onshape-to-robot generator comments
    content = re.sub(r'<!--[^\n]*Generated[^\n]*-->\s*\n?', '', content)
    content = re.sub(r'<!--[^\n]*Onshape[^\n]*-->\s*\n?', '', content, flags=re.IGNORECASE)
elif mode == 'scene':
    content = content.replace(
        '<mujoco model="scene">', '<mujoco model="Pterosaur Scene">', 1)
    content = content.replace(
        '<include file="pterosaur.xml" />', '<include file="pterosaur_modified.xml" />', 1)

with open(dst, 'w', encoding='utf-8', newline='\n') as f:
    f.write(content)
