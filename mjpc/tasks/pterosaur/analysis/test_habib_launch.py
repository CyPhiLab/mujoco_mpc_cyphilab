#!/usr/bin/env python3
"""Check habib_launch.py against launch_version_4.0.xlsx.

habib_ballistic_v4_values.json holds the cached value of every numeric
cell of the 'Ballistic Model (4)' sheet (default inputs). Run:
  python test_habib_launch.py
"""
import json
import math
import os

import habib_launch as H

HERE = os.path.dirname(os.path.abspath(__file__))


def test_matches_spreadsheet():
  with open(os.path.join(HERE, 'habib_ballistic_v4_values.json')) as f:
    expected = json.load(f)
  got = H.cells(H.Launcher())
  missing = sorted(set(expected) - set(got))
  mismatched = []
  for key, value in got.items():
    if key not in expected:
      continue
    if not math.isclose(value, expected[key], rel_tol=1e-9, abs_tol=1e-9):
      mismatched.append((key, value, expected[key]))
  assert not mismatched, f'cells differ: {mismatched}'
  return len(got), missing


if __name__ == '__main__':
  n, missing = test_matches_spreadsheet()
  print(f'{n} cells match the spreadsheet')
  print(f'not ported (inputs/labels only): {missing}')
