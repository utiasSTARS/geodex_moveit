# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""attached_object_spheres.py turns foam's levels into the attached_object.spheres parameter."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'attached_object_spheres.py'
SPEC = importlib.util.spec_from_file_location('attached_object_spheres', SCRIPT)
attached_object_spheres = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(attached_object_spheres)

# foam's output for a 0.04 x 0.24 x 0.16 m box, cut to two spheres in level 1.
LEVELS = [
    {'spheres': [{'origin': [0.0, 0.0, 0.0], 'radius': 0.145748}]},
    {'spheres': [{'origin': [0.0, -0.06, 0.0], 'radius': 0.1},
                 {'origin': [0.0, 0.06, 0.0], 'radius': 0.1}]},
]


def test_moves_the_spheres_into_the_link_frame(tmp_path, capsys):
    """The pose moves foam's spheres from the mesh frame to the link frame."""
    path = tmp_path / 'box-spheres.json'
    path.write_text(json.dumps(LEVELS))
    attached_object_spheres.main([str(path), '--pose', '0', '0', '0.055', '0', '0',
                                  str(math.pi / 2)])
    spheres = np.array(yaml.safe_load(capsys.readouterr().out)['attached_object']['spheres'])
    assert spheres.reshape(-1, 4) == pytest.approx(
        np.array([[0.06, 0.0, 0.055, 0.1], [-0.06, 0.0, 0.055, 0.1]]), abs=1e-9)


def test_level_zero_is_the_enclosing_sphere(tmp_path, capsys):
    """Level 0 of foam's tree is one sphere that encloses the mesh."""
    path = tmp_path / 'box-spheres.json'
    path.write_text(json.dumps(LEVELS))
    attached_object_spheres.main([str(path), '--level', '0'])
    spheres = yaml.safe_load(capsys.readouterr().out)['attached_object']['spheres']
    assert spheres == pytest.approx([0.0, 0.0, 0.0, 0.145748])
    assert all(isinstance(v, float) for v in spheres)


def test_radii_round_up():
    """Centers round to micrometers, and radii round up."""
    levels = [{'spheres': [{'origin': [0.1234567, 0.0, 0.0], 'radius': 0.0100001}]}]
    x, _, _, r = attached_object_spheres.link_spheres(levels, 0, [0.0] * 6)[0]
    assert x == pytest.approx(0.123457)
    assert r == pytest.approx(0.010001)


def test_measures_the_uncovered_gap(tmp_path):
    """The gap is 0 for a covering sphere and the corner distance for a small one."""
    trimesh = pytest.importorskip('trimesh')
    mesh = tmp_path / 'box.obj'
    trimesh.creation.box(extents=[0.04, 0.24, 0.16]).export(mesh)
    one = attached_object_spheres.link_spheres(LEVELS, 0, [0.0] * 6)
    assert attached_object_spheres.uncovered_gap(one, mesh, [0.0] * 6) == 0.0
    small = np.array([[0.0, 0.0, 0.0, 0.1]])
    corner = math.sqrt(0.02 ** 2 + 0.12 ** 2 + 0.08 ** 2)
    assert attached_object_spheres.uncovered_gap(small, mesh, [0.0] * 6) == pytest.approx(
        corner - 0.1)
