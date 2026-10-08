#!/usr/bin/env python3
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
"""
Print the attached_object.spheres parameter from the spheres foam fits to an object mesh.

foam's generate_spheres.py writes one list of spheres per level of its sphere tree. The script
takes one level, moves the spheres from the mesh frame to the frame of the link that holds the
object, and prints the parameter block. With --mesh, it also prints the largest distance from
100000 surface samples and the vertices of the mesh to the spheres, as an attachment_padding.

    attached_object_spheres.py <spheres.json> [--level L] [--pose X Y Z ROLL PITCH YAW] [--mesh M]
"""

import argparse
import json
import math

import numpy as np


def rotation(roll, pitch, yaw):
    """Return the rotation of fixed-axis roll, pitch and yaw, as in a URDF origin."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def link_spheres(levels, level, pose):
    """Return one level's spheres as rows of x, y, z and radius in the link frame."""
    spheres = levels[level]['spheres']
    centers = np.array([s['origin'] for s in spheres], dtype=float)
    radii = np.array([s['radius'] for s in spheres], dtype=float)
    centers = centers @ rotation(*pose[3:]).T + np.asarray(pose[:3], dtype=float)
    # Micrometer precision, with the radii rounded up.
    return np.column_stack([np.round(centers, 6), np.ceil(radii * 1e6 - 1e-9) / 1e6])


def uncovered_gap(spheres, mesh_path, pose, samples=100000):
    """Return the largest distance from the mesh surface to the nearest sphere, 0 when covered."""
    import trimesh
    mesh = trimesh.load(mesh_path, force='mesh')
    points, _ = trimesh.sample.sample_surface(mesh, samples, seed=0)
    points = np.vstack([points, mesh.vertices])
    points = points @ rotation(*pose[3:]).T + np.asarray(pose[:3], dtype=float)
    gap = np.full(len(points), np.inf)
    for x, y, z, r in spheres:
        gap = np.minimum(gap, np.linalg.norm(points - (x, y, z), axis=1) - r)
    return max(float(gap.max()), 0.0)


def parameter_block(spheres):
    """Return the YAML block of attached_object.spheres, two spheres per line."""
    # ROS 2 reads a list as double_array only when every value has a decimal point.
    values = [f'{v:.6f}'.rstrip('0') for v in spheres.ravel()]
    values = [v + '0' if v.endswith('.') else v for v in values]
    values = ['0.0' if v in ('-0.0', '0.0') else v for v in values]
    rows = [', '.join(values[i:i + 8]) for i in range(0, len(values), 8)]
    return 'attached_object:\n  spheres: [\n      ' + ',\n      '.join(rows) + ']\n'


def main(argv=None):
    """Print the parameter block, and the uncovered gap with --mesh."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('spheres', help="foam's JSON output")
    parser.add_argument('--level', type=int, default=-1,
                        help='sphere tree level, 0 for the enclosing sphere (default: last)')
    parser.add_argument('--pose', type=float, nargs=6, default=[0.0] * 6,
                        metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                        help='pose of the mesh frame in the link frame (default: identity)')
    parser.add_argument('--mesh', help='the mesh foam spherized, to measure the uncovered gap')
    args = parser.parse_args(argv)
    with open(args.spheres) as f:
        levels = json.load(f)
    spheres = link_spheres(levels, args.level, args.pose)
    print(parameter_block(spheres), end='')
    if args.mesh:
        gap = uncovered_gap(spheres, args.mesh, args.pose)
        count = f'{len(spheres)} sphere' + ('s' if len(spheres) > 1 else '')
        if gap > 0.0:
            print(f'# {count}. attachment_padding of at least '
                  f'{math.ceil(gap * 1e4) / 1e4:.4f} m covers the sampled mesh.')
        else:
            print(f'# {count}, covering the sampled mesh.')


if __name__ == '__main__':
    main()
