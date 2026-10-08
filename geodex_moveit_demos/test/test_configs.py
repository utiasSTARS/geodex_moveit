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
"""Every demo parameter file sets only parameters the plugin declares."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
PIPELINE_KEYS = {'planning_plugins', 'request_adapters', 'response_adapters'}


def leaves(node, prefix=''):
    """Return the dotted names of every non-mapping value."""
    out = set()
    for key, value in node.items():
        name = prefix + key
        if isinstance(value, dict) and not ('type' in value and isinstance(value['type'], str)):
            out |= leaves(value, name + '.')
        else:
            out.add(name)
    return out


def declared():
    """Return the dotted names of every parameter in the plugin's schema."""
    path = os.path.join(get_package_share_directory('geodex_moveit'), 'params',
                        'geodex_moveit_parameters.yaml')
    with open(path) as fh:
        return leaves(yaml.safe_load(fh)['geodex_moveit'])


@pytest.mark.parametrize('name', ['panda_geodex.yaml', 'fr3_shelf_geodex.yaml',
                                  'fr3_shelf_geodex_kinetic.yaml'])
def test_config_uses_declared_names(name):
    with open(CONFIG / name) as fh:
        keys = leaves(yaml.safe_load(fh))
    unknown = {k for k in keys - declared()
               if k not in PIPELINE_KEYS and not k.startswith('totg.')}
    assert not unknown, f'{name} sets undeclared parameters: {sorted(unknown)}'
