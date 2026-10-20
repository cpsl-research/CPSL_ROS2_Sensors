"""Every preset in configs/ is a v2 recorder YAML that parse_streams accepts (no ROS needed)."""
import glob
import os

import pytest
import yaml

from dataset_generator.stream_config import parse_streams, parse_tf_track

CONFIGS = sorted(glob.glob(os.path.join(
    os.path.dirname(__file__), '..', 'configs', '*.yaml')))


def test_configs_found():
    assert CONFIGS


@pytest.mark.parametrize('path', CONFIGS, ids=os.path.basename)
def test_config_parses(path):
    with open(path) as f:
        params = yaml.safe_load(f)['dataset_generator']['ros__parameters']
    assert 'streams' in params, 'v1 YAML (no streams:)'
    flat = {'%s.%s' % (n, k): v for n, c in params['streams'].items()
            for k, v in c.items()}
    streams = parse_streams(flat)
    assert streams.keys() == params['streams'].keys()
    parse_tf_track(params.get('tf_track'))
