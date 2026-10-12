"""Build the parameter file the launch hands to the recorder (no ROS imports).

nav2's RewrittenYaml re-dumps the YAML with PyYAML, which writes the string
"y" unquoted; rcl's parser reads a bare ``y`` as a boolean, so a field list
like ``["x", "y", "z"]`` fails with "Sequence should be of same type". Here
every string is written double-quoted, so no value can change type.
"""
import tempfile

import yaml


class _QuotedDumper(yaml.SafeDumper):
    pass


def _str(dumper, data):
    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='"')


_QuotedDumper.add_representer(str, _str)


def write_params(source_file, root_key='', overrides=None):
    """Load ``source_file``, apply top-level ros__parameters overrides,
    optionally nest under ``root_key``, and write a temp YAML; return its path.
    """
    with open(source_file, 'r') as f:
        data = yaml.safe_load(f)
    for node in data.values():
        params = node.get('ros__parameters') if isinstance(node, dict) else None
        if params is not None:
            params.update(overrides or {})
    if root_key:
        data = {root_key: data}
    out = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='dataset_params_', delete=False)
    yaml.dump(data, out, Dumper=_QuotedDumper)
    out.close()
    return out.name
