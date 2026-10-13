"""Parse the `streams:` table of a recorder YAML (no ROS imports)."""

_STREAM_KEYS = ('topic', 'type', 'enabled', 'qos', 'qos_depth', 'mode',
                'fields', 'skip_nans', 'compression', 'legacy_folder',
                'meta_for', 'config_topic')
# Free-form `mount_*` keys (mount_frame, mount_xyz, ...) are annotations only:
# accepted here and stored verbatim in the file's config_streams attr.


def parse_streams(params):
    """{'radar_adc.topic': v, ...} (the 'streams.' prefix stripped) ->
    {'radar_adc': {'topic': v, ...}}. Pure function, unit-tested."""
    out = {}
    for key, val in params.items():
        name, _, sub = key.partition('.')
        if not sub:
            raise ValueError("streams.%s must be a table with at least "
                             "'topic' and 'type'" % key)
        if sub not in _STREAM_KEYS and not sub.startswith('mount_'):
            raise ValueError('streams.%s: unknown key %r (allowed: %s)' %
                             (name, sub, ', '.join(_STREAM_KEYS)))
        out.setdefault(name, {})[sub] = val
    for name, cfg in out.items():
        for req in ('topic', 'type'):
            if req not in cfg:
                raise ValueError('streams.%s is missing %r' % (name, req))
    return out


TF_RESERVED = ('tf', 'tf_static')   # HDF5 groups the recorder writes itself


def _frame(name):
    return str(name).strip().lstrip('/')


def parse_tf_track(values):
    """
    Parse ``tf_track`` ("parent->child" strings) into (parent, child) tuples.

    ``None``, ``''`` and ``[""]`` mean off. Anything that is not exactly
    ``a->b`` raises ValueError. A leading ``/`` on a frame is dropped and
    duplicates are removed (order kept).
    """
    if values is None:
        return []
    if isinstance(values, str):
        values = [values]
    pairs = []
    for raw in values:
        text = str(raw).strip()
        if not text:
            continue
        parts = text.split('->')
        if len(parts) != 2 or not _frame(parts[0]) or not _frame(parts[1]):
            raise ValueError(
                "tf_track entry %r must look like 'parent->child'" % raw)
        pair = (_frame(parts[0]), _frame(parts[1]))
        if pair not in pairs:
            pairs.append(pair)
    return pairs


def tf_stream_name(parent, child):
    """Return the HDF5 group for a tracked pair: ``tf/<parent>__<child>``."""
    return 'tf/%s__%s' % (parent.replace('/', '.'), child.replace('/', '.'))
