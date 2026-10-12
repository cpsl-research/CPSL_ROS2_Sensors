"""Parse the `streams:` table of a recorder YAML (no ROS imports)."""

_STREAM_KEYS = ('topic', 'type', 'enabled', 'qos', 'qos_depth', 'mode',
                'fields', 'skip_nans', 'compression', 'legacy_folder',
                'meta_for')


def parse_streams(params):
    """{'radar_adc.topic': v, ...} (the 'streams.' prefix stripped) ->
    {'radar_adc': {'topic': v, ...}}. Pure function, unit-tested."""
    out = {}
    for key, val in params.items():
        name, _, sub = key.partition('.')
        if not sub:
            raise ValueError("streams.%s must be a table with at least "
                             "'topic' and 'type'" % key)
        if sub not in _STREAM_KEYS:
            raise ValueError('streams.%s: unknown key %r (allowed: %s)' %
                             (name, sub, ', '.join(_STREAM_KEYS)))
        out.setdefault(name, {})[sub] = val
    for name, cfg in out.items():
        for req in ('topic', 'type'):
            if req not in cfg:
                raise ValueError('streams.%s is missing %r' % (name, req))
    return out
