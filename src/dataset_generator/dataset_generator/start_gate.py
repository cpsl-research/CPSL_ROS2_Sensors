"""
Start gate for the HDF5 recorder (no ROS imports, so it is unit-testable).

The recorder writes nothing until every enabled stream has delivered a
message. ``evaluate`` is the whole decision; the node feeds it the set of
streams seen so far and the time waited.
"""
import json

WAITING = 'waiting'
OPEN = 'open'
ABORT = 'abort'


def evaluate(enabled, seen, elapsed_s, timeout_s, allow_missing):
    """
    Decide whether recording may start.

    ``enabled``: names of the enabled streams (order is kept in ``missing``).
    ``seen``: names that have delivered at least one message.
    ``timeout_s``: 0 (or less) waits forever.

    Returns ``(state, missing)``. ``state`` is ``waiting``, ``open`` or
    ``abort``; ``missing`` lists the enabled streams still silent. With
    ``allow_missing`` a timeout opens the gate instead of aborting.
    """
    seen = set(seen)
    missing = [n for n in enabled if n not in seen]
    if not missing:
        return OPEN, []
    if timeout_s and timeout_s > 0 and elapsed_s >= timeout_s:
        return (OPEN if allow_missing else ABORT), missing
    return WAITING, missing


def gate_line(state, missing_topics, wait_s):
    """Return the one stable log line per gate change (console parses it)."""
    if state == OPEN:
        return 'GATE open wait_s=%.2f' % wait_s
    return 'GATE %s missing=%s' % (state, ','.join(missing_topics))


def gate_json(state, missing, missing_topics, wait_s, timeout_s,
              allow_missing):
    """Return the same state as JSON, for the latched ``~/gate`` topic."""
    return json.dumps({
        'state': state, 'missing': list(missing),
        'missing_topics': list(missing_topics),
        'wait_s': round(float(wait_s), 3), 'timeout_s': float(timeout_s),
        'allow_missing': bool(allow_missing)}, sort_keys=True)
