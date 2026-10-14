"""dataset_recorder: record the streams listed in the YAML into one HDF5 file.

Replaces the per-sensor ``*_enable`` / ``*_topic`` parameters of the v1
``dataset_generator`` node with a ``streams:`` table (see configs/radar_sar.yaml
and docs/hdf5_format.md). Every sample is stored with its header stamp and the
host receive time.

Start gate: nothing is written until every enabled stream has delivered a
message (``start_timeout_s``; ``allow_missing`` starts anyway at the timeout).
On timeout without ``allow_missing`` the node logs ``GATE abort`` and exits
with code 2, having created no file.

Transforms (format v4): the last ``/tf_static`` transform per frame pair is
written to group ``/tf_static``, and every direct ``parent->child`` edge named
in ``tf_track`` is recorded from ``tf_topic`` as stream ``/tf/<p>__<c>``.
These never gate the start.
"""
import datetime
import hashlib
import json
import os
import queue
import signal
import socket
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message

from dataset_generator import radar_cfg, start_gate
from dataset_generator.converters import (get_converter, normalize_type,
                                          Sample, TF_COLUMNS, tf_transforms)
from dataset_generator.hdf5_writer import CaptureFile
from dataset_generator.stream_config import (parse_streams, parse_tf_track,
                                             TF_RESERVED, tf_stream_name)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

_CFG = object()   # queue tag: (_CFG, stream, radar cfg path)
_CALL = object()  # queue tag: (_CALL, fn) - run fn on the writer thread
GATE_EXIT_CODE = 2   # exit status when the start gate aborts


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__(
            'dataset_generator',
            automatically_declare_parameters_from_overrides=True)
        self._p = {}
        for name, default in (
                ('output_file', ''), ('dataset_path', ''),
                ('capture_mode', 'all'), ('sample_rate_hz', 10.0),
                ('flush_period_s', 5.0), ('duration_s', 0.0),
                ('queue_size', 256), ('start_timeout_s', 30.0),
                ('allow_missing', False), ('tf_topic', 'tf'),
                ('tf_static_topic', 'tf_static'),
                ('tf_track', ['odom->base_link']),
                ('robot_description_topic', 'robot_description')):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            self._p[name] = self.get_parameter(name).value
        if self._p['capture_mode'] not in ('all', 'sampled'):
            raise ValueError("capture_mode must be 'all' or 'sampled'")

        raw = {k: v.value for k, v in
               self.get_parameters_by_prefix('streams').items()}
        self.cfgs = parse_streams(raw)
        for name in self.cfgs:
            if name in TF_RESERVED:
                raise ValueError('stream name %r is reserved for the '
                                 "recorder's transform groups" % name)
        self.tf_pairs = parse_tf_track(self._p['tf_track'])
        self.tf_names = {p: tf_stream_name(*p) for p in self.tf_pairs}
        self._tfs = {}         # (parent, child) -> (stamp_ns, data, recv_ns)
        self._urdf = None
        self.out_path = self._output_path()
        # The file is created when the start gate opens (_open_gate).
        self.cap = None
        self.exit_code = 0
        self.writer = None
        self._gate_open = False
        self._gate_last = None
        self._seen = set()
        self._pending = []         # (name, attrs, compression, keep_stamps)
        self._pending_attrs = {}   # name -> attrs set before the file exists
        self._held_cfg = {}        # name -> radar cfg paths seen before open
        self.q = queue.Queue(maxsize=int(self._p['queue_size']))
        self.dropped = {}
        self._described = set()
        self.latest = {}      # stream -> (seq, msg, recv_ns), sampled mode
        self.written_seq = {}
        self.seq = {}
        self.meta_pairs = []
        self.cfg_streams = {}   # stream -> config topic
        self._cfg_first = {}    # stream -> first radar cfg path seen
        self._cfg_writer_path = {}   # writer thread only
        self.subs = []
        for name, cfg in self.cfgs.items():
            if not cfg.get('enabled', True):
                continue
            self._add_stream(name, cfg)

        self._add_tf()
        self._enabled = [n for n, c in self.cfgs.items()
                         if c.get('enabled', True)]
        self._topics = {n: self.cfgs[n]['topic'] for n in self._enabled}
        self._t0 = time.monotonic()
        # latched, so a late subscriber still sees the last gate state
        self._gate_pub = self.create_publisher(
            String, '~/gate', QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self._gate_timer = self.create_timer(0.1, self._gate_tick)
        self._gate_tick()

    # ---- transforms (format v4) -------------------------------------------
    def _resolved(self, topic):
        """Absolute topic name as the middleware sees it ('' stays '')."""
        return self.resolve_topic_name(topic) if topic else topic

    def _add_tf(self):
        rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        if self.tf_pairs:
            topic = self._p['tf_topic']
            for pair, name in self.tf_names.items():
                self.seq[name] = 0
                self.dropped[name] = 0
                self._pending.append((name, {
                    'topic': topic, 'msg_type': 'tf2_msgs/msg/TFMessage',
                    'qos': 'reliable', 'qos_depth': 100, 'mode': 'all',
                    'columns': list(TF_COLUMNS),
                    'parent_frame': pair[0], 'child_frame': pair[1]},
                    None, False))
            self.subs.append(self.create_subscription(
                TFMessage, topic, self._on_tf, rel))
            self.get_logger().info('tracking tf %s from %s' % (
                ', '.join('%s->%s' % p for p in self.tf_pairs), topic))
        if self._p['tf_static_topic']:
            self.subs.append(self.create_subscription(
                TFMessage, self._p['tf_static_topic'], self._on_tf_static,
                latched))
        if self._p['robot_description_topic']:
            self.subs.append(self.create_subscription(
                String, self._p['robot_description_topic'], self._on_urdf,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST, depth=1)))

    def _on_tf(self, msg):
        recv_ns = time.time_ns()
        if not self._gate_open:
            return
        # filter here, before the queue: /tf carries the whole robot
        for r in tf_transforms(msg, self.tf_names):
            name = self.tf_names[(r.parent, r.child)]
            try:
                self.q.put_nowait((name, Sample(
                    r.stamp_ns, {'data': r.data}, ()), recv_ns, {}))
            except queue.Full:
                self.dropped[name] += 1
                self.get_logger().warning(
                    '%s: write queue full, sample dropped' % name,
                    throttle_duration_sec=5.0)

    def _on_tf_static(self, msg):
        recv_ns = time.time_ns()
        changed = False
        for r in tf_transforms(msg):
            old = self._tfs.get((r.parent, r.child))
            if old is None or not np.array_equal(old[1], r.data):
                self._tfs[(r.parent, r.child)] = (r.stamp_ns, r.data, recv_ns)
                changed = True
        if changed and self._gate_open:
            self._queue_call(self._write_tf_static)

    def _write_tf_static(self):
        """Writer thread (or main thread at close): rewrite /tf_static."""
        self.cap.write_tf_static(
            (p, c, v[1][:3], v[1][3:], v[0], v[2])
            for (p, c), v in list(self._tfs.items()))

    def _on_urdf(self, msg):
        self._urdf = str(msg.data)
        if self._gate_open:
            self._queue_call(self._write_urdf)

    def _write_urdf(self):
        if self._urdf:
            self.cap.set_root_attr('robot_description', self._urdf)

    def _queue_call(self, fn):
        try:
            self.q.put((_CALL, fn), timeout=2.0)
        except queue.Full:
            self.get_logger().error('write queue full, %s skipped' %
                                    fn.__name__)

    # ---- start gate -------------------------------------------------------
    def _gate_tick(self):
        if self._gate_open or self.exit_code:
            return
        wait_s = time.monotonic() - self._t0
        timeout = float(self._p['start_timeout_s'])
        allow = bool(self._p['allow_missing'])
        state, missing = start_gate.evaluate(
            self._enabled, self._seen, wait_s, timeout, allow)
        topics = [self._topics[n] for n in missing]
        key = (state, tuple(missing))
        if key != self._gate_last:
            self._gate_last = key
            line = start_gate.gate_line(state, topics, wait_s)
            if state == start_gate.ABORT:
                self.get_logger().error(line)
            else:
                self.get_logger().info(line)
            self._gate_pub.publish(String(data=start_gate.gate_json(
                state, missing, topics, wait_s, timeout, allow)))
        if state == start_gate.WAITING:
            return
        self._gate_timer.cancel()
        if state == start_gate.ABORT:
            self.exit_code = GATE_EXIT_CODE
            rclpy.try_shutdown()
            return
        self._open_gate(missing, wait_s)

    def _open_gate(self, missing, wait_s):
        opened_ns = time.time_ns()
        attrs = self._root_attrs()
        attrs['start_gate'] = {
            'timeout_s': float(self._p['start_timeout_s']),
            'allow_missing': bool(self._p['allow_missing']),
            'wait_s': round(wait_s, 3), 'opened_recv_ns': opened_ns}
        attrs['streams_missing'] = list(missing)
        attrs['tf_track'] = ['%s->%s' % p for p in self.tf_pairs]
        # resolved (absolute) names: a relative default lands in the node's namespace
        attrs['tf_topic'] = self._resolved(self._p['tf_topic'])
        attrs['tf_static_topic'] = self._resolved(self._p['tf_static_topic'])
        os.makedirs(os.path.dirname(self.out_path) or '.', exist_ok=True)
        self.cap = CaptureFile(
            self.out_path, attrs,
            flush_period_s=float(self._p['flush_period_s']))
        for name, a, compression, keep in self._pending:
            self.cap.add_stream(name, a, compression=compression,
                                keep_stamps=keep)
        for name, a in self._pending_attrs.items():
            self.cap.update_stream_attrs(name, a)
        self._pending = []
        self._pending_attrs = {}
        self._gate_open = True
        self.writer = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer.start()
        for name, paths in self._held_cfg.items():
            for path in paths:
                self._queue_cfg(name, path)
        self._held_cfg = {}
        self._queue_call(self._write_tf_static)
        if self._urdf:
            self._queue_call(self._write_urdf)
        if any(self._mode(c) == 'sampled' for c in self.cfgs.values()):
            self.create_timer(1.0 / float(self._p['sample_rate_hz']),
                              self._sample_tick)
        if float(self._p['duration_s']) > 0:
            self._stop_timer = self.create_timer(
                float(self._p['duration_s']), self._duration_done)
        self.get_logger().info('recording %d stream(s) to %s' %
                               (len(self._enabled), self.out_path))

    def _set_attrs(self, name, attrs):
        """Stream attrs now if the file exists, else when the gate opens."""
        if self.cap is not None:
            self.cap.update_stream_attrs(name, attrs)
        else:
            self._pending_attrs.setdefault(name, {}).update(attrs)

    # ---- config ---------------------------------------------------------
    def _output_path(self):
        out = self._p['output_file']
        if out:
            return os.path.abspath(out)
        base = self._p['dataset_path'] or os.getcwd()
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.abspath(os.path.join(base, 'capture_%s.h5' % stamp))

    def _mode(self, cfg):
        return cfg.get('mode') or self._p['capture_mode']

    def _root_attrs(self):
        return {
            'host': socket.gethostname(),
            'ros_node': self.get_fully_qualified_name(),
            'capture_mode': self._p['capture_mode'],
            'sample_rate_hz': float(self._p['sample_rate_hz']),
            'created_utc': datetime.datetime.now(
                datetime.timezone.utc).isoformat(),
            'config_streams': {n: {k: (list(v) if isinstance(v, (list, tuple))
                                       else v) for k, v in c.items()}
                               for n, c in self.cfgs.items()},
            'timestamps': 'stamp_header_ns = message header stamp; '
                          'stamp_recv_ns = host time.time_ns() on arrival',
        }

    def _qos(self, cfg, conv):
        rel = cfg.get('qos') or conv.default_qos
        if rel not in ('reliable', 'best_effort'):
            raise ValueError("qos must be 'reliable' or 'best_effort'")
        depth = int(cfg.get('qos_depth') or 10)
        if rel == 'reliable':
            depth = max(depth, 10)
        return rel, depth, QoSProfile(
            reliability=(QoSReliabilityPolicy.RELIABLE if rel == 'reliable'
                         else QoSReliabilityPolicy.BEST_EFFORT),
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=depth)

    def _add_stream(self, name, cfg):
        msg_type = normalize_type(cfg['type'])
        conv = get_converter(msg_type)
        rel, depth, qos = self._qos(cfg, conv)
        mode = self._mode(cfg)
        attrs = {'topic': cfg['topic'], 'msg_type': msg_type, 'qos': rel,
                 'qos_depth': depth, 'mode': mode}
        if cfg.get('legacy_folder'):
            attrs['legacy_folder'] = cfg['legacy_folder']
        keep = bool(cfg.get('meta_for')) or \
            msg_type.endswith('ADCDataCube') or msg_type.endswith('ADCChirpMeta')
        self._pending.append(
            (name, attrs, cfg.get('compression'), keep))
        if cfg.get('meta_for'):
            self.meta_pairs.append((name, cfg['meta_for']))
        self.seq[name] = 0
        self.dropped[name] = 0
        opts = {k: cfg[k] for k in ('fields', 'skip_nans') if k in cfg}
        sub = self.create_subscription(
            get_message(msg_type), cfg['topic'],
            lambda m, n=name, c=conv, o=opts, md=mode: self._on_msg(
                n, c, o, md, m), qos)
        self.subs.append(sub)
        if msg_type.endswith('ADCDataCube'):
            self._add_cfg_sub(name, cfg)
        self.get_logger().info('stream %s: %s [%s] qos=%s depth=%d mode=%s' %
                               (name, cfg['topic'], msg_type, rel, depth, mode))

    # ---- radar cfg (latched path published by ti_radar_connect) ---------
    def _add_cfg_sub(self, name, cfg):
        topic = cfg.get('config_topic') or \
            cfg['topic'].replace('adc_data_cube', 'radar_config_path')
        self._set_attrs(name, {'radar_cfg_status': 'not_received'})
        if topic == cfg['topic']:
            self.get_logger().warning(
                'stream %s: cannot derive the radar cfg topic from %r; set '
                'streams.%s.config_topic (radar_cfg_status stays '
                'not_received)' % (name, cfg['topic'], name))
            return
        # matches TIRadarConnectNode: RELIABLE + TRANSIENT_LOCAL, depth 5
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.cfg_streams[name] = topic
        self._set_attrs(name, {'radar_cfg_topic': topic})
        self.subs.append(self.create_subscription(
            String, topic, lambda m, n=name: self._on_cfg_path(n, m), qos))
        self.get_logger().info('stream %s: radar cfg path from %s' %
                               (name, topic))

    def _on_cfg_path(self, name, msg):
        path = str(msg.data)
        if not self._gate_open:   # latched: keep it until the file exists
            held = self._held_cfg.setdefault(name, [])
            if not held or held[-1] != path:
                held.append(path)
            return
        self._queue_cfg(name, path)

    def _queue_cfg(self, name, path):
        first = self._cfg_first.get(name)
        if first is not None and first == path:
            return
        if first is None:
            self._cfg_first[name] = path
        try:   # a one-off message; the queue is empty at startup
            self.q.put((_CFG, name, path), timeout=2.0)
        except queue.Full:
            self.get_logger().error(
                '%s: write queue full, radar cfg path %r not stored' %
                (name, path))

    def _store_cfg(self, name, path):
        """Writer thread: read the cfg file, store text/hash/metrics."""
        first = self._cfg_writer_path.get(name)
        if first is not None:
            if path != first:
                self.cap.update_stream_attrs(name, {'radar_cfg_changed': True})
                self.get_logger().warning(
                    '%s: radar cfg path changed during the capture (%s -> %s);'
                    ' keeping the first' % (name, first, path))
            return
        self._cfg_writer_path[name] = path
        attrs = {'radar_cfg_path': path, 'radar_cfg_changed': False}
        try:
            with open(path, 'rb') as f:
                raw = f.read()
            text = raw.decode('utf-8', errors='replace')
            attrs.update(
                radar_cfg_text=text,
                radar_cfg_sha256=hashlib.sha256(raw).hexdigest(),
                radar_cfg_metrics=json.dumps(
                    radar_cfg.parse_and_compute(text), sort_keys=True),
                radar_cfg_status='ok')
        except Exception as e:  # noqa: BLE001
            attrs['radar_cfg_status'] = 'unreadable: %s' % e
            self.get_logger().warning('%s: radar cfg %r unreadable: %s' %
                                      (name, path, e))
        self.cap.update_stream_attrs(name, attrs)

    # ---- receive --------------------------------------------------------
    def _on_msg(self, name, conv, opts, mode, msg):
        recv_ns = time.time_ns()
        if not self._gate_open:   # before the gate opens: note it, discard it
            self._seen.add(name)
            return
        if mode == 'sampled':
            self.seq[name] += 1
            self.latest[name] = (self.seq[name], msg, recv_ns, conv, opts)
            return
        self._enqueue(name, conv, opts, msg, recv_ns)

    def _sample_tick(self):
        for name, item in list(self.latest.items()):
            seq, msg, recv_ns, conv, opts = item
            if self.written_seq.get(name) == seq:
                continue
            self.written_seq[name] = seq
            self._enqueue(name, conv, opts, msg, recv_ns)

    def _enqueue(self, name, conv, opts, msg, recv_ns):
        try:
            sample = conv.fn(msg, opts)
            extra = {}
            if name not in self._described:
                self._described.add(name)
                extra = conv.describe(msg, opts)
        except Exception as e:  # noqa: BLE001 - log and keep recording
            self.get_logger().error('%s: convert failed: %s' % (name, e),
                                    throttle_duration_sec=5.0)
            self.dropped[name] += 1
            return
        try:
            self.q.put_nowait((name, sample, recv_ns, extra))
        except queue.Full:
            self.dropped[name] += 1
            self.get_logger().warning('%s: write queue full, sample dropped' %
                                      name, throttle_duration_sec=5.0)

    # ---- writer thread ----------------------------------------------------
    def _writer_loop(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            if item[0] is _CALL:
                try:
                    item[1]()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error('%s failed: %s' %
                                            (item[1].__name__, e))
                continue
            if item[0] is _CFG:
                try:
                    self._store_cfg(item[1], item[2])
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error('%s: radar cfg store failed: %s' %
                                            (item[1], e))
                continue
            name, s, recv_ns, extra = item
            try:
                if extra:
                    self.cap.update_stream_attrs(name, extra)
                self.cap.append(name, s.header_ns, recv_ns, s.fields, s.ragged)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error('%s: write failed: %s' % (name, e),
                                        throttle_duration_sec=5.0)

    def _duration_done(self):
        self.get_logger().info('duration_s reached; stopping')
        rclpy.try_shutdown()

    def finish(self):
        """Drain the queue, write capture metadata, close the file."""
        if self.cap is None:
            self.get_logger().info(
                'no file written: the start gate never opened')
            return
        self.q.put(None)
        self.writer.join(timeout=60.0)
        try:   # the final word on static transforms (writer is stopped)
            self._write_tf_static()
            self._write_urdf()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('final /tf_static write failed: %s' % e)
        for pair, name in self.tf_names.items():
            if self.cap.count(name) == 0:
                self.get_logger().warning(
                    'tf %s->%s: no transforms on %s (n_samples=0)' %
                    (pair[0], pair[1], self._resolved(self._p['tf_topic'])))
        for meta, cube in self.meta_pairs:
            res = self.cap.cross_check(meta, cube)
            if res:
                self.get_logger().info('%s vs %s: %s' % (cube, meta, res))
        for name in self.cfg_streams:
            if name not in self._cfg_writer_path:
                self.cap.update_stream_attrs(
                    name, {'radar_cfg_status': 'not_received'})
                self.get_logger().warning(
                    '%s: no radar cfg path received on %s' %
                    (name, self.cfg_streams[name]))
        for name, n in self.dropped.items():
            self.cap.update_stream_attrs(name, {'n_dropped_node': n})
        self.cap.close()
        self.get_logger().info('closed %s (%s)' % (self.out_path, ', '.join(
            '%s=%d' % (n, self.cap.count(n)) for n in self.cap.streams())))


def main(args=None):
    """Run the recorder; returns the process exit code (2 = gate abort)."""
    rclpy.init(args=args)
    node = DatasetRecorder()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Closing has begun: further ^C / SIGTERM must not interrupt the
        # drain or the final h5py attribute writes (closed_cleanly).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()
    return node.exit_code


if __name__ == '__main__':
    sys.exit(main())
