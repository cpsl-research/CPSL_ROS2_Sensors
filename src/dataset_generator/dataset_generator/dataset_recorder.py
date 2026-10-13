"""dataset_recorder: record the streams listed in the YAML into one HDF5 file.

Replaces the per-sensor ``*_enable`` / ``*_topic`` parameters of the v1
``dataset_generator`` node with a ``streams:`` table (see configs/radar_sar.yaml
and docs/hdf5_format.md). Every sample is stored with its header stamp and the
host receive time.
"""
import datetime
import hashlib
import json
import os
import queue
import signal
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message

from dataset_generator import radar_cfg
from dataset_generator.converters import get_converter, normalize_type
from dataset_generator.hdf5_writer import CaptureFile
from dataset_generator.stream_config import parse_streams
from std_msgs.msg import String

_CFG = object()   # queue tag: (_CFG, stream, radar cfg path)


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
                ('queue_size', 256)):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            self._p[name] = self.get_parameter(name).value
        if self._p['capture_mode'] not in ('all', 'sampled'):
            raise ValueError("capture_mode must be 'all' or 'sampled'")

        raw = {k: v.value for k, v in
               self.get_parameters_by_prefix('streams').items()}
        self.cfgs = parse_streams(raw)
        self.out_path = self._output_path()
        os.makedirs(os.path.dirname(self.out_path) or '.', exist_ok=True)
        self.cap = CaptureFile(
            self.out_path, self._root_attrs(),
            flush_period_s=float(self._p['flush_period_s']))

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

        self.writer = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer.start()
        if any(self._mode(c) == 'sampled' for c in self.cfgs.values()):
            self.create_timer(1.0 / float(self._p['sample_rate_hz']),
                              self._sample_tick)
        if float(self._p['duration_s']) > 0:
            self._stop_timer = self.create_timer(
                float(self._p['duration_s']), self._duration_done)
        self.get_logger().info('recording %d stream(s) to %s' %
                               (len(self.subs), self.out_path))

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
        self.cap.add_stream(name, attrs, compression=cfg.get('compression'),
                            keep_stamps=keep)
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
        self.cap.update_stream_attrs(
            name, {'radar_cfg_status': 'not_received'})
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
        self.cap.update_stream_attrs(name, {'radar_cfg_topic': topic})
        self.subs.append(self.create_subscription(
            String, topic, lambda m, n=name: self._on_cfg_path(n, m), qos))
        self.get_logger().info('stream %s: radar cfg path from %s' %
                               (name, topic))

    def _on_cfg_path(self, name, msg):
        path = str(msg.data)
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
        self.q.put(None)
        self.writer.join(timeout=60.0)
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


if __name__ == '__main__':
    main()
