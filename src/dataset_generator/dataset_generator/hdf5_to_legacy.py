"""Compatibility layer: HDF5 capture -> the pre-v2 folder-of-.npy layout.

    python3 -m dataset_generator.hdf5_to_legacy capture.h5 out_dir [--reference STREAM]

Old layout: ``<out_dir>/<folder>/frame_<idx>.npy`` (camera: PNG in v1; here
.npy too). Folder = the stream's ``legacy_folder`` attr, else its name.
Sample ``idx`` is a row of the *reference* stream (default: the first stream
in h5py iteration order, which is alphabetical, not the YAML order); for
every other stream the newest sample received at or before the reference
sample's receive time is used, mirroring v1's "latest message" semantics.
ADC cubes become complex arrays [rx, samples, chirps], as in v1.
"""
import argparse
import os

import h5py
import numpy as np


def _columns(grp):
    return [c.decode() if isinstance(c, bytes) else str(c)
            for c in grp.attrs.get('columns', [])]


def _sample(grp, i, offsets):
    d = grp['data']
    if 'row_count' in grp:
        return d[offsets[i]:offsets[i + 1]]
    x = d[i]
    if _columns(grp)[:2] == ['real', 'imag']:
        return x[0].astype(np.float64) + 1j * x[1].astype(np.float64)
    return x


def export(h5_path, out_dir, reference=None):
    with h5py.File(h5_path, 'r') as f:
        names = [n for n in f if n not in ('tf', 'tf_static')
                 and 'stamp_recv_ns' in f[n]
                 and f[n]['stamp_recv_ns'].shape[0] > 0
                 and 'data' in f[n]]
        if not names:
            raise SystemExit('no stream with data in %s' % h5_path)
        ref = reference or names[0]
        ref_t = f[ref]['stamp_recv_ns'][:]
        info = {}
        for n in names:
            g = f[n]
            off = None
            if 'row_count' in g:
                off = np.concatenate([[0], np.cumsum(g['row_count'][:])])
            info[n] = (g['stamp_recv_ns'][:], off)
        counts = {}
        for n in names:
            g = f[n]
            folder = g.attrs.get('legacy_folder', n)
            if isinstance(folder, bytes):
                folder = folder.decode()
            os.makedirs(os.path.join(out_dir, folder), exist_ok=True)
            t, off = info[n]
            for idx, rt in enumerate(ref_t):
                j = int(np.searchsorted(t, rt, side='right')) - 1
                if j < 0:
                    continue
                np.save(os.path.join(out_dir, folder, 'frame_%d.npy' % idx),
                        _sample(g, j, off))
                counts[n] = counts.get(n, 0) + 1
        return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('h5')
    ap.add_argument('out_dir')
    ap.add_argument('--reference', default=None)
    a = ap.parse_args()
    print(export(a.h5, a.out_dir, a.reference))


if __name__ == '__main__':
    main()
