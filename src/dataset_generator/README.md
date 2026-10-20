# dataset_generator

Records sensor streams. One recorder, `dataset_recorder`, launched by `record_dataset.launch.py` from a YAML with a `streams:` table:

* **`dataset_recorder`** - every file in `configs/` (e.g. `ugv_dataset.yaml`, `radar_sar.yaml`, `uav_dataset.yaml`). One HDF5 file per capture, per-message header + receive stamps.
  Format, YAML schema, QoS and the old-format converter: [docs/hdf5_format.md](docs/hdf5_format.md).
  The recorder waits for every enabled stream before it writes (`start_timeout_s`, `allow_missing`;
  abort = no file, exit code 2) and, since format v4, records `/tf_static` and the `/tf` pairs
  listed in `tf_track`, so sensor-frame data can be moved into the body or odom frame.

The old v1 per-sensor recorder (`.npy` folders) is removed; `hdf5_to_legacy` still converts a capture to that folder layout.

Needs `python3-h5py` for v2. Tests: `cd src/dataset_generator && PYTHONPATH=. python3 -m pytest test/test_hdf5_writer.py`
(skipped when h5py is missing).
