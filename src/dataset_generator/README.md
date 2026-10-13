# dataset_generator

Records sensor streams. Two recorders, chosen by `record_dataset.launch.py` from the YAML:

* **v2 `dataset_recorder`** - YAML with a `streams:` table (`configs/ugv_dataset.yaml`,
  `configs/radar_sar.yaml`). One HDF5 file per capture, per-message header + receive stamps.
  Format, YAML schema, QoS and the old-format converter: [docs/hdf5_format.md](docs/hdf5_format.md).
  The recorder waits for every enabled stream before it writes (`start_timeout_s`, `allow_missing`;
  abort = no file, exit code 2) and, since format v4, records `/tf_static` and the `/tf` pairs
  listed in `tf_track`, so sensor-frame data can be moved into the body or odom frame.
* **v1 `dataset_generator`** - the per-sensor `*_enable` / `*_topic` YAMLs
  (`ugv_dataset_v1.yaml`, `uav_*.yaml`, `human_movement.yaml`); these are unchanged v1 configs
  and still write folders of `.npy` files on a timer.

Needs `python3-h5py` for v2. Tests: `cd src/dataset_generator && PYTHONPATH=. python3 -m pytest test/test_hdf5_writer.py`
(skipped when h5py is missing).
