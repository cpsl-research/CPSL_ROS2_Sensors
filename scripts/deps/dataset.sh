#!/usr/bin/env bash
# dataset_generator (the HDF5 recorder, src/dataset_generator): h5py. Not a
# driver, always built, so install.sh and the Dockerfile run it unconditionally;
# console's rosdep step also installs it (package.xml: exec_depend python3-h5py).
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure python3-h5py
