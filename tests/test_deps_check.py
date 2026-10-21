"""`scripts/deps/<driver>.sh --check`: no root, installs nothing, exit 0 or 1 + `missing:` on stderr.

A fake `dpkg-query` first on PATH stands in for the package database, so these prove
the scripts' logic, not what is installed on the machine running them."""
import os
import stat
import subprocess
from pathlib import Path

import pytest

DEPS = Path(__file__).resolve().parents[1] / "scripts" / "deps"
SCRIPTS = sorted(p.name for p in DEPS.glob("[a-z]*.sh"))
MARKERS = {"livox.sh": ("LIVOX_SDK_INSTALLED", "Livox-SDK2"),
           "realsense.sh": ("REALSENSE_INSTALLED", "librealsense")}

FAKE_DPKG = """#!/bin/sh
# usage: dpkg-query -W -f='${Status}' <pkg>; FAKE_MISSING is a space-separated list
pkg=""
for a in "$@"; do pkg="$a"; done
for m in $FAKE_MISSING; do
    if [ "$m" = "$pkg" ]; then exit 1; fi
done
printf 'install ok installed'
"""


def run(script, tmp_path, missing=(), marker_present=True):
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    dpkg = fake / "dpkg-query"
    dpkg.write_text(FAKE_DPKG)
    dpkg.chmod(dpkg.stat().st_mode | stat.S_IXUSR)
    marker = tmp_path / "marker.so"
    if marker_present:
        marker.write_text("")
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}", FAKE_MISSING=" ".join(missing),
               LIVOX_SDK_INSTALLED=str(marker), REALSENSE_INSTALLED=str(marker))
    return subprocess.run(["bash", str(DEPS / script), "--check"], env=env, text=True,
                          capture_output=True, timeout=30)


def first_package(script):
    """The first entry of the script's package array, with ${ROS_DISTRO} expanded."""
    text = (DEPS / script).read_text()
    name = "PREREQ=(" if "PREREQ=(" in text else "PACKAGES=("  # leapmotion lists its apt-repo prerequisites first
    start = text.index(name) + len(name)
    first = text[start:text.index(")", start)].split()[0].strip('"')
    return first.replace("${ROS_DISTRO}", "jazzy")


def test_every_script_is_covered():
    assert {"dataset.sh", "leapmotion.sh", "livox.sh", "ouster.sh", "radar.sh", "realsense.sh",
            "usb_cam.sh", "vicon.sh"} <= set(SCRIPTS)


@pytest.mark.parametrize("script", SCRIPTS)
def test_all_installed_exits_zero(script, tmp_path):
    r = run(script, tmp_path)
    assert r.returncode == 0, r.stderr
    assert "missing" not in r.stderr


@pytest.mark.parametrize("script", [s for s in SCRIPTS if s != "radar.sh"])
def test_one_missing_package_exits_one_and_names_it(script, tmp_path):
    pkg = first_package(script)
    assert pkg
    r = run(script, tmp_path, missing=[pkg])
    assert r.returncode == 1
    assert f"missing: {pkg}" in r.stderr


def test_radar_has_nothing_to_check(tmp_path):
    assert run("radar.sh", tmp_path, missing=["anything"]).returncode == 0


@pytest.mark.parametrize("script", sorted(MARKERS))
def test_absent_marker_exits_one(script, tmp_path):
    r = run(script, tmp_path, marker_present=False)
    assert r.returncode == 1
    assert f"missing: {MARKERS[script][1]}" in r.stderr
