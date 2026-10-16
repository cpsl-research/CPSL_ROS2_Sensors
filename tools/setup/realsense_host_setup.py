#!/usr/bin/env python3
"""Make this host let a user-space librealsense open an Intel RealSense camera and its IMU.

    python3 tools/setup/realsense_host_setup.py --json            # checks, read-only
    python3 tools/setup/realsense_host_setup.py                   # same, human readable
    python3 tools/setup/realsense_host_setup.py --apply --dry-run # print what --apply would run
    python3 tools/setup/realsense_host_setup.py --apply           # ask once, then run (sudo per command)

Three checks, each OK, MISSING, WARN or N-A (exit status 1 iff any is MISSING):

  rules   the udev rules file.  Installed from the copy vendored in ``rules/`` (no
          network at apply time), to /etc/udev/rules.d/99-realsense-libusb.rules.
  camera  RealSense USB devices on the bus (VID 8086, PIDs taken from the vendored rules).
  iio     the IMU: the camera's iio devices, and whether ``scan_elements/*_en`` can be
          written by a non-root user.  librealsense enables each IMU channel by writing
          those files and publishes no stream at all when it cannot.

Unlike the radar and Livox checks, these are valid inside a container too: sysfs and
/etc/udev/rules.d are the host's and are visible there.  Report mode may run as root.

``--apply`` never runs as root: it calls sudo per command, after one confirmation.  It
installs the vendored rules (first copying a differing file to
``99-realsense-libusb.rules.bak-<UTC stamp>``, a name udev ignores), runs
``udevadm control --reload`` and ``udevadm trigger --action=add --parent-match=<camera>``
for each camera (with no camera: reload only; plug it in afterwards), then re-checks.  It
never deletes anything.  A host that already has librealsense's own apt rules
(``*librealsense*`` in a udev rules directory) gets nothing installed.

Vendored rules: ``rules/99-realsense-libusb.rules`` is the unmodified file from
https://github.com/IntelRealSense/librealsense/blob/v2.58.4/config/99-realsense-libusb.rules
Copyright (c) Intel Corporation, licensed Apache-2.0 (the librealsense project licence).
Its sha256 is pinned below; a differing vendored copy is refused.

Python stdlib only.  All I/O sits behind ``Host`` so tests can use a fake.
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

RULES_VERSION = "v2.58.4"
RULES_URL = ("https://raw.githubusercontent.com/IntelRealSense/librealsense/"
             f"{RULES_VERSION}/config/99-realsense-libusb.rules")
RULES_SHA256 = "34c287000862710b60732aea942bda6f51faf7436b5b53f66fbfd021bc8c22dd"
RULES_NAME = "99-realsense-libusb.rules"
VENDORED = Path(__file__).resolve().parent / "rules" / RULES_NAME
RULES_DIR = "/etc/udev/rules.d"
APT_RULE_GLOBS = ("/lib/udev/rules.d/*librealsense*", "/usr/lib/udev/rules.d/*librealsense*",
                  "/etc/udev/rules.d/*librealsense*")
VENDOR_ID = "8086"
SCHEMA = 1

OK, MISSING, WARN, NA = "OK", "MISSING", "WARN", "N-A"


@dataclass
class Step:
    argv: list

    def display(self) -> str:
        return shlex.join(self.argv)


@dataclass
class Check:
    """``fix_cmds`` are what --apply runs; ``manual`` are only ever printed."""

    name: str
    status: str
    detail: str
    fix_cmds: list = field(default_factory=list)
    manual: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["fix_cmds"] = [s.display() for s in self.fix_cmds]
        return d


# ---------------------------------------------------------------------- host

class Host:
    """All I/O.  Read-only except ``execute`` (reached only from --apply)."""

    def euid(self) -> int:
        return os.geteuid()

    def utc_stamp(self) -> str:
        return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())

    def execute(self, step: Step) -> int:
        return subprocess.run(step.argv).returncode

    def read_bytes(self, path: str):
        try:
            return Path(path).read_bytes()
        except OSError:
            return None

    def glob(self, pattern: str) -> list:
        return sorted(glob.glob(pattern))

    def cameras(self, pids: set) -> list:
        """RealSense USB devices -> [{syspath, pid, product, serial}]."""
        out = []
        for d in sorted(Path("/sys/bus/usb/devices").glob("*")):
            def rd(name, d=d):
                try:
                    return (d / name).read_text().strip()
                except OSError:
                    return ""
            if rd("idVendor").lower() == VENDOR_ID and rd("idProduct").lower() in pids:
                out.append({"syspath": str(d.resolve()), "pid": rd("idProduct").lower(),
                            "product": rd("product"), "serial": rd("serial")})
        return out

    def iio_devices(self, syspath: str) -> list:
        """iio devices whose real sysfs path lies under the camera's."""
        out = []
        for d in sorted(Path("/sys/bus/iio/devices").glob("iio:device*")):
            real = str(d.resolve())
            if real.startswith(syspath.rstrip("/") + "/"):
                out.append(real)
        return out

    def en_files(self, dev: str) -> list:
        return sorted(str(p) for p in Path(dev, "scan_elements").glob("*_en"))

    def en_state(self, path: str) -> tuple:
        """(mode, writable by a non-root user).  As root os.access says yes to
        everything, so the mode bits decide (other-writable); otherwise os.access."""
        try:
            mode = os.stat(path).st_mode & 0o7777
        except OSError:
            return 0, False
        if os.geteuid() == 0:
            return mode, bool(mode & 0o002)
        return mode, os.access(path, os.W_OK)


# --------------------------------------------------------------------- rules

def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def vendored_pids(text: str) -> set:
    """PIDs of VID 8086 devices named in the rules."""
    return {m.group(1).lower() for ln in text.splitlines() if f'"{VENDOR_ID}"' in ln
            for m in [re.search(r'ATTRS\{idProduct\}=="([0-9a-fA-F]{4})"', ln)] if m}


def load_vendored(host: Host) -> tuple:
    """(bytes | None, problem | None).  Refuses a copy that differs from the pinned hash."""
    data = host.read_bytes(str(VENDORED))
    if data is None:
        return None, f"vendored rules missing: {VENDORED}"
    if sha256(data) != RULES_SHA256:
        return None, (f"vendored rules differ from the pinned {RULES_VERSION} sha256 "
                      f"(got {sha256(data)[:12]}, want {RULES_SHA256[:12]})")
    return data, None


def reload_trigger_steps(cams: list) -> list:
    steps = [Step(["sudo", "udevadm", "control", "--reload"])]
    steps += [Step(["sudo", "udevadm", "trigger", "--action=add", f"--parent-match={c['syspath']}"])
              for c in cams]
    return steps


def check_rules(host: Host, cams: list) -> Check:
    dest = f"{RULES_DIR}/{RULES_NAME}"
    data, problem = load_vendored(host)
    if data is None:
        return Check("rules", WARN, problem or "vendored rules unavailable",
                     manual=["git checkout -- tools/setup/rules   # in the CPSL_ROS2_Sensors repo"])
    installed = host.read_bytes(dest)
    notes = []
    if not cams:
        notes.append("no camera plugged in: --apply only reloads udev; plug the camera in afterwards")
    after = reload_trigger_steps(cams)
    install = Step(["sudo", "install", "-m", "0644", str(VENDORED), dest])
    if installed is not None and sha256(installed) == RULES_SHA256:
        return Check("rules", OK, f"{dest} matches the vendored librealsense {RULES_VERSION} rules")
    if installed is None:
        apt = [p for g in APT_RULE_GLOBS for p in host.glob(g)]
        if apt:
            return Check("rules", OK, f"librealsense's own rules are installed ({apt[0]}); "
                         f"nothing to install", notes=["not checked against the vendored copy"])
        return Check("rules", MISSING, f"{dest} is not installed",
                     fix_cmds=[install] + after, notes=notes)
    bak = f"{dest}.bak-{host.utc_stamp()}"
    return Check("rules", MISSING,
                 f"{dest} differs from the vendored {RULES_VERSION} rules (sha256 {sha256(installed)[:12]})",
                 fix_cmds=[Step(["sudo", "cp", "-p", dest, bak]), install] + after,
                 notes=[f"the existing file is first copied to {bak}"] + notes)


# -------------------------------------------------------------------- camera

def check_camera(cams: list) -> Check:
    if not cams:
        return Check("camera", NA, "no RealSense camera on the USB bus")
    desc = "; ".join(f"{c['product'] or 'RealSense'} {c['serial']} (pid {c['pid']}) at {c['syspath']}"
                     for c in cams)
    return Check("camera", OK, f"{len(cams)} camera(s): {desc}")


# ----------------------------------------------------------------------- iio

def check_iio(host: Host, cams: list) -> Check:
    if not cams:
        return Check("iio", NA, "no camera, so no IMU to check")
    devs = [(c, d) for c in cams for d in host.iio_devices(c["syspath"])]
    if not devs:
        return Check("iio", NA, "the camera has no iio device (no IMU, or the hid-sensor "
                     "kernel modules are not loaded); nothing to check")
    states, nofile = [], []
    for _c, d in devs:
        files = host.en_files(d)
        if not files:
            nofile.append(d)
        states += [(f, *host.en_state(f)) for f in files]
    if not states:
        return Check("iio", WARN, f"{len(devs)} iio device(s) but no scan_elements/*_en: {', '.join(nofile)}")
    bad = [(f, m) for f, m, w in states if not w]
    modes = ", ".join(sorted({f"{m:04o}" for _f, m, _w in states}))
    if not bad:
        return Check("iio", OK, f"{len(states)} scan_elements/*_en file(s) on {len(devs)} iio device(s) "
                     f"are writable by a non-root user (modes {modes})")
    return Check("iio", MISSING,
                 f"{len(bad)} of {len(states)} scan_elements/*_en file(s) are not writable by a non-root "
                 f"user, e.g. {bad[0][0]} mode {bad[0][1]:04o}; librealsense then publishes no stream "
                 f"with the IMU on",
                 fix_cmds=reload_trigger_steps(cams),
                 manual=["if it persists after --apply, unplug and replug the camera"])


# ---------------------------------------------------------------------- main

def run_checks(host: Host) -> list:
    data, _ = load_vendored(host)
    pids = vendored_pids(data.decode("utf-8", "replace")) if data else set()
    cams = host.cameras(pids)
    return [check_rules(host, cams), check_camera(cams), check_iio(host, cams)]


def exit_code(checks: list) -> int:
    return 1 if any(c.status == MISSING for c in checks) else 0


def plan(checks: list) -> list:
    """The steps --apply runs: the fixes of every MISSING check, de-duplicated, in order."""
    steps, seen = [], set()
    for c in checks:
        if c.status == MISSING:
            for s in c.fix_cmds:
                if s.display() not in seen:
                    seen.add(s.display())
                    steps.append(s)
    return steps


def format_check(c: Check) -> str:
    lines = [f"[{c.status:^7}] {c.name:<7} {c.detail}"]
    lines += ["           fix: " + s.display() for s in c.fix_cmds]
    lines += ["           run by hand: " + m for m in c.manual]
    lines += ["           note: " + n for n in c.notes]
    return "\n".join(lines)


def _ask_tty(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() in ("y", "yes")
    except EOFError:
        return False


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--json", action="store_true", help="machine-readable report (read-only)")
    ap.add_argument("--apply", action="store_true", help="install the udev rules and re-trigger (sudo per command)")
    ap.add_argument("--dry-run", action="store_true", help="with --apply: print the commands, run nothing")
    return ap


def main(argv=None, host: Host | None = None, ask=_ask_tty, out=None) -> int:
    host = host or Host()
    out = out or sys.stdout
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.dry_run and not args.apply:
        ap.error("--dry-run goes with --apply")
    if args.json and args.apply and not args.dry_run:
        ap.error("--json reports only; combine it with --apply only together with --dry-run")
    if host.euid() == 0 and args.apply:
        print("realsense_host_setup: do not run --apply as root or with sudo; it calls sudo itself "
              "(report mode may run as root)", file=sys.stderr)
        return 2
    checks = run_checks(host)
    steps = plan(checks) if args.apply else []
    if args.json:
        rep = {"schema": SCHEMA, "checks": [c.to_dict() for c in checks], "exit": exit_code(checks)}
        if args.apply:
            rep["would_run"] = [s.display() for s in steps]
        json.dump(rep, out, indent=2)
        out.write("\n")
        return exit_code(checks)
    for c in checks:
        print(format_check(c), file=out)
    if not args.apply:
        return exit_code(checks)
    if not steps:
        print("\n--apply: nothing to do", file=out)
        return exit_code(checks)
    print("\n" + ("would run (nothing executed):" if args.dry_run else "will run:"), file=out)
    for s in steps:
        print("  " + s.display(), file=out)
    if args.dry_run:
        return exit_code(checks)
    if not ask("Run these now? [y/N] "):
        print("skipped", file=out)
        return 1
    for s in steps:
        rc = host.execute(s)
        if rc != 0:
            print(f"command failed (exit {rc}): {s.display()}", file=out)
            return 1
    again = run_checks(host)
    print("\nre-check:", file=out)
    for c in again:
        print(format_check(c), file=out)
    if any(c.name == "iio" and c.status == MISSING for c in again) or \
            any(c.name == "camera" and c.status == NA for c in again):
        print("replug the camera if the IMU check is not OK yet (udev applies rules on add)", file=out)
    return exit_code(again)


if __name__ == "__main__":
    sys.exit(main())
