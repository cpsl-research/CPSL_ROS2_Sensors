"""Hardware-free tests for tools/setup/realsense_host_setup.py (stdlib unittest; pytest collects them too).

A fake Host stands in for sysfs, /etc/udev/rules.d and sudo, so these prove the logic and
the pinned vendored file, not a real camera."""
import hashlib
import io
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/setup"))
import realsense_host_setup as rh  # noqa: E402

VENDORED_BYTES = rh.VENDORED.read_bytes()
DEST = f"{rh.RULES_DIR}/{rh.RULES_NAME}"
CAM = {"syspath": "/sys/devices/usb2/2-3", "pid": "0b3a", "product": "D435I", "serial": "123"}
DEV = "/sys/devices/usb2/2-3/2-3:1.5/iio:device0"
EN = [f"{DEV}/scan_elements/in_accel_x_en", f"{DEV}/scan_elements/in_anglvel_x_en"]


class FakeHost(rh.Host):
    def __init__(self, installed=None, apt=(), cams=(CAM,), iio=(DEV,), en_modes=(0o666, 0o666),
                 euid=1000, vendored=VENDORED_BYTES):
        self.installed, self.apt, self.cams, self.iio = installed, list(apt), list(cams), list(iio)
        self.en_modes, self._euid, self.vendored = en_modes, euid, vendored
        self.executed, self.pids_seen = [], None

    def euid(self):
        return self._euid

    def utc_stamp(self):
        return "20261016T000000Z"

    def read_bytes(self, path):
        if path == str(rh.VENDORED):
            return self.vendored
        return self.installed if path == DEST else None

    def glob(self, pattern):
        return list(self.apt) if pattern.startswith("/lib/") else []

    def cameras(self, pids):
        self.pids_seen = pids
        return list(self.cams)

    def iio_devices(self, syspath):
        return list(self.iio)

    def en_files(self, dev):
        return list(EN)

    def en_state(self, path):
        m = self.en_modes[EN.index(path)]
        return m, bool(m & 0o002)

    def execute(self, step):
        self.executed.append(step.argv)
        return 0


def checks(host):
    return {c.name: c for c in rh.run_checks(host)}


def run_main(host, *argv, ask=lambda p: True):
    out = io.StringIO()
    rc = rh.main(list(argv), host=host, ask=ask, out=out)
    return rc, out.getvalue()


class Vendored(unittest.TestCase):
    def test_hash_equals_constant(self):
        self.assertEqual(hashlib.sha256(VENDORED_BYTES).hexdigest(), rh.RULES_SHA256)

    def test_pids_come_from_the_rules(self):
        pids = rh.vendored_pids(VENDORED_BYTES.decode())
        self.assertIn("0b3a", pids)
        self.assertGreater(len(pids), 10)

    def test_corrupt_vendored_copy_is_refused(self):
        c = checks(FakeHost(vendored=VENDORED_BYTES + b"# x\n"))["rules"]
        self.assertEqual(c.status, rh.WARN)
        self.assertEqual(c.fix_cmds, [])


class Rules(unittest.TestCase):
    def test_missing(self):
        c = checks(FakeHost())["rules"]
        self.assertEqual(c.status, rh.MISSING)
        argvs = [s.argv for s in c.fix_cmds]
        self.assertEqual(argvs[0], ["sudo", "install", "-m", "0644", str(rh.VENDORED), DEST])
        self.assertIn(["sudo", "udevadm", "control", "--reload"], argvs)
        self.assertIn(["sudo", "udevadm", "trigger", "--action=add",
                       f"--parent-match={CAM['syspath']}"], argvs)

    def test_matching_is_ok_and_apply_runs_nothing(self):
        host = FakeHost(installed=VENDORED_BYTES)
        self.assertEqual(checks(host)["rules"].status, rh.OK)
        rc, out = run_main(host, "--apply")
        self.assertEqual((rc, host.executed), (0, []))
        self.assertIn("nothing to do", out)

    def test_differing_backs_up_first(self):
        c = checks(FakeHost(installed=b"old\n"))["rules"]
        self.assertEqual(c.status, rh.MISSING)
        first = c.fix_cmds[0].argv
        self.assertEqual(first, ["sudo", "cp", "-p", DEST, DEST + ".bak-20261016T000000Z"])
        self.assertEqual(c.fix_cmds[1].argv[:3], ["sudo", "install", "-m"])
        self.assertFalse(first[-1].endswith(".rules"))   # udev ignores it

    def test_apt_rules_are_ok_with_a_note_and_install_nothing(self):
        c = checks(FakeHost(apt=["/lib/udev/rules.d/60-librealsense2-udev-rules.rules"]))["rules"]
        self.assertEqual(c.status, rh.OK)
        self.assertEqual(c.fix_cmds, [])
        self.assertTrue(c.notes)

    def test_no_camera_reloads_only(self):
        c = checks(FakeHost(cams=()))["rules"]
        self.assertEqual([s.argv for s in c.fix_cmds[1:]], [["sudo", "udevadm", "control", "--reload"]])
        self.assertTrue(any("plug the camera in" in n for n in c.notes))


class CameraAndIio(unittest.TestCase):
    def test_no_camera_is_na(self):
        c = checks(FakeHost(cams=()))
        self.assertEqual((c["camera"].status, c["iio"].status), (rh.NA, rh.NA))

    def test_camera_found_with_pids_from_rules(self):
        host = FakeHost()
        self.assertEqual(checks(host)["camera"].status, rh.OK)
        self.assertIn("0b3a", host.pids_seen)

    def test_no_iio_devices_is_na_not_ok(self):
        self.assertEqual(checks(FakeHost(iio=()))["iio"].status, rh.NA)

    def test_iio_unwritable_is_missing_with_trigger_fix(self):
        c = checks(FakeHost(en_modes=(0o644, 0o644)))["iio"]
        self.assertEqual(c.status, rh.MISSING)
        self.assertIn("0644", c.detail)
        self.assertTrue(any("udevadm trigger" in s.display() for s in c.fix_cmds))

    def test_iio_partly_unwritable_is_missing(self):
        self.assertEqual(checks(FakeHost(en_modes=(0o666, 0o644)))["iio"].status, rh.MISSING)

    def test_iio_writable_is_ok(self):
        self.assertEqual(checks(FakeHost())["iio"].status, rh.OK)


class Modes(unittest.TestCase):
    def test_root_report_mode_works_and_exit_is_1_iff_missing(self):
        rc, out = run_main(FakeHost(euid=0, installed=VENDORED_BYTES))
        self.assertEqual(rc, 0)
        self.assertIn("rules", out)
        rc, _ = run_main(FakeHost(euid=0))
        self.assertEqual(rc, 1)

    def test_apply_refused_as_root(self):
        host = FakeHost(euid=0)
        rc, _ = run_main(host, "--apply")
        self.assertEqual(rc, 2)
        self.assertEqual(host.executed, [])

    def test_json_is_read_only_and_parses(self):
        host = FakeHost()
        rc, out = run_main(host, "--json")
        d = json.loads(out)
        self.assertEqual([c["name"] for c in d["checks"]], ["rules", "camera", "iio"])
        self.assertEqual((rc, d["exit"], host.executed), (1, 1, []))

    def test_dry_run_executes_nothing_and_dedupes(self):
        host = FakeHost(en_modes=(0o644, 0o644))
        run_main(host, "--apply", "--dry-run")
        self.assertEqual(host.executed, [])
        _, out = run_main(host, "--apply", "--dry-run")
        plan = out.split("would run")[1]
        self.assertEqual(plan.count("udevadm control --reload"), 1)   # rules + iio fixes merged
        self.assertEqual(plan.count("udevadm trigger"), 1)

    def test_apply_runs_steps_after_one_confirmation(self):
        host = FakeHost()
        asked = []
        rc, out = run_main(host, "--apply", ask=lambda p: asked.append(p) or True)
        self.assertEqual(len(asked), 1)
        self.assertEqual(host.executed[0][:3], ["sudo", "install", "-m"])
        self.assertIn("re-check", out)
        self.assertFalse(any(a[:2] == ["sudo", "rm"] for a in host.executed))

    def test_apply_declined_runs_nothing(self):
        host = FakeHost()
        rc, _ = run_main(host, "--apply", ask=lambda p: False)
        self.assertEqual((rc, host.executed), (1, []))

    def test_json_with_real_apply_is_rejected(self):
        with self.assertRaises(SystemExit):
            run_main(FakeHost(), "--json", "--apply")


if __name__ == "__main__":
    unittest.main()
