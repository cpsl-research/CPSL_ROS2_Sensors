"""No tracked source or config file cites a harness task ID (area name, hyphen, two digits).

Code comments give the technical reason, or a commit hash or doc section.
Markdown, the harness, and vendored driver submodules are out of scope.
stdlib only.
"""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PATTERN = re.compile(
    r"\b(radar|core|gui|sensors|release|firmware|docs|rover|nav|create3|ugv|plumb|docker|tf|fwstd|fwdocs)-[0-9]{2}\b")
SKIP_PREFIXES = (
    ".friday/",
    "src/CPSL_ROS_livox_ros_driver2/", "src/CPSL_ROS2_LeapMotion/", "src/ros2-vicon-bridge/",
    "src/ouster-ros/", "src/CPSL_TI_Radar/", "submodules/leapc-python-bindings/",
)

# path -> reason it is allowed to match. Empty on purpose.
ALLOWLIST: dict = {}


def _tracked():
    out = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, check=True,
                         capture_output=True).stdout.decode()
    for rel in filter(None, out.split("\0")):
        if rel.endswith(".md") or rel.startswith(SKIP_PREFIXES):
            continue
        yield rel


def test_no_harness_task_ids():
    hits = []
    for rel in _tracked():
        if rel in ALLOWLIST:
            continue
        try:
            text = (ROOT / rel).read_bytes().decode("utf-8")
        except (UnicodeDecodeError, FileNotFoundError, IsADirectoryError):
            continue  # binary, deleted or a submodule path
        for n, line in enumerate(text.splitlines(), 1):
            if PATTERN.search(line):
                hits.append(f"{rel}:{n}: {line.strip()[:100]}")
    assert not hits, "harness task IDs in tracked source:\n" + "\n".join(hits)
