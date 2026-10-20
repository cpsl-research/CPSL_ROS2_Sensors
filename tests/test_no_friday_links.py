"""No tracked markdown (except CLAUDE.md) may link into .friday/ -- that tree is local harness state."""
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _tracked_md():
    out = subprocess.run(["git", "ls-files", "-z", "--", "*.md"], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    return [p for p in out.split("\0") if p and Path(p).name != "CLAUDE.md"]


def test_no_markdown_links_into_friday():
    bad = []
    for rel in _tracked_md():
        f = ROOT / rel
        if not f.is_file():
            continue
        for n, line in enumerate(f.read_text(errors="replace").splitlines(), 1):
            if ".friday/" in line:
                bad.append(f"{rel}:{n}")
    assert not bad, "markdown references .friday/: " + ", ".join(bad)
