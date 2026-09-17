"""Download Ubuntu 20.04 (focal) arm64 .deb files from ports.ubuntu.com and verify their SHA256.

Used to give the NVIDIA torch wheel its libopenblas.so.0 on a Jetson without sudo: the .deb files are
only unpacked under ~/walking/opt/syslibs (dpkg-deb -x), never installed into the system.
Usage: python fetch_focal_arm64_debs.py <out_dir> <package> [<package> ...]
"""
from __future__ import annotations

import gzip
import hashlib
import sys
import urllib.request
from pathlib import Path

BASE = "http://ports.ubuntu.com/ubuntu-ports"
# later entries win, so -updates/-security override the release pocket
POCKETS = [f"{suite}/{comp}" for suite in ("focal", "focal-security", "focal-updates")
           for comp in ("main", "universe")]


def index(pocket: str) -> dict[str, dict[str, str]]:
    url = f"{BASE}/dists/{pocket}/binary-arm64/Packages.gz"
    text = gzip.decompress(urllib.request.urlopen(url, timeout=60).read()).decode("utf-8", "replace")
    out = {}
    for block in text.split("\n\n"):
        fields = dict(line.split(": ", 1) for line in block.splitlines() if ": " in line and line[0] != " ")
        if "Package" in fields:
            out[fields["Package"]] = fields
    return out


out_dir = Path(sys.argv[1])
out_dir.mkdir(parents=True, exist_ok=True)
wanted = sys.argv[2:]
found: dict[str, dict[str, str]] = {}
for pocket in POCKETS:
    idx = index(pocket)
    for name in wanted:
        if name in idx:
            found[name] = dict(idx[name], pocket=pocket)

for name in wanted:
    if name not in found:
        raise SystemExit(f"package not found in focal arm64 indices: {name}")
    f = found[name]
    dest = out_dir / Path(f["Filename"]).name
    data = urllib.request.urlopen(f"{BASE}/{f['Filename']}", timeout=120).read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != f["SHA256"]:
        raise SystemExit(f"SHA256 mismatch for {dest.name}")
    dest.write_bytes(data)
    print(f"{name} {f['Version']} [{f['pocket']}] sha256 ok -> {dest.name}  depends: {f.get('Depends', '')}")
