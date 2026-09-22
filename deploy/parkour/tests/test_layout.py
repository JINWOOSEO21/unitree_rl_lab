"""배포 패키지 경계 검사.

- 런타임 패키지(go2_bridge, em_sidecar)는 eval/·terrain/ 을 import 하지 않는다.
  eval/ 은 지워도 로봇·시뮬레이터 동작에 영향이 없어야 한다.
- 런타임 패키지에 sys.path 조작이 없다 (진입점 __main__.py 만 예외).
- vendored/elevation_map_backend.py 본문은 헤더의 sha256 과 같다.

실행: cd deploy/parkour && python -m pytest tests/test_layout.py
"""

import hashlib
import re
from pathlib import Path

PARKOUR = Path(__file__).resolve().parents[1]
RUNTIME = ("go2_bridge", "em_sidecar")
FORBIDDEN = ("eval", "terrain", "tools")


def _py_files(pkg):
    return [p for p in (PARKOUR / pkg).rglob("*.py") if "__pycache__" not in p.parts]


def test_runtime_does_not_import_eval_or_terrain():
    pat = re.compile(r"^\s*(?:from|import)\s+(" + "|".join(FORBIDDEN) + r")(?:\.|\s|$)", re.M)
    bad = [(p.relative_to(PARKOUR), m.group(0).strip()) for pkg in RUNTIME for p in _py_files(pkg) for m in pat.finditer(p.read_text())]
    assert not bad, f"runtime imports non-runtime code: {bad}"


def test_runtime_has_no_sys_path_hacks():
    bad = [p.relative_to(PARKOUR) for pkg in RUNTIME for p in _py_files(pkg) if p.name != "__main__.py" and "sys.path.insert" in p.read_text()]
    assert not bad, f"sys.path hacks in runtime packages: {bad}"


def test_vendored_backend_matches_pinned_sha():
    src = PARKOUR / "vendored" / "elevation_map_backend.py"
    lines = src.read_text().splitlines(keepends=True)
    end = max(i for i, l in enumerate(lines) if l.startswith("# ====="))
    header, body = "".join(lines[: end + 1]), "".join(lines[end + 1 :])
    pinned = re.search(r"sha256:\s*([0-9a-f]{64})", header).group(1)
    assert hashlib.sha256(body.encode()).hexdigest() == pinned, "vendored body was modified (reformatted?)"
