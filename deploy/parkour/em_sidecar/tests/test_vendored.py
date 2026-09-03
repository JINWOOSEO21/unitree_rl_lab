"""vendored 사본이 학습 원본과 여전히 같은지 확인한다.

`vendored/elevation_map_backend.py` 는 학습 저장소 파일을 글자 그대로 복사한 것이고,
헤더에 원본의 sha256 이 적혀 있다. 학습 쪽이 바뀌면 여기가 조용히 낡는다 —
"학습과 같은 지도"라는 전제가 무너지는데 아무 증상이 없다. 그래서 명시적으로 잰다.

학습 저장소가 없는 환경(젯슨 등)에서는 SKIP 한다. 그 경우에도 사본 자체가
헤더에 적힌 내용과 일관된지는 확인할 수 없으므로, 이 검사는 개발 머신 전용이다.

    python -m em_sidecar.tests.test_vendored [--parkour-repo <path>]
"""
from __future__ import annotations

import argparse
import hashlib
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
VENDORED = HERE.parent.parent / "vendored" / "elevation_map_backend.py"
REL_SRC = "parkour_isaaclab/envs/mdp/elevation_map_backend.py"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parkour-repo",
                    default=str(Path.home() / "workspace/codes/Isaaclab_Parkour"))
    a = ap.parse_args()

    text = VENDORED.read_text()
    m = re.search(r"sha256:\s*([0-9a-f]{64})", text)
    if not m:
        print("[RESULT] FAILED — vendored 헤더에 sha256 이 없다")
        return 1
    recorded = m.group(1)
    print(f"헤더에 기록된 원본 sha256: {recorded}")

    # 헤더를 뗀 나머지가 곧 원본이어야 한다 (헤더는 '# ===' 블록 하나뿐).
    body_start = text.index("\n", text.rindex("# " + "=" * 73)) + 1
    body = text[body_start:]
    body_sha = hashlib.sha256(body.encode()).hexdigest()
    print(f"사본 본문(헤더 제외) sha256:  {body_sha}")
    if body_sha != recorded:
        print("[RESULT] FAILED — 사본 본문이 헤더의 sha256 과 다르다 "
              "(사본을 손댔거나 헤더 경계가 어긋났다)")
        return 1

    src = Path(a.parkour_repo) / REL_SRC
    if not src.exists():
        print(f"\n원본 없음 ({src}) — 학습 저장소 대조는 SKIP")
        print("[RESULT] OK")
        return 0

    src_sha = hashlib.sha256(src.read_bytes()).hexdigest()
    print(f"학습 저장소 현재 sha256:      {src_sha}")
    if src_sha != recorded:
        print("\n[RESULT] FAILED — 학습 원본이 바뀌었다. vendored 사본과 헤더를 갱신할 것:")
        print(f"    (cd {a.parkour_repo} && sha256sum {REL_SRC})")
        return 1
    print("\n[RESULT] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
