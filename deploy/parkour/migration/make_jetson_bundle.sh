#!/usr/bin/env bash
# 데스크톱(x86_64, 인터넷 가능)에서 Jetson(JetPack 5.1.1, Python 3.8, aarch64)용 오프라인 설치 번들을 만든다.
# Jetson 의 인터넷/시계(1970년, HTTPS 인증서 검증 실패 가능) 상태와 무관하게 설치하기 위함이다.
# 버전 근거는 README 의 "Jetson 패키지 버전" 절 참고. 로봇/Jetson 에는 아무것도 보내지 않는다.
#
#   bash make_jetson_bundle.sh <pip 가 있는 python> [출력 디렉터리]
set -euo pipefail

PY="${1:?usage: make_jetson_bundle.sh <python-with-pip> [out_dir]}"
OUT="${2:-$HOME/workspace/codes/go2_jetson_bundle}"
CODES="${CODES:-$HOME/workspace/codes}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARKOUR="$(cd "$HERE/.." && pwd)"

TORCH_WHL="torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/$TORCH_WHL"
CYCLONE_C_URL="https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.10.2.tar.gz"
CYCLONE_PY_VER="0.10.2"

mkdir -p "$OUT/wheels" "$OUT/src"
dl() {  # cp38 / aarch64 바이너리 wheel 만 받는다. 없으면 실패한다 (= 호환 wheel 부재를 여기서 발견).
  "$PY" -m pip download --quiet --only-binary=:all: --dest "$OUT/wheels" \
    --python-version 38 --implementation cp --abi cp38 \
    --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 --platform linux_aarch64 "$@"
}

echo "[1/6] venv 부트스트랩용 pip/setuptools/wheel (시스템 pip 20.0.2 는 manylinux_2_17 태그를 모른다)"
dl "pip<25.1" "setuptools<76" wheel "Cython<3"

echo "[2/6] NVIDIA PyTorch wheel (JetPack 5.1.1 공식, ~170MB)"
[ -s "$OUT/wheels/$TORCH_WHL" ] || curl -fL --retry 3 -o "$OUT/wheels/$TORCH_WHL" "$TORCH_URL"

echo "[3/6] 수치/매핑 의존성 (bridge 매핑 경로가 실제로 로드하는 것만)"
dl numpy==1.24.4 scipy==1.10.1 cupy-cuda11x==12.3.0 "shapely>=2.0,<2.1" \
   simple-parsing ruamel.yaml pyyaml
# torch 2.0.0 런타임 의존성
dl filelock typing-extensions sympy "networkx<3.2" jinja2

echo "[4/6] cyclonedds python 바인딩 sdist + 그 의존성 (aarch64 wheel 없음 -> Jetson 에서 빌드)"
SDIST_URL=$("$PY" - "$CYCLONE_PY_VER" <<'EOF'
import json, sys, urllib.request
d = json.load(urllib.request.urlopen(f"https://pypi.org/pypi/cyclonedds/{sys.argv[1]}/json"))
print(next(f["url"] for f in d["urls"] if f["packagetype"] == "sdist"))
EOF
)
curl -fL --retry 3 -o "$OUT/wheels/cyclonedds-$CYCLONE_PY_VER.tar.gz" "$SDIST_URL"
dl rich-click

echo "[5/6] 소스: CycloneDDS C 0.10.2, unitree_sdk2_python, elevation_mapping_cupy, deploy/parkour"
curl -fL --retry 3 -o "$OUT/src/cyclonedds-0.10.2.tar.gz" "$CYCLONE_C_URL"
tar -C "$CODES" --exclude=.git --exclude=.omc --exclude=__pycache__ --exclude=build \
    -czf "$OUT/src/unitree_sdk2_python.tar.gz" unitree_sdk2_python
tar -C "$CODES/Isaaclab_Parkour" --exclude=.git --exclude=__pycache__ --exclude=docs \
    -czf "$OUT/src/elevation_mapping_cupy.tar.gz" elevation_mapping_cupy
# captures(수 GB 로그), videos, mujoco 자산은 bridge 실행에 불필요
tar -C "$PARKOUR/.." --exclude=captures --exclude=videos --exclude=mujoco --exclude=.omc \
    --exclude=__pycache__ --exclude=.pytest_cache -czf "$OUT/src/parkour.tar.gz" parkour

echo "[6/6] manifest"
cp "$HERE/jetson_setup.sh" "$OUT/" 2>/dev/null || true
( cd "$OUT" && find wheels src -type f | sort | xargs sha256sum > SHA256SUMS )
{
  echo "created=$(date -Is) host=$(hostname)"
  echo "target=JetPack 5.1.1 / L4T R35.3.1 / CUDA 11.4 / Python 3.8 / aarch64"
  echo "unitree_rl_lab=$(git -C "$PARKOUR" rev-parse --short HEAD 2>/dev/null) dirty=$(git -C "$PARKOUR" status --short 2>/dev/null | wc -l)"
  ls "$OUT/wheels" | sed 's/^/wheel /'
} > "$OUT/MANIFEST.txt"
du -sh "$OUT"; echo "bundle: $OUT"
