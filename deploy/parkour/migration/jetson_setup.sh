#!/usr/bin/env bash
# Go2 Jetson bridge 환경 설치 (단계 B-2, B-3). 오프라인 번들(make_jetson_bundle.sh 산출물) 사용.
# sudo 없음, 시스템 Python / /usr/local / 기존 서비스 변경 없음, DDS 송신 없음.
# 모든 산출물은 $ROOT (기본 ~/walking) 아래에만 생긴다. 지우려면 그 폴더만 지우면 된다.
#
#   cd ~/walking/go2_jetson_bundle
#   bash jetson_setup.sh check    # 번들 무결성 + 빌드 전제 조건 점검 (아무것도 설치하지 않음)
#   bash jetson_setup.sh unpack   # 소스 압축 해제
#   bash jetson_setup.sh venv     # Python 3.8 venv + pip 부트스트랩
#   bash jetson_setup.sh dds      # CycloneDDS C 0.10.2 를 $ROOT/opt 에 빌드 + python 바인딩
#   bash jetson_setup.sh pkgs     # torch / cupy / numpy / scipy ... + unitree_sdk2py
#   bash jetson_setup.sh smoke    # GPU 단계별 스모크 테스트 (DDS 미사용)
#   bash jetson_setup.sh all      # unpack -> venv -> dds -> pkgs -> smoke
set -euo pipefail

BUNDLE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-$HOME/walking}"
VENV="$ROOT/venv"
DDS_PREFIX="$ROOT/opt/cyclonedds"
SRC="$ROOT/src"
LOG="$ROOT/logs"; mkdir -p "$LOG"
PIP=("$VENV/bin/python" -m pip install --no-index --find-links "$BUNDLE/wheels")

write_env() {
  cat > "$ROOT/env.sh" <<EOF
# source ~/walking/env.sh  -- bridge 실행 전 매번 적용
source "$VENV/bin/activate"
export CYCLONEDDS_HOME="$DDS_PREFIX"
export LD_LIBRARY_PATH="$DDS_PREFIX/lib:/usr/local/cuda/lib64\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export CUDA_PATH=/usr/local/cuda
export PATH="/usr/local/cuda/bin:\$PATH"
export GO2_PARKOUR="$SRC/parkour"
export GO2_EMCUPY="$SRC/elevation_mapping_cupy"
EOF
}

do_check() {
  local bad=0
  ( cd "$BUNDLE" && sha256sum --quiet -c SHA256SUMS ) && echo "ok   bundle sha256" || { echo "FAIL bundle sha256"; bad=1; }
  python3.8 -c 'import venv, ensurepip' 2>/dev/null && echo "ok   python3.8 venv" || { echo "FAIL python3.8 venv (python3.8-venv 필요)"; bad=1; }
  [ -f "$(python3.8 -c 'import sysconfig; print(sysconfig.get_paths()["include"])')/Python.h" ] \
    && echo "ok   Python.h (cyclonedds 바인딩 빌드용)" || { echo "FAIL Python.h 없음 (python3.8-dev 필요)"; bad=1; }
  for t in cmake gcc g++ make; do command -v $t >/dev/null && echo "ok   $t" || { echo "FAIL $t"; bad=1; }; done
  [ -x /usr/local/cuda/bin/nvcc ] && echo "ok   nvcc $(/usr/local/cuda/bin/nvcc --version | grep -o 'release [0-9.]*')" || { echo "FAIL nvcc"; bad=1; }
  ldconfig -p | grep -q libopenblas && echo "ok   libopenblas (torch 런타임)" || { echo "FAIL libopenblas 없음 (sudo apt install libopenblas-dev 필요)"; bad=1; }
  ldconfig -p | grep -q 'libcudnn.so.8' && echo "ok   libcudnn 8" || echo "warn libcudnn 8 not in ldconfig"
  echo "info free disk: $(df -h --output=avail "$ROOT" | tail -1)   date: $(date -Is) (1970년이어도 오프라인 설치에는 영향 없음)"
  return $bad
}

do_unpack() {
  mkdir -p "$SRC"
  for a in cyclonedds-0.10.2 unitree_sdk2_python elevation_mapping_cupy parkour; do
    tar -xzf "$BUNDLE/src/$a.tar.gz" -C "$SRC"
  done
  ls "$SRC"
}

do_venv() {
  [ -x "$VENV/bin/python" ] || python3.8 -m venv "$VENV"
  "${PIP[@]}" --upgrade pip setuptools wheel "Cython<3"
  write_env
  "$VENV/bin/python" -m pip --version
}

do_dds() {
  cmake -S "$SRC/cyclonedds-0.10.2" -B "$SRC/cyclonedds-0.10.2/build" \
    -DCMAKE_INSTALL_PREFIX="$DDS_PREFIX" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SHM=OFF > "$LOG/cyclonedds_configure.log" 2>&1
  cmake --build "$SRC/cyclonedds-0.10.2/build" --target install -j"$(nproc)" > "$LOG/cyclonedds_build.log" 2>&1
  ls "$DDS_PREFIX/lib" | grep ddsc
  CYCLONEDDS_HOME="$DDS_PREFIX" "${PIP[@]}" --no-build-isolation "$BUNDLE/wheels/cyclonedds-0.10.2.tar.gz"
}

do_pkgs() {
  "${PIP[@]}" numpy==1.24.4 scipy==1.10.1
  "${PIP[@]}" "$BUNDLE"/wheels/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
  "${PIP[@]}" cupy-cuda11x==12.3.0 shapely simple-parsing ruamel.yaml pyyaml
  "${PIP[@]}" --no-deps --no-build-isolation -e "$SRC/unitree_sdk2_python"
  "$VENV/bin/python" -m pip list 2>/dev/null | grep -iE "^(torch|cupy|numpy|scipy|cyclonedds|unitree|shapely|simple|ruamel|PyYAML) "
}

do_smoke() {
  # shellcheck disable=SC1091
  source "$ROOT/env.sh"
  local out="$LOG/gpu_smoke_$(hostname)_$(date +%Y%m%d_%H%M%S).log"
  ( timeout 8 tegrastats --interval 1000 > "$out.tegrastats" 2>&1 & )
  python "$GO2_PARKOUR/migration/gpu_smoke.py" "$GO2_PARKOUR" "$GO2_EMCUPY" 200 2>&1 | tee "$out"
  echo "log: $out"
}

case "${1:-}" in
  check) do_check ;;
  unpack) do_unpack ;;
  venv) do_venv ;;
  dds) do_dds ;;
  pkgs) do_pkgs ;;
  smoke) do_smoke ;;
  all) do_check; do_unpack; do_venv; do_dds; do_pkgs; do_smoke ;;
  *) sed -n '2,14p' "$0"; exit 2 ;;
esac
