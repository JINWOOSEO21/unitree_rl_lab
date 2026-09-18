#!/usr/bin/env bash
# Go2 Jetson bridge 환경 설치. 오프라인 번들(make_jetson_bundle.sh 산출물) 사용.
# sudo 없음, 시스템 Python / /usr/local / 기존 서비스 변경 없음, DDS 송신 없음.
# 모든 산출물은 $ROOT (기본 ~/walking) 아래에만 생긴다. 지우려면 그 폴더만 지우면 된다.
#
#   cd ~/walking/go2_jetson_bundle
#   bash jetson_setup.sh check    # 번들 무결성 + 빌드 전제 조건 점검 (아무것도 설치하지 않음)
#   bash jetson_setup.sh unpack   # 소스 압축 해제
#   bash jetson_setup.sh syslibs  # libopenblas 등 .deb 를 $ROOT/opt/syslibs 에 풀기 (시스템 설치 아님)
#   bash jetson_setup.sh venv     # Python 3.8 venv + pip 부트스트랩 (ensurepip 불필요)
#   bash jetson_setup.sh dds      # CycloneDDS C 0.10.2 를 $ROOT/opt 에 빌드 + python 바인딩
#   bash jetson_setup.sh pkgs     # torch / cupy / numpy / scipy ... + unitree_sdk2py
#   bash jetson_setup.sh smoke    # GPU 단계별 스모크 테스트 (DDS 미사용)
#   bash jetson_setup.sh all      # check -> unpack -> syslibs -> venv -> dds -> pkgs -> smoke
#   bash jetson_setup.sh report   # 설치/스모크 결과를 한 파일로 요약 (읽기 전용, 전달용)
set -euo pipefail

BUNDLE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-$HOME/walking}"
VENV="$ROOT/venv"
DDS_PREFIX="$ROOT/opt/cyclonedds"
SRC="$ROOT/src"
LOG="$ROOT/logs"; mkdir -p "$LOG"
PIP=("$VENV/bin/python" -m pip install --no-index --find-links "$BUNDLE/wheels")
# pipefail 아래에서 `ldconfig -p | grep -q` 는 grep 이 먼저 끝나면 ldconfig 가 SIGPIPE 로 죽어 오탐(false negative)이 난다.
# 출력을 한 번만 받아 두고 here-string 으로 검사한다.
LDCACHE="$(ldconfig -p 2>/dev/null || true)"
has_syslib() { grep -q -- "$1" <<<"$LDCACHE"; }

write_env() {
  cat > "$ROOT/env.sh" <<EOF
# source ~/walking/env.sh  -- bridge 실행 전 매번 적용
source "$VENV/bin/activate"
export CYCLONEDDS_HOME="$DDS_PREFIX"
export LD_LIBRARY_PATH="$DDS_PREFIX/lib:$ROOT/opt/syslibs/lib:/usr/local/cuda/lib64\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export CUDA_PATH=/usr/local/cuda
export PATH="/usr/local/cuda/bin:\$PATH"
export GO2_PARKOUR="$SRC/parkour"
export GO2_EMCUPY="$SRC/elevation_mapping_cupy"
EOF
}

do_check() {
  local bad=0
  ( cd "$BUNDLE" && sha256sum --quiet -c SHA256SUMS ) && echo "ok   bundle sha256" || { echo "FAIL bundle sha256"; bad=1; }
  # ensurepip(python3.8-venv) 이 없어도 된다: --without-pip 로 만들고 번들의 pip wheel 로 부트스트랩한다.
  python3.8 -c 'import venv' 2>/dev/null && ls "$BUNDLE"/wheels/pip-*.whl >/dev/null 2>&1 \
    && echo "ok   python3.8 venv (--without-pip + 번들 pip wheel)" || { echo "FAIL python3.8 venv 모듈 또는 번들 pip wheel 없음"; bad=1; }
  [ -f "$(python3.8 -c 'import sysconfig; print(sysconfig.get_paths()["include"])')/Python.h" ] \
    && echo "ok   Python.h (cyclonedds 바인딩 빌드용)" || { echo "FAIL Python.h 없음 (python3.8-dev 필요)"; bad=1; }
  for t in cmake gcc g++ make; do command -v $t >/dev/null && echo "ok   $t" || { echo "FAIL $t"; bad=1; }; done
  [ -x /usr/local/cuda/bin/nvcc ] && echo "ok   nvcc $(/usr/local/cuda/bin/nvcc --version | grep -o 'release [0-9.]*')" || { echo "FAIL nvcc"; bad=1; }
  # torch wheel 의 DT_NEEDED 중 JetPack 기본 설치에 없을 수 있는 것. 시스템에 없으면 번들 .deb 를 풀어서 쓴다.
  for lib in libopenblas.so.0 libgfortran.so.5 libnuma.so.1; do
    if has_syslib "$lib"; then echo "ok   $lib (system)"
    elif command -v dpkg-deb >/dev/null && ls "$BUNDLE"/debs/*.deb >/dev/null 2>&1; then echo "ok   $lib (system 에 없음 -> 번들 .deb 를 $ROOT/opt/syslibs 에 풀어 사용, sudo 불필요)"
    else echo "FAIL $lib 없음, 번들 debs/ 도 없음"; bad=1; fi
  done
  has_syslib "libcudnn.so.8" && echo "ok   libcudnn 8" || echo "warn libcudnn 8 not in ldconfig"
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

do_syslibs() {
  # .deb 를 시스템에 설치하지 않고 풀기만 한다. 시스템에 이미 있는 라이브러리는 링크하지 않는다(시스템 것 우선).
  local stage="$ROOT/opt/syslibs/root" lib="$ROOT/opt/syslibs/lib"
  mkdir -p "$stage" "$lib"
  for d in "$BUNDLE"/debs/*.deb; do dpkg-deb -x "$d" "$stage"; done
  for name in libopenblas.so.0 libgfortran.so.5 libnuma.so.1; do
    if has_syslib "$name"; then rm -f "$lib/$name"; echo "skip $name (system 것 사용)"; continue; fi
    local src; src="$(find "$stage" -name "$name" | head -1)"
    [ -n "$src" ] || { echo "FAIL $name not found in debs"; return 1; }
    ln -sfn "$(readlink -f "$src")" "$lib/$name"
    echo "link $name -> $(readlink -f "$src")"
  done
}

do_venv() {
  if [ ! -x "$VENV/bin/python" ]; then
    python3.8 -m venv --without-pip "$VENV"
    "$VENV/bin/python" "$(ls "$BUNDLE"/wheels/pip-*.whl | head -1)/pip" install --no-index \
      --find-links "$BUNDLE/wheels" pip
  fi
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
  "$VENV/bin/python" -m pip list 2>/dev/null | grep -iE "^(torch|cupy|numpy|scipy|cyclonedds|unitree|shapely|simple|ruamel|PyYAML)[^ ]* "
}

do_smoke() {
  # shellcheck disable=SC1091
  source "$ROOT/env.sh"
  local out="$LOG/gpu_smoke_$(hostname)_$(date +%Y%m%d_%H%M%S).log"
  ( timeout 40 tegrastats --interval 1000 > "$out.tegrastats" 2>&1 & )
  # 스모크 도중 CPU 를 누가 쓰는지 (우리 python vs 기존 서비스) 구분하기 위한 스냅샷
  ( sleep 12; top -b -n 1 -o %CPU 2>/dev/null | head -n 16 > "$out.top" ) &
  python "$GO2_PARKOUR/migration/gpu_smoke.py" "$GO2_PARKOUR" "$GO2_EMCUPY" 200 2>&1 | tee "$out"
  echo "log: $out"
}

do_report() {
  # 읽기 전용. 설치 결과를 한 파일로 모아 전달용으로 만든다. 일부 항목이 없어도 끝까지 수집한다.
  set +e +o pipefail
  local out="$LOG/setup_report_$(hostname)_$(date +%Y%m%d_%H%M%S).txt"
  {
    echo "== date $(date -Is) bundle $(grep -o 'unitree_rl_lab=[^ ]*' "$BUNDLE/MANIFEST.txt" 2>/dev/null)"
    echo "== layout"; ls "$ROOT" "$ROOT/opt" "$ROOT/opt/syslibs/lib" "$SRC" 2>&1
    echo "== syslibs links"; ls -la "$ROOT/opt/syslibs/lib" 2>&1 | awk '{print $9, $10, $11}'
    echo "== system has?"; for l in libopenblas.so.0 libgfortran.so.5 libnuma.so.1 libcudnn.so.8; do
      has_syslib "$l" && echo "system   $l" || echo "absent   $l"; done
    echo "== cyclonedds C"; ls "$DDS_PREFIX/lib" 2>&1 | grep ddsc || true
    echo "== env.sh"; cat "$ROOT/env.sh" 2>&1
    echo "== pip packages"; "$VENV/bin/python" -m pip list 2>/dev/null \
      | grep -iE "^(torch|cupy|numpy|scipy|cyclonedds|unitree|shapely|simple|ruamel|PyYAML|pip|setuptools)[^ ]* " || true
    echo "== torch openblas resolution"
    ( source "$ROOT/env.sh" && ldd "$VENV/lib/python3.8/site-packages/torch/lib/libtorch_cpu.so" 2>&1 \
        | grep -E "openblas|numa|gfortran|not found" ) || true
    echo "== smoke logs"; ls -la "$LOG" 2>&1 | awk '{print $5, $9}'
    local last; last="$(ls -t "$LOG"/gpu_smoke_*.log 2>/dev/null | grep -v tegrastats | head -1 || true)"
    [ -n "$last" ] && { echo "== last smoke: $last"; grep -E "PASS|FAIL|Error|Traceback" "$last" || tail -20 "$last"; \
      echo "== tegrastats during smoke"; tail -3 "$last.tegrastats" 2>/dev/null | cut -c1-200; \
      echo "== top snapshot during smoke (who uses the CPU)"; cut -c1-150 "$last.top" 2>/dev/null; } || echo "no smoke log yet"
  } > "$out" 2>&1
  cat "$out"; echo; echo "saved: $out"
}

case "${1:-}" in
  report) do_report ;;
  check) do_check ;;
  unpack) do_unpack ;;
  venv) do_venv ;;
  dds) do_dds ;;
  pkgs) do_pkgs ;;
  smoke) do_smoke ;;
  syslibs) do_syslibs ;;
  all) do_check; do_unpack; do_syslibs; do_venv; do_dds; do_pkgs; do_smoke ;;
  *) sed -n '2,18p' "$0"; exit 2 ;;
esac
