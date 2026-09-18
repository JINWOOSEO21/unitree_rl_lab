#!/usr/bin/env bash
# Galaxy Book4 Pro (Ubuntu 22.04 x86_64) controller 측 준비 (단계 A-2, A-3).
# 로봇 제어/DDS 송신 없음. sudo 는 'deps' 가 출력하는 apt 한 줄만 사용자가 직접 실행한다.
#
#   bash notebook_setup.sh deps      # 필요한 apt 명령 출력 + 누락 패키지 점검
#   bash notebook_setup.sh clone     # unitree_rl_lab(BRANCH) + unitree_sdk2(SDK_COMMIT) clone
#   bash notebook_setup.sh sdk       # SDK 원본의 헤더/라이브러리 확인 (별도 설치 없음)
#   bash notebook_setup.sh build     # go2_ctrl 새 build (데스크톱 build 디렉터리 복사 금지)
#   bash notebook_setup.sh manifest  # 버전/hash 기록
#   bash notebook_setup.sh run <nic> # 실제 LowCmd. 사용자 지시가 있을 때만.
set -euo pipefail

CODES="${CODES:-$HOME/workspace/codes}"
BRANCH="${BRANCH:-main}"
RL_LAB_URL="${RL_LAB_URL:-https://github.com/JINWOOSEO21/unitree_rl_lab.git}"
SDK_URL="${SDK_URL:-https://github.com/unitreerobotics/unitree_sdk2.git}"
SDK_COMMIT="${SDK_COMMIT:-9754cd1}"          # 데스크톱에서 검증된 unitree_sdk2 commit
SDK_ROOT="${SDK_ROOT:-$CODES/unitree_sdk2}"
DEPLOY="$CODES/unitree_rl_lab/deploy"
GO2="$DEPLOY/robots/go2"
APT_PKGS=(build-essential cmake git libboost-program-options-dev libyaml-cpp-dev
          libeigen3-dev libfmt-dev libx11-dev python3)

# Require an explicit NIC and isolate this process from ROS DDS environment settings.
# The build pins DDS libraries to SDK_ROOT using DT_RPATH.
go2_env() {
  [ -n "${CYCLONEDDS_URI:-}" ] && echo "[go2_env] CYCLONEDDS_URI 제거: $CYCLONEDDS_URI" >&2
  env -u CYCLONEDDS_URI "$@"
}

case "${1:-}" in
deps)
  echo "직접 실행: sudo apt update && sudo apt install -y ${APT_PKGS[*]}"
  for p in "${APT_PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 && echo "  ok      $p" || echo "  MISSING $p"
  done
  # go2 CMakeLists 는 정적 라이브러리 이름을 직접 링크한다. 22.04 패키지에 .a 가 있는지 확인.
  for a in libboost_program_options.a libyaml-cpp.a; do
    find /usr/lib -name "$a" 2>/dev/null | grep -q . && echo "  ok      $a" || echo "  MISSING $a (정적 라이브러리)"
  done
  ;;
clone)
  mkdir -p "$CODES"
  # LFS 대상은 MuJoCo .obj 메시뿐이며 배포에 불필요하다. git-lfs 없이도 clone 되도록 smudge 를 건너뛴다.
  [ -d "$CODES/unitree_rl_lab/.git" ] || GIT_LFS_SKIP_SMUDGE=1 git clone --branch "$BRANCH" "$RL_LAB_URL" "$CODES/unitree_rl_lab"
  [ -d "$SDK_ROOT/.git" ] || git clone "$SDK_URL" "$SDK_ROOT"
  git -C "$SDK_ROOT" checkout --detach "$SDK_COMMIT"
  ;;
sdk)
  SDK_ARCH="$(uname -m)"
  for path in include/unitree/robot/channel/channel_factory.hpp \
      "lib/$SDK_ARCH/libunitree_sdk2.a" \
      "thirdparty/lib/$SDK_ARCH/libddsc.so.0" \
      "thirdparty/lib/$SDK_ARCH/libddscxx.so.0"; do
    [ -f "$SDK_ROOT/$path" ] || { echo "Missing SDK file: $SDK_ROOT/$path" >&2; exit 1; }
  done
  echo "SDK ready: $SDK_ROOT (no separate installation)"
  ;;
build)
  # Clear legacy manual include/link flags when reusing an installed-SDK build cache.
  # CMake now gets headers and libraries from the source checkout's SDK targets.
  cmake -S "$GO2" -B "$GO2/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUNITREE_SDK_ROOT="$SDK_ROOT" \
    -DCMAKE_CXX_FLAGS= -DCMAKE_EXE_LINKER_FLAGS=
  cmake --build "$GO2/build" -j"$(nproc)"
  echo "DDS libraries must resolve under $SDK_ROOT/thirdparty/lib:"
  ldd "$GO2/build/go2_ctrl" | grep -E "ddsc|onnxruntime|not found" || true
  ;;
run)
  # 실제 LowCmd 를 보낸다. 사용자가 명시적으로 지시할 때만 실행한다.
  NET="${2:?사용법: notebook_setup.sh run <network-interface>}"
  LOGDIR="${LOGDIR:-$HOME/go2_logs}"; mkdir -p "$LOGDIR"
  RUN_ID="$(date +%Y%m%d_%H%M%S_%N)"
  LOG="$LOGDIR/go2_ctrl_${RUN_ID}.log"
  echo "run_id=$RUN_ID host=$(hostname) net=$NET wall=$(date -Is) log=$LOG"
  { echo "run_id=$RUN_ID host=$(hostname) net=$NET"
    echo "wall=$(date -Is) monotonic_ns=$(awk '{printf "%.0f", $1*1e9}' /proc/uptime)"
    timedatectl show -p NTPSynchronized --value 2>/dev/null | sed 's/^/ntp_synchronized=/'
  } | tee "$LOG"
  go2_env "$GO2/build/go2_ctrl" --network "$NET" --keyboard 2>&1 | tee -i -a "$LOG"
  ;;
manifest)
  echo "host=$(hostname) date=$(date -Is) session=${XDG_SESSION_TYPE:-?} display=${DISPLAY:-}"
  lsb_release -ds; gcc --version | head -1; cmake --version | head -1
  dpkg -l libboost-program-options-dev libyaml-cpp-dev libeigen3-dev libfmt-dev libx11-dev 2>/dev/null | awk '/^ii/{print $2, $3}'
  echo "unitree_rl_lab $(git -C "$CODES/unitree_rl_lab" rev-parse --short HEAD) dirty=$(git -C "$CODES/unitree_rl_lab" status --short | wc -l)"
  echo "unitree_sdk2   $(git -C "$SDK_ROOT" rev-parse --short HEAD)"
  (cd "$CODES/unitree_rl_lab/deploy" && sha256sum parkour/contract/policy.onnx parkour/contract/deploy.yaml \
     parkour/contract/policy_meta.json parkour/contract/em_geometry.npz \
     thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0)
  ;;
*)
  sed -n '2,10p' "$0"; exit 2 ;;
esac
