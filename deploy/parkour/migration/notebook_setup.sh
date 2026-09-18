#!/usr/bin/env bash
# Galaxy Book4 Pro (Ubuntu 22.04 x86_64) controller 측 준비 (단계 A-2, A-3).
# 로봇 제어/DDS 송신 없음. sudo 는 'deps' 가 출력하는 apt 한 줄만 사용자가 직접 실행한다.
#
#   bash notebook_setup.sh deps      # 필요한 apt 명령 출력 + 누락 패키지 점검
#   bash notebook_setup.sh clone     # unitree_rl_lab(BRANCH) + unitree_sdk2(SDK_COMMIT) clone
#   bash notebook_setup.sh sdk       # unitree_sdk2 를 $SDK_PREFIX 에 설치 (sudo 불필요)
#   bash notebook_setup.sh build     # go2_ctrl 새 build (데스크톱 build 디렉터리 복사 금지)
#   bash notebook_setup.sh manifest  # 버전/hash 기록
#   bash notebook_setup.sh run <nic> # 실제 LowCmd. 사용자 지시가 있을 때만.
set -euo pipefail

CODES="${CODES:-$HOME/workspace/codes}"
BRANCH="${BRANCH:-main}"
RL_LAB_URL="${RL_LAB_URL:-https://github.com/JINWOOSEO21/unitree_rl_lab.git}"
SDK_URL="${SDK_URL:-https://github.com/unitreerobotics/unitree_sdk2.git}"
SDK_COMMIT="${SDK_COMMIT:-9754cd1}"          # 데스크톱에서 검증된 unitree_sdk2 commit
SDK_PREFIX="${SDK_PREFIX:-$HOME/opt/unitree_sdk2}"
DEPLOY="$CODES/unitree_rl_lab/deploy"
GO2="$DEPLOY/robots/go2"
APT_PKGS=(build-essential cmake git libboost-program-options-dev libyaml-cpp-dev
          libeigen3-dev libfmt-dev libx11-dev python3)

# Require an explicit NIC and isolate this process from ROS DDS environment settings.
# The build pins DDS libraries to SDK_PREFIX using DT_RPATH.
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
  [ -d "$CODES/unitree_sdk2/.git" ] || git clone "$SDK_URL" "$CODES/unitree_sdk2"
  git -C "$CODES/unitree_sdk2" checkout --detach "$SDK_COMMIT"
  ;;
sdk)
  cmake -S "$CODES/unitree_sdk2" -B "$CODES/unitree_sdk2/build" -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX"
  cmake --build "$CODES/unitree_sdk2/build" -j"$(nproc)"
  cmake --install "$CODES/unitree_sdk2/build" | tail -n 1
  ls "$SDK_PREFIX/lib"
  ;;
build)
  # CMakeLists 의 /usr/local/include/ddscxx 경로는 없으면 무시된다. prefix 를 플래그로 연결한다.
  # --disable-new-dtags 는 DT_RUNPATH 대신 DT_RPATH 를 쓴다. ROS 2 가 설치된 장비에서는
  # .bashrc 의 setup.bash 가 LD_LIBRARY_PATH 에 /opt/ros/<distro>/lib/x86_64-linux-gnu 를 넣고,
  # loader 는 LD_LIBRARY_PATH 를 DT_RUNPATH 보다 먼저 본다. 그러면 ROS 의 libddsc.so.0 이
  # SDK 것보다 우선 잡혀서 ROS Cyclone C 코어 + unitree ddscxx C++ 바인딩이 섞인다
  # (ROS 는 libddscxx 를 배포하지 않으므로 ddscxx 만 prefix 에서 온다).
  # DT_RPATH 는 LD_LIBRARY_PATH 보다 먼저 검색되므로 ROS 를 source 한 셸에서도 SDK 것이 잡힌다.
  cmake -S "$GO2" -B "$GO2/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-I$SDK_PREFIX/include -I$SDK_PREFIX/include/ddscxx" \
    -DCMAKE_EXE_LINKER_FLAGS="-L$SDK_PREFIX/lib -Wl,--disable-new-dtags -Wl,-rpath,$SDK_PREFIX/lib"
  cmake --build "$GO2/build" -j"$(nproc)"
  echo "--- go2_ctrl 공유 라이브러리 해석 (not found 없이 전부 $SDK_PREFIX 여야 함)"
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
  echo "unitree_sdk2   $(git -C "$CODES/unitree_sdk2" rev-parse --short HEAD)"
  (cd "$CODES/unitree_rl_lab/deploy" && sha256sum parkour/contract/policy.onnx parkour/contract/deploy.yaml \
     parkour/contract/policy_meta.json parkour/contract/em_geometry.npz \
     thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0)
  ;;
*)
  sed -n '2,10p' "$0"; exit 2 ;;
esac
