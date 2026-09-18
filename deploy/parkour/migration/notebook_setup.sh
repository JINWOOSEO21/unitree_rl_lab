#!/usr/bin/env bash
# Galaxy Book4 Pro (Ubuntu 22.04 x86_64) controller 측 준비 (단계 A-2, A-3).
# 로봇 제어/DDS 송신 없음. sudo 는 'deps' 가 출력하는 apt 한 줄만 사용자가 직접 실행한다.
#
#   bash notebook_setup.sh deps      # 필요한 apt 명령 출력 + 누락 패키지 점검
#   bash notebook_setup.sh clone     # unitree_rl_lab(BRANCH) + unitree_sdk2(SDK_COMMIT) clone
#   bash notebook_setup.sh sdk       # unitree_sdk2 를 $SDK_PREFIX 에 설치 (sudo 불필요)
#   bash notebook_setup.sh build     # go2_ctrl 새 build (데스크톱 build 디렉터리 복사 금지)
#   bash notebook_setup.sh test      # CTest (키보드 PTY, held heading, shutdown, gyro bias 포함)
#   bash notebook_setup.sh probe     # 읽기 전용 X11 a/d hold 진단 (DDS/모터 출력 없음)
#   bash notebook_setup.sh rx <nic> [초]  # 수신 전용 rt/lowstate 확인 (LowCmd 없음)
#   bash notebook_setup.sh manifest  # 버전/hash 기록
#   bash notebook_setup.sh run <nic> # 실제 LowCmd. 사용자 지시가 있을 때만.
set -euo pipefail

CODES="${CODES:-$HOME/workspace/codes}"
BRANCH="${BRANCH:-migration}"
RL_LAB_URL="${RL_LAB_URL:-https://github.com/JINWOOSEO21/unitree_rl_lab.git}"
SDK_URL="${SDK_URL:-https://github.com/unitreerobotics/unitree_sdk2.git}"
SDK_COMMIT="${SDK_COMMIT:-9754cd1}"          # 데스크톱에서 검증된 unitree_sdk2 commit
SDK_PREFIX="${SDK_PREFIX:-$HOME/opt/unitree_sdk2}"
DEPLOY="$CODES/unitree_rl_lab/deploy"
GO2="$DEPLOY/robots/go2"
APT_PKGS=(build-essential cmake git libboost-program-options-dev libyaml-cpp-dev
          libeigen3-dev libfmt-dev libx11-dev python3)

# Go2 DDS 를 ROS 2 환경에서 격리한다.
#
# 이 노트북의 ~/.zshrc 는 CYCLONEDDS_URI 로 ~/cyclonedds.xml 을 전역 지정하는데,
# 그 파일은 Domain id="any" 아래에 NetworkInterface name="wlo1",
# AllowMulticast false, 그리고 무관한 네트워크(192.168.35.x)의 Peers 를 강제한다.
# ROS 2 에는 맞는 설정이지만 Go2 에는 전부 틀렸다.
#
# 이 주석은 원래 "그대로 두면 유선 인터페이스를 인자로 넘겨도 Wi-Fi 에 바인딩된다"
# 고 적혀 있었다. C++ go2_ctrl 경로에서는 사실이 아니다 (2026-09-18 실측 + 바이너리
# 분석으로 정정). ChannelFactory::Init(domain, nic) 은 nic 가 비어 있지 않으면
#   <CycloneDDS><Domain Id="any"><General><Interfaces>
#     <NetworkInterface name="<nic>" priority="default" multicast="default"/>
#   </Interfaces></General></Domain></CycloneDDS>
# 를 메모리에서 만들어 dds_create_domain() 에 직접 넘긴다. 이게
# dds_create_participant() 보다 먼저 돌아 domain 0 을 만들어 두므로, participant 가
# CYCLONEDDS_URI 를 읽을 때는 이미 늦는다 — 환경변수 XML 은 병합이 아니라 통째로
# 버려진다. libunitree_sdk2.a 에는 getenv 참조 자체가 없다.
# 실측: go2_state_probe --network <유선nic> 은 CYCLONEDDS_URI 유무와 무관하게
# 239.255.0.1 을 유선 NIC 에서 가입했다 (실행 전 없음 → 실행 중 가입 → 종료 후 없음).
#
# 따라서 진짜 규칙은 "환경변수를 지워라"가 아니라 "--network 를 반드시 넘겨라"다.
# param.h 의 기본값이 빈 문자열이고, 비어 있으면 dds_create_domain 을 건너뛰어
# CYCLONEDDS_URI 가 그대로 먹는다. 아래 제거는 그 실수와 Python 도구
# (unitree_sdk2py, 별도 경로라 미검증) 에 대한 보험으로 남겨 둔다.
#
# .zshrc 와 ROS 2 환경은 건드리지 않고, 여기서 띄우는 프로세스에서만 제거한다.
# libddsc 를 DT_RPATH 로 SDK 것에 고정한 것과 같은 계열의 격리다 (build 주석 참고).
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
probe)
  # 모터 출력/DDS 없는 읽기 전용 X11 입력 진단. 반드시 포커스된 터미널에서 직접 실행한다.
  # --keyboard-check 는 Policy 상태가 아니면 a/d 조향을 통과시키지 않아 이 검증을 못 한다.
  "$GO2/build/go2_keyboard_x11_probe"
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
  go2_env "$GO2/build/go2_ctrl" --network "$NET" --keyboard 2>&1 | tee -a "$LOG"
  ;;
rx)
  # 단계 C-1. rt/lowstate 수신만 확인한다. go2_state_probe 는 subscriber 만 만들고
  # publisher/service client 가 없어 LowCmd 를 보내지 않는다.
  NET="${2:?사용법: notebook_setup.sh rx <network-interface> [seconds]}"
  SECS="${3:-10}"
  BIN="$GO2/build/go2_state_probe"
  # parkour/tools 에는 CMakeLists 가 없어 여기서 직접 빌드한다.
  # 산출물은 gitignore 되는 build/ 아래에 둔다.
  mkdir -p "$GO2/build"
  [ -x "$BIN" ] || g++ -O2 -std=c++17 "$DEPLOY/parkour/tools/go2_state_probe.cpp" -o "$BIN" \
      -I"$SDK_PREFIX/include" -I"$SDK_PREFIX/include/ddscxx" \
      -L"$SDK_PREFIX/lib" -Wl,--disable-new-dtags -Wl,-rpath,"$SDK_PREFIX/lib" \
      -lunitree_sdk2 -lddscxx -lddsc -lpthread -lrt
  echo "--- DDS 라이브러리 해석 (ROS 것이 아니라 $SDK_PREFIX 여야 함)"
  ldd "$BIN" | grep -E "ddsc" || true
  # probe 는 배너를 flush 하지 않아 파이프로 넘기면 블록 버퍼링된다. stdbuf 로 줄 단위 출력.
  go2_env stdbuf -oL "$BIN" "$NET" "$SECS" 0
  ;;
test)
  (cd "$GO2/build" && ctest --output-on-failure)
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
  sed -n '2,13p' "$0"; exit 2 ;;
esac
