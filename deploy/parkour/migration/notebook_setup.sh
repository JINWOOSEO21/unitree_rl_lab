#!/usr/bin/env bash
# Galaxy Book4 Pro (Ubuntu 22.04 x86_64) controller 측 준비 (단계 A-2, A-3).
# 로봇 제어/DDS 송신 없음. sudo 는 'deps' 가 출력하는 apt 한 줄만 사용자가 직접 실행한다.
#
#   bash notebook_setup.sh deps      # 필요한 apt 명령 출력 + 누락 패키지 점검
#   bash notebook_setup.sh clone     # unitree_rl_lab(BRANCH) + unitree_sdk2(SDK_COMMIT) clone
#   bash notebook_setup.sh sdk       # unitree_sdk2 를 $SDK_PREFIX 에 설치 (sudo 불필요)
#   bash notebook_setup.sh build     # go2_ctrl 새 build (데스크톱 build 디렉터리 복사 금지)
#   bash notebook_setup.sh test      # CTest (키보드 PTY, held heading, shutdown, gyro bias 포함)
#   bash notebook_setup.sh manifest  # 버전/hash 기록
set -euo pipefail

CODES="${CODES:-$HOME/workspace/codes}"
BRANCH="${BRANCH:-migration/jetson-galaxybook}"
RL_LAB_URL="${RL_LAB_URL:-https://github.com/JINWOOSEO21/unitree_rl_lab.git}"
SDK_URL="${SDK_URL:-https://github.com/unitreerobotics/unitree_sdk2.git}"
SDK_COMMIT="${SDK_COMMIT:-9754cd1}"          # 데스크톱에서 검증된 unitree_sdk2 commit
SDK_PREFIX="${SDK_PREFIX:-$HOME/opt/unitree_sdk2}"
GO2="$CODES/unitree_rl_lab/deploy/robots/go2"
APT_PKGS=(build-essential cmake git libboost-program-options-dev libyaml-cpp-dev
          libeigen3-dev libfmt-dev libx11-dev python3)

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
  cmake -S "$GO2" -B "$GO2/build" \
    -DCMAKE_CXX_FLAGS="-I$SDK_PREFIX/include -I$SDK_PREFIX/include/ddscxx" \
    -DCMAKE_EXE_LINKER_FLAGS="-L$SDK_PREFIX/lib -Wl,-rpath,$SDK_PREFIX/lib"
  cmake --build "$GO2/build" -j"$(nproc)"
  echo "--- go2_ctrl 공유 라이브러리 해석 (not found 가 없어야 함)"
  ldd "$GO2/build/go2_ctrl" | grep -E "ddsc|onnxruntime|not found" || true
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
  sed -n '2,11p' "$0"; exit 2 ;;
esac
