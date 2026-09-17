# Jetson + Galaxy Book4 Pro 이식: manifest와 진행 기록

계획: `../migration_jetson_galaxybook_plan.md`. 이 문서는 2026-09-17 데스크톱 세션에서 실측한 값만 기록한다.
로봇 제어, DDS 송신, Jetson/노트북 설치는 이 세션에서 수행하지 않았다.

## 결정 사항 (2026-09-17 사용자 확인)

| 항목 | 결정 |
|---|---|
| 코드 전달 | 브랜치 `migration/jetson-galaxybook` commit + push, 각 장비에서 clone. main 직접 push 없음 |
| 노트북 작업 | 노트북에서 별도 Claude Code 세션으로 진행 (데스크톱과 네트워크 대역이 달라 SSH 원격 불가). 아래 "노트북 세션 요청문" 사용 |
| Jetson 작업 | `jetson_survey.sh`를 사용자가 직접 실행하고 결과 파일 전달. 비밀번호는 공유하지 않음 |
| 키보드 세션 | GDM 로그인에서 "Ubuntu on Xorg" 선택. Wayland/XWayland 경로는 사용하지 않음 |

## 소스 기준선

| repo | commit | 상태 |
|---|---|---|
| unitree_rl_lab | main `6fc3da7` + 이 브랜치의 미커밋분 23개 파일 | 데스크톱 working tree와 바이트 단위 일치 확인 |
| unitree_sdk2 (C++) | `9754cd1` | clean, 원격 HEAD 그대로 사용 가능 |
| unitree_sdk2_python | `65691c8` | clean. `python>=3.8`, `cyclonedds==0.10.2` 고정 |
| elevation_mapping_cupy | `20a8a26` (leggedrobotics) | clean, submodule 없음 |

모델/런타임 SHA256 (`deploy/` 기준):

```
02b82a5b6f87cf4c9814c8af4d16e6d23a0bd5333f17638337f8db1fc898ef70  parkour/contract/policy.onnx
f6c4238c44670cd863746f5cb54aa04a3ae89a55af1160238272154de75b305e  parkour/contract/deploy.yaml
21144fed1738707ac967d6cc6c85aad4d14c4d13eb2880b7c05f4df178c5aa8e  parkour/contract/policy_meta.json
b1e4207b405705658de1fcba6fada8133a0bfaa49660a96d4cae6048d48556a1  parkour/contract/em_geometry.npz
3da6146e14e7b8aaec625dde11d6114c7457c87a5f93d744897da8781e35c673  thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
```

위 5개 파일은 모두 git에 일반 blob으로 추적된다. `robots/go2/config/config.yaml`의 `policy_dir`는 상대경로(`../../parkour/contract`)다.

## 데스크톱 참고 버전 (복사 대상 아님)

Ubuntu 24.04 계열, gcc 13.3.0, cmake는 conda env_isaaclab 것 사용, boost 1.83 / yaml-cpp 0.8.0 / eigen 3.4.0 / fmt 9.1.0.
bridge 환경: Python 3.11.15, numpy 1.26.0, torch 2.7.0+cu128, cupy 13.6.0 (CUDA runtime 12.9), pyyaml 6.0.2, cyclonedds 0.10.2.
이 조합은 Jetson(L4T R35.3.1, Python 3.8)에 그대로 설치할 수 없다. Jetson 버전은 `jetson_survey.sh` 실측 후 공식 자료로 정한다.

## 이 세션의 검증 결과

- worktree 새 build: go2_ctrl 빌드 성공, CTest 7/7 통과 (keyboard, startup handoff, non-tty 거부, keyboard PTY, held heading, shutdown, gyro bias).
- `notebook_setup.sh`의 `deps → sdk → build → test`를 임시 prefix로 실행: go2_ctrl이 prefix의 `libddsc.so.0`/`libddscxx.so.0`과 repo 내 `libonnxruntime.so.1`로 해석됨, CTest 7/7 통과. CMakeLists 수정 없음.
- Python: `tools/tests` + `tests` 71 통과, 2 실패. 실패 2건은 `test_audit_go2_targets.py`이며 gitignore된 `captures/frame_inspection_20260915/jetson_go2_description.urdf`가 없어서다. captures가 있는 데스크톱 checkout에서는 6/6 통과. `go2_sensor_bridge.py` 런타임 경로는 captures를 참조하지 않는다.
- Python 3.8 정적 검사: bridge 런타임(`tools`, `em_sidecar`, `vendored`, `common`, `lio`) 46개 파일과 elevation_mapping_cupy 58개 파일에서 3.8 비호환 문법/API 없음. 실제 3.8 인터프리터 실행 검증은 Jetson에서 해야 한다.

## 노트북(Galaxy Book4 Pro) 세션 결과 — 2026-09-17, 단계 A-2~A-4

장비 실측: host `seojinwoo`, Ubuntu 22.04.5, Intel Core Ultra 7 155H, x86_64, RAM 30GiB,
`wlo1=10.50.1.32/21` (Go2 AP 아님), 세션 `wayland`, `DISPLAY=:0`.
로봇 전원/DDS 송신/LowCmd 없음. 이 세션에서 `go2_ctrl` 본 실행은 하지 않았다.

### A-2 빌드

- `deps`: apt 9개 패키지가 이미 전부 설치되어 있어 sudo 불필요. **`libboost_program_options.a`와
  `libyaml-cpp.a` 둘 다 22.04 패키지에 존재** — 위 주의점 2는 해소됐다. 링크 방식 변경 없음.
- `unitree_sdk2` `9754cd1` clean clone → `~/opt/unitree_sdk2` 설치 (sudo 없음).
- `go2_ctrl` 새 build 성공. 데스크톱 build 디렉터리 복사 없음. CMakeLists의 소스 변경 없음.
- 버전 manifest: gcc 11.4.0, cmake 3.22.1, boost 1.74.0.3, yaml-cpp 0.7.0, eigen 3.4.0,
  fmt 8.1.1, libx11 1.7.5. **모델/런타임 SHA256 5개가 데스크톱 기준선과 완전히 일치.**

### A-2 발견: ROS 2 Humble이 SDK의 CycloneDDS를 가로챈다 (수정함)

이 노트북은 `.bashrc`에서 ROS 2 Humble을 source 한다. 그러면 `LD_LIBRARY_PATH`에
`/opt/ros/humble/lib/x86_64-linux-gnu`가 들어간다. 기존 `notebook_setup.sh build`는
`-Wl,-rpath`만 줬고, 요즘 링커 기본값은 **DT_RUNPATH**인데 loader는 `LD_LIBRARY_PATH`를
DT_RUNPATH보다 **먼저** 본다. 결과:

```
libddsc.so.0   => /opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so.0   (ROS Cyclone 0.10.5)
libddscxx.so.0 => /home/seojinwoo/opt/unitree_sdk2/lib/libddscxx.so.0 (unitree)
```

ROS는 `libddscxx`를 배포하지 않으므로 C++ 바인딩만 SDK에서 오고 C 코어는 ROS 것이 잡혀,
**서로 다른 빌드의 Cyclone C 코어 + C++ 바인딩이 섞인 상태**로 로봇과 통신하게 된다.
데스크톱에는 ROS가 없어 드러나지 않았던 문제다.

수정: `notebook_setup.sh build`에 `-Wl,--disable-new-dtags`를 추가해 DT_RPATH를 쓰게 했다.
DT_RPATH는 `LD_LIBRARY_PATH`보다 먼저 검색되므로 ROS를 source 한 셸에서도 SDK 것이 잡힌다.
재빌드 후 (ROS가 source 된 상태에서) 확인:

```
libddsc.so.0     => /home/seojinwoo/opt/unitree_sdk2/lib/libddsc.so.0
libddscxx.so.0   => /home/seojinwoo/opt/unitree_sdk2/lib/libddscxx.so.0
libonnxruntime.so.1 => deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1
```

### A-3 테스트

- CTest **7/7 통과** (keyboard, startup handoff, non-tty 거부, keyboard PTY, held heading,
  shutdown, gyro bias).
- ONNX offline trace 검사 (`deploy/parkour/tests/test_obs_golden`, 지정 입력
  `contract/golden_trace.npz` 100프레임): **4/4 PASS** — prop 53 조립 2.38e-07,
  history 530 정렬 0, 액션 지연/스케일 0, ONNX 정책 출력 2.03e-06.
  즉 이 노트북의 ONNX Runtime x86_64 수치 결과가 기준선과 일치한다.
- 참고: `long_trace.npz`는 `em_sidecar/tests/test_leg_odometry.py`용 leg odometry fixture이며
  `test_obs_golden`의 입력이 아니다. 여기에 물리면 관측 조립 단계가 FAIL로 나오지만
  (ONNX 출력 단계는 1000프레임 12000개 전부 PASS) 이는 fixture를 잘못 물린 것이지 회귀가 아니다.

### A-4 키보드 / X11 — 누름/해제/동시누름 통과, 포커스 이탈만 남음

코드 확인 결과 **`--keyboard-check`로는 a/d hold 조향을 검증할 수 없다**:
`Go2TerminalInput::poll()`은 상태가 Policy가 아니면 매 폴링마다 `held_heading_.clear()` 하고
`Go2KeyboardControl::set_heading_keys()`도 Policy가 아니면 early-return 한다.
`--keyboard-check`는 FSM 없이 `Go2KeyboardControl`을 Passive 그대로 두므로 a/d가 항상 0이다.
(계획 A-4의 "부족하면 별도 read-only 입력 진단을 사용한다"에 해당.)

그래서 읽기 전용 진단 `deploy/robots/go2/tools/keyboard_x11_probe.cpp`를 추가했다
(`go2_keyboard_x11_probe`, `notebook_setup.sh probe`). DDS/FSM/LowCmd 없이 실제
`Go2HeldHeadingInput` + `Go2KeyboardControl`(Policy 강제)을 돌려 a/d의 누름/해제/동시누름/
포커스 이탈과 그 결과 `heading_offset`(±0.2618 rad = ±15도)을 출력한다.

현재 wayland 세션 실측:

```
XDG_SESSION_TYPE=wayland DISPLAY=:0 WAYLAND_DISPLAY=wayland-0
XGetInputFocus window=0x0 (None: no X client holds focus)
RESULT: X11 steering UNAVAILABLE. a/d hold steering would be disabled.
```

`go2_ctrl --keyboard-check`도 같은 세션에서
`[keyboard] X11 unavailable: a/d hold steering disabled` 를 출력한다 (안전하게 비활성화될 뿐
오작동하지는 않는다). **따라서 wayland 세션에서는 a/d 조향을 쓸 수 없다.**

#### Xorg 재로그인 후 실측 (사용자 직접 실행, 2026-09-17)

GDM에서 "Ubuntu on Xorg" 선택 → `XDG_SESSION_TYPE=x11`, `XGetInputFocus`가 실제 터미널
window 를 반환하고 `available()` 이 true. 결과:

| 입력 | `heading_offset` | 판정 |
|---|---|---|
| `a` 누름 | `+0.2618 rad (+15.0도)` | PASS |
| `a` 해제 | `0` | PASS |
| `d` 누름 | `-0.2618 rad (-15.0도)` | PASS |
| `d` 해제 | `0` | PASS |
| `a`+`d` 동시 | `0` (상쇄) | PASS |
| 동시누름에서 한 키만 해제 | 남은 키 값으로 복귀 (`a=1 d=1 → 0` 다음 `a=1 d=0 → +15`) | PASS |

값이 `0.2617993878 rad = 15도` 와 정확히 일치한다. 동시누름 상태에서 빠져나올 때 latch 없이
복귀하는 것도 확인했다. wayland 와 달리 Xorg 에서는 조향이 정상 동작한다.

**포커스 이탈은 아직 미확인.** 최초 probe 는 포커스를 잃었을 때도 `a=0 d=0 -> 0` 으로만 찍어서
키 해제와 구분되지 않았다 (probe 설계 실수). `Go2HeldHeadingInput::focused()` 접근자를 추가하고
probe 가 `=== FOCUS LOST / REGAINED ===` 를 명시적으로 출력하도록 고쳤다. 남은 확인:
**`a` 를 누른 채로** 다른 창을 클릭해 `FOCUS LOST` 직후 `0` 이 되는지 (키가 아직 눌린 상태에서)
보는 것. 포커스 처리 자체는 `go2_held_heading_tests` 가 fake Xlib 백엔드로 이미 검증하지만,
실제 X 서버에서 `XGetInputFocus` 가 창 전환을 보고하는지는 실측이 필요하다.

## 알려진 주의점

1. **git-lfs**: `.gitattributes`의 LFS 대상은 MuJoCo `.obj` 17개뿐이고 배포에 불필요하다. git-lfs가 없는 장비에서 전역 LFS 필터가 켜져 있으면 checkout이 실패하므로 `GIT_LFS_SKIP_SMUDGE=1`로 clone한다 (`notebook_setup.sh clone`에 반영).
2. ~~**정적 라이브러리**~~: 해소됨. Ubuntu 22.04 패키지에 `libboost_program_options.a`와 `libyaml-cpp.a`가 모두 있어 링크 방식 변경이 필요 없었다 (2026-09-17 노트북 실측).
3. **Jetson 오프라인 가능성**: Jetson이 인터넷에 못 나가면 wheel/소스를 데스크톱에서 받아 옮겨야 한다. 조사 스크립트가 pypi 도달성을 기록한다.
4. **captures 의존 도구**: `audit_go2_targets.py`, `check_go2_repeatability.py`, `shadow_go2_policy.py` 등 오프라인 분석 도구는 데스크톱 캡처가 필요하다. 이식 대상 장비에서는 실행하지 않는다.
5. **ROS 2와 CycloneDDS 충돌**: ROS 2가 설치된 장비에서는 `notebook_setup.sh build`의 `-Wl,--disable-new-dtags`가 반드시 필요하다. 위 "A-2 발견" 참고. 다른 방법으로 빌드할 때는 `ldd`로 `libddsc.so.0`이 SDK prefix에서 오는지 매번 확인한다. Jetson에도 ROS가 있으면 bridge 쪽 `cyclonedds` Python 바인딩에 같은 종류의 충돌이 없는지 확인할 것.
6. **wayland에서는 a/d 조향 불가**: `Go2HeldHeadingInput`은 `XGetInputFocus`/`XQueryKeymap`을 쓴다. wayland 세션에서는 focus가 `None`으로 잡혀 조향이 비활성화된다 (crash 없이 비활성화). 실제 주행 세션은 반드시 "Ubuntu on Xorg"로 로그인한다.

## 다음 단계

1. ~~노트북 clone + 세션 실행~~ 완료. 단계 A-2/A-3 통과, A-4는 아래가 남았다.
2. ~~"Ubuntu on Xorg" 재로그인 + a / d / a+d 확인~~ 완료 (위 표). **남은 것: 포커스 이탈 1건** — `a`를 누른 채 다른 창 클릭 → `FOCUS LOST` 후 `0`. 물리 키 입력이라 대행 불가.
3. Jetson: `bash jetson_survey.sh` 실행 후 결과 파일 전달 (노트북을 Go2에 유선 연결한 뒤 노트북 세션에서 해도 된다).
4. 이후 단계 B(설치) → C(유선 분산, 수신 전용) → D(AP 무선, 수신 전용) → 측정 보고 → E(실제 제어).

### 노트북에서 최초 1회

```
sudo apt update && sudo apt install -y git
mkdir -p ~/workspace/codes && cd ~/workspace/codes
GIT_LFS_SKIP_SMUDGE=1 git clone --branch migration/jetson-galaxybook https://github.com/JINWOOSEO21/unitree_rl_lab.git
cd unitree_rl_lab && claude
```

브랜치가 main에 merge된 뒤에는 `--branch main`으로 바꾼다.

### 노트북 세션 요청문

> `deploy/parkour/migration_jetson_galaxybook_plan.md`와 `deploy/parkour/migration/README.md`를 읽고 Galaxy Book4 Pro controller 쪽 이식을 진행해줘. 이 장비가 노트북이다. 단계 A-2~A-4부터 시작해: `deploy/parkour/migration/notebook_setup.sh`의 deps → clone → sdk → build → test → manifest 순서로 실행하고, sudo가 필요한 apt 명령은 나에게 실행을 요청해. 데스크톱 build 디렉터리는 복사하지 말고 새로 빌드해. 세션이 Xorg인지(`echo $XDG_SESSION_TYPE`) 먼저 확인하고, 모터 출력 없이 `--keyboard-check`와 a/d hold/release/동시누름/포커스이탈을 검증해. go2_ctrl 본 실행과 LowCmd 송신은 내가 명시적으로 지시하기 전에는 하지 마. 결과와 manifest 출력을 README에 기록하고 같은 브랜치에 commit/push해줘.

