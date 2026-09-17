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

## Jetson 1차 조사 결과 (2026-09-17, `jetson_survey.sh`)

원본은 `deploy/parkour/captures/migration/`(gitignore)에 보관한다. Jetson 작업 폴더는 `~/walking`이다.

| 항목 | 실측값 |
|---|---|
| 플랫폼 | L4T R35.3.1 (JetPack 5.1.1), Ubuntu 20.04.5 aarch64, kernel 5.10.104-tegra |
| GPU 스택 | CUDA 11.4.19 (nvcc V11.4.315), cuDNN 8.6.0.166, TensorRT 8.5.2.2 |
| Python | 시스템 3.8.10 (pip 20.0.2), python3.9 존재, venv 가능, `~/.local/bin/uv` 설치됨. conda 없음 |
| 기존 GPU/DDS 패키지 | torch, cupy, cyclonedds, unitree_sdk2py 모두 없음. 시스템 numpy 1.17.4, scipy 1.3.3 |
| 빌드 도구 | gcc/g++ 11.4.0, cmake 3.16.3, git 2.25.1 |
| 자원 | 8코어, RAM 15GiB 중 1.3GiB 사용, / 여유 248GiB, MAXN, 유휴 tj 약 58°C, GPU 0% |
| 네트워크 | eth0=192.168.123.18/24, default via 192.168.123.1. 인터넷 도달성은 1차 스크립트 결함으로 미확인 |
| DDS | `/usr/local/lib/libddsc.so.0.11.0` + `libunitree_sdk2.a` 설치됨, `~/cyclonedds`, `~/cyclonedds_ws` 존재. 전역 `CYCLONEDDS_URI`가 eth0로 설정됨 |

이식에 영향을 주는 발견:

1. **시계가 1970-02-08이고 NTP 미동기, RTC도 1970년이다.** 인터넷이 되더라도 HTTPS 인증서 검증이 실패해 pip/git이 막힐 수 있다. 계획 D-5의 wall+monotonic anchor 분석에서도 Jetson wall time은 신뢰할 수 없으므로 LowState tick 같은 공유 ID를 기준으로 삼는다.
2. **CycloneDDS C 라이브러리가 0.11 계열이다.** unitree_sdk2_python은 `cyclonedds==0.10.2`를 고정하므로 호환 여부를 확인하고, 필요하면 `~/walking` 아래에 0.10.x를 별도 prefix로 빌드한다. 시스템 `/usr/local`은 기존 로봇 서비스가 쓰므로 덮어쓰지 않는다.
3. **상시 구동되는 기존 서비스가 많다** (Go2 ROS launch, odometry/SLAM, grid map, 전력 모니터링, 카메라 스트리밍, ROS foxy docker 컨테이너, 원격 데스크톱). odometry 프로세스 하나는 조사 시점 실행 2초째로 재시작 반복이 의심된다. 서비스 이름과 프로세스 목록은 공개 repo에 적지 않고 원본 파일에만 둔다. 이 중 LowCmd를 보내는 것이 있는지는 단계 E 전에 반드시 확인한다 (`jetson_survey2.sh` 3번).
4. bridge 매핑 경로가 실제로 로드하는 서드파티는 torch, cupy, numpy, scipy, shapely, simple_parsing, ruamel.yaml, pyyaml뿐이다 (데스크톱에서 backend 인스턴스화 후 `sys.modules` 실측). elevation_mapping_cupy `requirements.txt`의 opencv, scikit-image, matplotlib, catkin-tools는 설치하지 않는다.
5. backend가 쓰는 API는 `torch.as_tensor/stack/where/no_grad`, `cp.ElementwiseKernel/asarray/MemoryPool` 수준이고 DLPack을 쓰지 않는다. `ElementwiseKernel`은 런타임 NVRTC 컴파일이 필요하다 (CUDA 11.4 toolkit 존재).

## Jetson 패키지 버전 (공식 자료 확인, 2026-09-17)

| 패키지 | 버전 | 근거 |
|---|---|---|
| torch | `2.0.0+nv23.05` cp38 aarch64 | NVIDIA "Installing PyTorch for Jetson Platform" 문서가 JetPack 5.1.1용으로 지정한 v511 wheel. JetPack 5.x의 최신은 2.1.0a0(nv23.06, v512)이며 2.2 이상은 JetPack 6/cp310 전용 |
| cupy | `cupy-cuda11x==12.3.0` | cp38 aarch64 wheel이 있는 마지막 버전. CuPy v13은 Python 3.8 지원 중단. v12 지원 목록에 CUDA 11.4 포함, v12.0.0부터 aarch64 wheel을 PyPI에서 제공 |
| numpy / scipy | `1.24.4` / `1.10.1` | Python 3.8을 지원하는 마지막 릴리스 (PyPI `requires_python`). NVIDIA 문서의 `numpy==1.26.1` 고정은 3.9 이상 전용이라 따르지 않는다 |
| cyclonedds (python) | `0.10.2` sdist | PyPI에 Linux aarch64 wheel이 없어 소스 빌드. unitree_sdk2_python README는 C 라이브러리를 `releases/0.10.x`에서 빌드해 `CYCLONEDDS_HOME`으로 지정하라고 안내한다. 0.10.2 바인딩과 0.11 C 라이브러리의 호환을 보증하는 공식 문서는 찾지 못했으므로 C 0.10.2를 `~/walking/opt`에 별도 빌드한다 |
| shapely, simple-parsing, ruamel.yaml, pyyaml | 2.0.7, 0.1.7, 0.18.16, 6.0.3 | cp38/aarch64로 해석된 버전 (`make_jetson_bundle.sh` 실행 결과) |

데스크톱 검증: `make_jetson_bundle.sh`로 29개 wheel이 모두 cp38/aarch64 바이너리로 해석됨(335MB). CycloneDDS C 0.10.2는 스크립트의 CMake 옵션으로 빌드되어 `libddsc.so.0.10.2`를 생성(최소 cmake 3.16, Jetson은 3.16.3). `gpu_smoke.py`는 데스크톱(RTX 3060, torch 2.7/cupy 13.6)에서 6단계 통과, update+sample p50 2.2ms / 첫 호출 207ms. 이 wheel들이 Jetson 실기에서 동작하는지는 아직 검증되지 않았다.

### Jetson 설치 절차 (sudo 없음, `~/walking` 밖은 건드리지 않음)

```
# 데스크톱
bash deploy/parkour/migration/make_jetson_bundle.sh <pip 있는 python>     # ~/workspace/codes/go2_jetson_bundle
scp -r ~/workspace/codes/go2_jetson_bundle unitree@192.168.123.18:~/walking/
# Jetson
cd ~/walking/go2_jetson_bundle
bash jetson_setup.sh check     # 먼저 이것만. FAIL 항목이 있으면 중단하고 보고
bash jetson_setup.sh all       # unpack -> venv -> dds -> pkgs -> smoke
```

Jetson `check` 1회차(2026-09-17)에서 `python3.8-venv`(ensurepip)와 `libopenblas`가 없다고 나왔다. 둘 다 sudo 없이 해결한다.

- venv: `python3.8 -m venv --without-pip` 후 번들의 pip wheel을 직접 실행해 부트스트랩한다.
- libopenblas: torch wheel의 `DT_NEEDED`를 직접 조사해 시스템에 없을 수 있는 것은 `libopenblas.so.0`, `libnuma.so.1`뿐임을 확인했다(MPI 불필요). `fetch_focal_arm64_debs.py`가 ports.ubuntu.com의 focal arm64 인덱스에서 `.deb`를 받아 SHA256을 대조하고, `jetson_setup.sh syslibs`가 `~/walking/opt/syslibs`에 풀어 시스템에 없는 라이브러리만 링크한다. 시스템에는 설치하지 않으며 `env.sh`의 `LD_LIBRARY_PATH`로만 연결된다.

`smoke`의 `elevation-mapper` 줄에 나오는 update+sample p95/max가 bridge의 cloud/map 200ms deadline 대비 Jetson의 여유를 보여준다. bridge 실행 전에는 `source ~/walking/env.sh`를 적용한다.

## 알려진 주의점

1. **git-lfs**: `.gitattributes`의 LFS 대상은 MuJoCo `.obj` 17개뿐이고 배포에 불필요하다. git-lfs가 없는 장비에서 전역 LFS 필터가 켜져 있으면 checkout이 실패하므로 `GIT_LFS_SKIP_SMUDGE=1`로 clone한다 (`notebook_setup.sh clone`에 반영).
2. **정적 라이브러리**: go2 CMakeLists는 `libboost_program_options.a`, `libyaml-cpp.a`를 이름으로 링크한다. Ubuntu 22.04 패키지에 `.a`가 있는지는 노트북에서 `notebook_setup.sh deps`로 확인한다. 없으면 그때 링크 방식을 결정한다.
3. **Jetson 오프라인 가능성**: Jetson이 인터넷에 못 나가면 wheel/소스를 데스크톱에서 받아 옮겨야 한다. 조사 스크립트가 pypi 도달성을 기록한다.
4. **captures 의존 도구**: `audit_go2_targets.py`, `check_go2_repeatability.py`, `shadow_go2_policy.py` 등 오프라인 분석 도구는 데스크톱 캡처가 필요하다. 이식 대상 장비에서는 실행하지 않는다.

## 다음 단계

1. 노트북: 이 브랜치를 clone한 뒤 노트북에서 Claude Code를 실행하고 아래 요청문을 전달한다.
2. Jetson: 1차 조사 완료. `~/walking`에서 `bash jetson_survey2.sh`(인터넷/시계, CycloneDDS 버전, 기존 서비스의 LowCmd 사용 여부)를 실행해 결과를 전달하고, 위 "Jetson 설치 절차"를 진행한다.
3. 이후 단계 B(설치) → C(유선 분산, 수신 전용) → D(AP 무선, 수신 전용) → 측정 보고 → E(실제 제어).

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

