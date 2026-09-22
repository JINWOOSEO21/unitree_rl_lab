# Go2 EDU 배포

노트북(Ubuntu 22.04, x86_64)은 `go2_ctrl`로 정책과 모터 제어를 실행합니다.
Jetson(Ubuntu 20.04, L4T R35.3.1)은 LiDAR 점군·LowState를 받아 odometry와
elevation map을 계산하고 terrain scandots와 본체 gyro bias를 DDS로 전달합니다.
ROS는 leg·MIT odometry에 필요하지 않습니다.

## 노트북 설치와 빌드

저장소 기본 경로는 `~/workspace/codes/unitree_rl_lab`입니다. 다른 경로를 쓰면
`CODES`를 해당 저장소의 상위 디렉터리로 지정하세요.

```bash
cd ~/workspace/codes/unitree_rl_lab
bash deploy/parkour/notebook_setup.sh deps
# deps가 안내하는 누락 패키지를 설치한 뒤:
bash deploy/parkour/notebook_setup.sh clone
bash deploy/parkour/notebook_setup.sh sdk
bash deploy/parkour/notebook_setup.sh build
```

이미 `~/workspace/codes/unitree_sdk2`가 있다면 `build`만 실행합니다.
SDK 원본의 헤더와 포함된 라이브러리를 직접 사용하므로 별도 설치는 필요하지 않습니다.
다른 SDK 경로는 스크립트의 `SDK_ROOT` 또는 CMake의 `UNITREE_SDK_ROOT`로 지정합니다. 빌드 결과는
`robots/go2/build/go2_ctrl`입니다.
빌드 스크립트는 ROS의 DDS 라이브러리와 충돌하지 않도록 SDK 라이브러리 경로를 고정합니다.

## Jetson 환경

기존 `~/walking/env.sh` 환경에서는 아래 실행 절차를 바로 사용할 수 있습니다.
Jetson용 오프라인 번들 생성·설치 스크립트(`make_jetson_bundle.sh`, `jetson_setup.sh`,
`gpu_smoke.py`, `fetch_focal_arm64_debs.py`)는 저장소에서 제거했습니다. 마지막으로
생성한 번들 사본은 `~/workspace/codes/go2_jetson_bundle/`에 남아 있으며, 필요하면
git 이력(`deploy/parkour/migration/`, 커밋 b840b97 기준)에서 복구할 수 있습니다.

Jetson 설치 환경의 주요 버전은 Python 3.8, PyTorch `2.0.0+nv23.05`,
CuPy `12.3.0`(CUDA 11.x), NumPy `1.24.4`, SciPy `1.10.1`,
CycloneDDS C/Python `0.10.2`입니다.

## 실행

Jetson에서 먼저 센서 브리지를 실행합니다.

```bash
source ~/walking/env.sh
python -m go2_bridge \
  --network eth0 --odom mit --emcupy-root "$GO2_EMCUPY" \
  --publish-scandots --summary-only
```

`--odom leg`가 기본값이며 100 Hz, `--odom mit`는 기본 75 Hz입니다.
`--leg-odom-hz`와 `--mit-odom-hz`로 각각 변경합니다.
`--check-dependencies`는 DDS participant 생성 없이 import를 확인합니다.
MIT의 센서·좌표계·추정 방법은 [MIT odometry 안내](parkour/go2_bridge/MIT_ODOMETRY.md)를 참고하세요.

노트북의 X11 데스크톱에서 포커스된 터미널을 열고 컨트롤러를 실행합니다.
`enx00e04c637ac7`은 로봇에 연결된 유선 인터페이스 이름으로 바꿉니다.

```bash
cd ~/workspace/codes/unitree_rl_lab/deploy/robots/go2
mkdir -p ~/go2_logs
env -u CYCLONEDDS_URI stdbuf -oL ./build/go2_ctrl \
  --network enx00e04c637ac7 --keyboard --log 2>&1 \
  | tee -i ~/go2_logs/controller.log
```

컨트롤러가 시작되면 `1`로 기립하고 네 발을 지지한 채 정지하여 본체 gyro의
10초 보정을 완료합니다. 브리지의 유효한 scandots와 gyro bias를 확인한 후
`2`로 정책에 진입합니다. Space는 Stand 복귀, `3`은 안정된 Stand에서 하강,
Ctrl+C는 Stand → StandDown → 자세 확인 후 종료입니다.
컨트롤러가 완전히 종료될 때까지 센서 브리지를 유지합니다.
자세한 키와 종료 조건은 [키보드 안내](robots/go2/KEYBOARD.md)에 있습니다.

`tee -i`는 Ctrl+C 중에도 종료 로그를 받습니다. `--log`는 별도로
`robots/go2/log/log.txt`에 컨트롤러 로그를 남깁니다.

## 유지되는 파일

- `parkour/contract/`: 정책 ONNX, 관측 계약, 기구학·scan 격자.
- `parkour/go2_bridge/`(실기), `parkour/em_sidecar/`(MuJoCo), `parkour/vendored/`: 센서 브리지,
  odometry, elevation map 연결 및 기록·시각화 도구.
- `parkour/notebook_setup.sh`: 노트북(controller) 측 의존성·SDK·빌드 준비.
- `parkour/eval/`: 주행 기록·오도메트리 분석·영상 렌더 (런타임이 import 하지 않음).
- `parkour/terrain/`: IsaacLab 지형을 MuJoCo hfield/scene 으로 변환하는 자산 생성.
- 저장소의 `source/`와 `scripts/`: 학습.
- `thirdparty/`, `../doc/licenses/`, `../LICENCE`: 런타임 의존성과 라이선스.

빌드 디렉터리, 센서 로그, 캐시, 영상은 Git에 포함하지 않습니다.
정책·설정·필수 MuJoCo 자산과 서드파티 라이선스는 배포 시 함께 유지하세요.
