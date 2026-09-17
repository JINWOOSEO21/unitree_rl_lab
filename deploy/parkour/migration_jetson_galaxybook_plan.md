# Go2 Jetson + Galaxy Book4 Pro 이식 인수인계

작성: 2026-09-17. 이 문서는 다음 세션의 실행 계획이다. 이번 세션에서는 설치/로봇 제어/commit/push를 수행하지 않았다.

## 목표와 범위

- Jetson: go2_sensor_bridge.py, leg odometry, gyro bias 추정, CuPy elevation map, scandots/gyro_bias DDS 송신.
- Galaxy Book4 Pro: go2_ctrl, ONNX 추론, 키보드 입력, LowCmd DDS 송신.
- Go2 자체 AP에 노트북을 연결하는 것이 최종 목표. 먼저 유선에서 이식 결과를 확인한 후 무선으로 바꾼다.
- SDK 역할 유지: bridge=unitree_sdk2_python, controller=unitree_sdk2 C++.
- Point-LIO, 정책 재학습, odometry 알고리즘 변경은 이번 이식 범위 밖.
- AP 접속/SSH 성공은 control board까지 DDS 통신 성공을 의미하지 않는다.

## 확인된 장비

Jetson: NVIDIA Orin NX Developer Kit, aarch64, L4T R35.3.1, Ubuntu 20.04.5, 기본 Python 3.8.10, RAM 15GiB, / 여유 248GiB. eth0=192.168.123.18/24. SSH 사용자 unitree. 비밀번호는 문서에 저장하지 않는다.

노트북: Galaxy Book4 Pro, Intel Core Ultra 7 155H, x86_64, Ubuntu 22.04.5, RAM 30GiB. 기존 네트워크 wlo1=10.50.1.32/21은 Go2 AP 주소가 아니다. 세션 wayland, DISPLAY=:0. 물리 키 hold/release 구현은 X11 기반이다.

데스크톱: /home/seo-jinwoo/workspace/codes; NIC enp42s0; env_isaaclab.

## 현재 코드 보존: 가장 먼저 할 일

unitree_rl_lab main HEAD=6fc3da7. 2026-09-17 확인 시 tracked/untracked 변경 22개가 있으며, graceful shutdown와 공유 gyro bias 코드가 아직 commit되지 않았다. 단순 git clone만 하면 최신 검증 동작이 빠진다.

1. git status/diff와 untracked 파일을 다시 확인한다. 사용자의 다른 변경을 덮어쓰지 않는다.
2. 현재 동작 소스와 policy.onnx, deploy.yaml, policy_meta.json, em_geometry.npz의 SHA256을 기록한다.
3. 최신 변경을 포함한 재현 가능한 소스 스냅샷을 만든다. git tracked 파일만 보내고 끝내지 말고 untracked 실행 의존 파일도 포함한다. git 기반 공유를 선택하면 관련 변경 검토/테스트 후 commit하고, push는 해당 세션의 사용자 지시에 따른다.
4. SDK 두 repo 및 elevation_mapping_cupy의 commit, dirty patch, submodule 상태도 기록한다. 미반영 파일이 있는 SDK를 원격 HEAD로 대체하지 않는다.
5. 데스크톱의 Python/패키지/CUDA/컴파일 의존 버전은 참고용으로 저장하되 x86 conda 환경/build/CUDA 바이너리를 Jetson에 복사하지 않는다.

## 현재 동작과 기준선

- controller 초기: sport 상태 조회 -> 필요 시 StandDown -> down 확인 -> sport 해제 -> 경쟁 LowCmd 없음 확인.
- 1=FixStand, 2=Policy, 3=제어된 StandDown, 0=Passive damping (엎드리는 motion 아님).
- Ctrl+C=Stand -> StandDown -> 실제 down 안정 확인 -> 종료. 프로세스 crash/전원단절/무선단절까지 보장하지 않는다.
- a/d=누르는 동안 base 전방 기준 ±15도, 해제 시 0. q/e=지속되는 body-relative ±10도 누적, 계속 회전할 수 있음. w=방향0, i/k=속도.
- bias 미수신/미보정/stale이면 2 진입 금지. Policy 진입 시 bias 복사 후 고정. bridge는 주기적으로 bias 상태 발행, 새 세션 미보정 상태는 대기 candidate 무효화.
- leg calibration은 연속 정지 조건 약10초 충족 필요. 기립 직후 10초가 무조건 충분한 것은 아님.

최신 기준선: captures/20260917_131652_267037285.jsonl, /tmp/go2_ctrl_20260917_131649_452658918.log.
Policy 13:17:47.554~13:17:51.742 KST, 4.188초. scan41개, 10.01Hz, 최대 간격117ms, cloud 수신→map 출력 p95 84.4ms/max92.2ms. 구간 내 bridge fault 없음. gyro bias [-0.0068460887,-0.0051443926,0.0045797525] rad/s가 bridge/controller 일치.
leg 추정 yaw -55.2도, 높이 -9.55cm. 이는 추정값이며 ground truth 정확도가 검증된 것은 아니다.
시각화: captures/review_20260917_131652/index.html.

## 배치할 폴더

공통 deploy 소스는 unitree_rl_lab 구조를 우선 유지한다. 대규모 정리는 이식과 섞지 않는다.

Jetson:
- unitree_rl_lab/deploy/parkour (tools, em_sidecar, vendored, contract 등 상대 import 의존 포함)
- unitree_sdk2_python + CycloneDDS Python/native 런타임
- 독립 경로의 elevation_mapping_cupy 소스 (중첩 import/설정/커널 포함)
- Jetson용 전용 Python 환경

노트북:
- unitree_rl_lab/deploy (robots/go2, 공통 include, parkour include/contract 및 필요한 thirdparty)
- unitree_sdk2 C++ 및 CycloneDDS native 런타임
- ONNX Runtime x86_64 C++ (현재 CMake는 deploy/thirdparty/onnxruntime-linux-x64-1.22.0 경로 참조)
- Boost program_options, yaml-cpp, Eigen, fmt, X11 및 개발 헤더

IsaacLab/Isaac Sim 설치는 필요 없다. backend는 이미 deploy/parkour/vendored/elevation_map_backend.py를 사용한다. 외부 elevation_mapping_cupy만 별도로 필요하며 --emcupy-root로 지정한다. 현재 기본값은 ~/workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy이므로 새 배치에서는 명시적으로 덮어쓴다.

## 단계 A: 로봇 전원 없이 준비

1. 위 소스 스냅샷/버전 목록/모델 hash 준비.
2. 노트북에서 SDK 및 go2_ctrl 새 build. 과거 데스크톱 build 디렉터리를 복사하지 않는다. CMake 절대경로, 모델 경로, 공유라이브러리 탐색 경로를 확인한다.
3. CTest, 키보드 PTY/hold-release 테스트, ONNX offline trace 검사. 기존 검증 명령은 repo에서 확인하여 실행한다.
4. Wayland에서 XWayland 호환을 추정하지 않는다. 먼저 실제 X11 세션에서 a/d 누름/해제/동시누름/포커스이탈을 모터 출력 없이 검증한다. --keyboard-check의 범위를 코드에서 확인하고 부족하면 별도 read-only 입력 진단을 사용한다.
5. Jetson용 설치 후보/의존 파일을 준비한다. Python3.8 문법/라이브러리 호환성과 C++ 표준/ABI를 점검한다. 구체적인 Torch/CuPy wheel은 L4T/CUDA 실측 뒤 NVIDIA/CuPy 공식 자료에서 해당 버전의 aarch64 지원을 확인한다. 최신 pip 패키지를 무조건 설치하지 않는다.
6. 로컬 저장 실행 스크립트와 로그 수집 경로 설계. Jetson bridge 로그는 Jetson 로컬, controller 로그는 노트북 로컬에 기록한다.

## 단계 B: Jetson 연결 후 환경 조사/설치 (구동 명령 없이)

1. SSH 접속 후 nvcc/CUDA 실제 버전, NVIDIA 드라이버, 기존 Torch/CuPy, Python 환경, 자동실행 서비스/컨테이너, NIC/라우팅을 기록한다.
2. 기존 로봇 서비스와 시스템 Python을 덮어쓰지 않고 독립 환경을 만든다. OS/JetPack 업그레이드는 이번 기본 경로에 포함하지 않는다.
3. 맞는 aarch64 GPU 패키지 설치. Torch CUDA 텐서, CuPy 커널, DLPack 상호운용, elevation map update/sample, 메모리 사용을 단계별 smoke test한다.
4. bridge import 및 관련 unit tests. calibration/IMU frame/스캔 배열 순서/좌표 변환은 데스크톱과 동일한 소스로 검증한다.

## 단계 C: 유선 분산 실행으로 기능 확인

1. 출력 없는 DDS 수신 probe로 Jetson eth0에서 LowState와 raw cloud, 노트북 유선 NIC에서 LowState가 오는지 확인한다.
2. Jetson에서 bridge 실행. notebook에서 rt/parkour/scandots와 rt/parkour/gyro_bias 수신을 확인한다. 도메인0 및 IDL/SDK 호환성을 확인한다.
3. 모터 송신 없이 raw LowState 수신율/공백, cloud 수신율, scan 약10Hz, 처리 지연/최대 공백, bias calibration/heartbeat/session 변경, CPU/GPU/RAM/온도 로그를 수집한다.
4. bridge 재시작, controller 수신부 재시작은 우선 loopback/probe로 확인한다. Policy 중 bias snapshot 고정과 비Policy 시 candidate 갱신을 검증한다.
5. Jetson의 현재 bridge에는 LowState20ms, cloud/map200ms 등의 deadline이 있다. 느리다고 timeout부터 늘리지 말고 원인/최악지연을 측정한다.

## 단계 D: Go2 AP 무선 검증

1. SSID/인증정보는 실제 장비에서 확인. 노트북을 AP에 연결한 뒤 IP/route/선택 NIC 확인. wlo1 주소를 기존 LAN 값으로 고정하지 않는다.
2. SSH와 별개로 노트북↔Jetson, 노트북↔control board의 DDS discovery/unicast/multicast 도달성을 수신 probe로 검증한다. AP가 controller subnet으로 중계하는지 확인한다.
3. 단순 --network wlo1만으로 해결된다고 가정하지 않는다. 도달성에 따라 CycloneDDS interface/peer 설정 검토. 단순 주소 추가로 없는 라우팅을 해결할 수는 없다.
4. 유선 단계와 같은 측정을 최소 수분간 반복. map10Hz뿐 아니라 controller용 LowState 연속성, 전송지연과 burst/gap, AP 신호/거리/부하를 비교한다.
5. DDS receipt age는 송신→수신 지연을 직접 보장하지 않는다. 호스트별 monotonic_ns는 직접 빼지 않는다. NTP/chrony 상태·시간 offset을 기록하고 wall+monotonic anchor를 각 로그에 남겨 분석 가능하게 한다. LowState tick 등 공유 source ID도 활용한다.
6. 통신 단절 시 로봇 측 watchdog/동작을 문서와 코드로 확인한다. laptop의 Ctrl+C graceful shutdown은 Wi-Fi 연결이 끊어지면 전달이 보장되지 않는다. 구동 중 의도적인 단절 실험은 이 단계에서 하지 않는다.
7. AP로 LowState/LowCmd 경로가 확보되지 않거나 지연이 부적합하면 유선 구성으로 유지. controller를 Jetson에 두는 대안은 목표 구조 변경이므로 별도 결정한다.

## 단계 E: 실제 제어 단계 검증

1. 동시 LowCmd 송신 프로세스가 없는지 확인. 기존 startup StandDown/handoff, down 확인, watchdog 관련 조건을 유지한다.
2. 첫 실행은 1(기립) -> 보정 상태 확인 -> 3(StandDown), Ctrl+C 정상 종료만 확인한다.
3. 그다음 1 -> 2 짧은 직진 -> 1 -> 3. bridge/controller 로그를 모두 저장한다.
4. 마지막에 a/d 짧은 hold/release를 확인. q/e 누적 방향 입력은 이식 동등성 평가에서 섞지 않는다.
5. 동일한 조건에서 데스크톱 기준선과 비교: scan rate/gap, controller inference/control timing, bias latch, 키 release, fault, 종료 동작. 궤적/지도 검토는 정확도 측정과 구분한다.

## 최종 명령 형태 (설치 경로 확정 후 실제 스크립트로 제공)

Jetson의 parkour 디렉터리와 전용 Python 환경에서:
python tools/go2_sensor_bridge.py --interface eth0 --odom leg --publish-scandots --duration 0 --emcupy-root <독립 elevation_mapping_cupy 경로>

노트북의 go2/build에서 (무선 수신 검증 완료 후):
./go2_ctrl --network wlo1 --keyboard

로그는 각 장비에서 timestamp 이름으로 저장. 세션을 구분하는 run ID와 hostname도 남긴다. 로컬 로그를 종료 후 한 폴더로 수집한다. bridge 실행만으로 로봇을 구동하지는 않지만 go2_ctrl 시작은 실제 handoff/LowCmd에 영향을 준다.

## 완료 산출물

- 양쪽 설치/빌드/실행 명령과 버전 manifest, 모델/소스 hash.
- timestamp 로그 실행 스크립트, 양쪽 로그 수집 스크립트.
- 모터 출력 없는 DDS/키보드 진단 명령.
- 유선 분산과 AP 무선 각각 측정 결과, 지원되지 않는 경로의 명시.
- 1/3 및 짧은 Policy 테스트 결과, 종료/입력 동작 확인, 남은 제한.

## 다음 세션에 전달할 요청

이 문서를 읽고 Jetson bridge + Galaxy Book4 Pro controller 이식을 진행해줘. 먼저 최신 미커밋 코드 보존과 노트북 오프라인 빌드/입력 테스트부터 시작해줘. Jetson 설치 전 실제 CUDA/Python/서비스를 조사하고 버전별 공식 자료로 패키지 호환성을 확인해줘. 유선 분산 검증 후 Go2 AP DDS를 수신 전용으로 검증하고, 실제 로봇 구동 전에는 그 측정 결과를 보고해줘. 현재 알고리즘과 SDK 역할을 유지하고, Point-LIO나 OS 업그레이드는 진행하지 마.
