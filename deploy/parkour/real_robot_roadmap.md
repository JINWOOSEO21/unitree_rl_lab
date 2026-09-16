# Go2 실기 전환: 연결 시간을 최소화하는 실행 계획

작성일: 2026-09-15. 목표는 PC에서 ONNX 정책을 실행하고 실제 Go2와 DDS로
상태/명령을 주고받는 것이다. Jetson에 정책을 배포하는 작업은 현재 범위가 아니다.

## 2026-09-16 재부팅 후 수신 전용 확인

- 실제 `CheckMode()` 응답: code 0, form `0`, name `mcf` (활성 controller).
  첫 조회 시 이미 기립 상태였다. 이번 관측으로 부팅 직후~기립 전의 mode 전환
  시점까지 확인한 것은 아니다. StandDown/ReleaseMode/LowCmd는 실행하지 않았다.
- LowState 약 500Hz, 배터리 SOC 원시값 66, 전압 약 29.85V.
  기립 상태 foot_force 범위 SDK 순서: [39..40, 41..42, 42..43, 42].
- bridge 수신 전용 60초 처리에서 정상 scan 597개. 첫 GPU 계산 272ms로 인한
  `cloud_too_old_after_mapping` 1회는 거절·지도 초기화되었다.
  시작 후 10초를 제외한 출력률 9.9987Hz, 간격 p50 100.03ms / p95 112.44ms /
  max 131.54ms, 150ms 초과 간격 0회. 모든 scan 값은 finite 및 [-1,1] 범위였다.
- 단, 정지 중 leg 위치 변화는 [-0.1251, +0.1575, -0.00568]m로 수평 약 20.1cm다.
  측정 관절각 최대 변화는 0.00076rad였다. scan 셀별 전체 시간 범위의 p95는
  정규화 높이 0.3274로, 10Hz 출력률 통과를 지형 정확도/안정성 통과로 볼 수 없다.
  근거: 로컬 `captures/live_standing_map_20260916/summary.json`, `events.jsonl`.
- Jetson 부팅 로그에서 maum_ral_ros/maum_ral_sio가 자동 시작하고 네트워크 초기화
  실패 후 재시작하여 부팅 약 19.8초에 SportClient ready가 된 것을 확인했다.
  읽은 로그에는 자동 기립을 일으킨 구체적인 명령 출처가 드러나지 않았다.
  서비스 변경이나 원격 파일 수정은 하지 않았다.

## 2026-09-15 기록: sport 해제, 원시 점군 + leg odometry 연결

- 사용자가 SDK StandDown 후 엎드림을 확인한 뒤 ReleaseMode를 실행했다.
  반환 코드 0, CheckMode name 빈 문자열을 해제 직후와 15초 측정 종료 시 확인했다.
  해제 후 lowstate 7,488건/raw cloud 229건은 수신됐지만 sportmodestate,
  robot_odom, robot_pose, cloud_base, cloud_deskewed는 모두 0건이었다.
  근거: `captures/sport_release_20260915T171531/summary.json`.
- `tools/go2_sensor_bridge.py`는 이제 `rt/utlidar/cloud`를 구독하고,
  `em_sidecar/go2_cloud.py`의 검증된 고정 변환으로 base_link XYZ를 계산한다.
  shadow runner의 실시간 경로에도 이 변경이 적용된다. 오프라인 기존 cloud_base
  replay 및 시뮬레이션 sidecar의 입력 경로는 유지한다.
- 실기 기록 두 개의 대응점 215,034개에서 최대 오차 2.4821e-6m 미만.
  sport 해제 후 기록 229프레임/905,760점 변환 성공. 저장된 odometry를 사용한
  3프레임 GPU 지도 시험에서 모두 finite한 132 scan을 생성했다.
  근거: `captures/raw_cloud_base_validation/summary.json`.
- 원시 점군의 모든 finite 점을 변환한다. 기존 cloud_base publisher의 점 선택,
  자체 몸체 필터, 이동 왜곡 보정을 재현한 것은 아니므로 지도까지 동일하다는
  의미는 아니다. 물리 base/URDF 높이 정합 문제도 남아 있다.
- 기존 `LegOdometry` 알고리즘은 유지하고 `tools/go2_leg_pose.py`로 연결했다.
  bridge와 shadow runner의 실시간 기본값은 `--odom leg`다. 이 모드에서는
  lowstate와 raw cloud만 구독하며 robot_odom/sportmodestate를 구독하지 않는다.
  비교용으로 `--odom robot`을 지정하면 robot_odom을 사용한다.
- leg 위치 원점은 시작 시 0, 방향은 LowState IMU 기준이다. sport 초기 위치나
  시뮬레이션 IMU-site offset을 적용하지 않는다. 관절/발 순서는 deploy.yaml에서
  읽는다. 기존 지도의 초기화부터 이 좌표계를 사용하며 도중 source 전환은 없다.
  중복 tick은 pose 갱신에서 제외하고 시계 역행/100ms 초과 공백은 재시작을 요구한다.
  신뢰 접촉 없이 hold 구간까지 지난 pose는 지도에 사용하지 않는다.
- 10초 실제 수신 시험: lowstate 4,993건, 약 499.2Hz. 엎드린 상태의 원시 발 센서
  값 8~12가 기준 20보다 낮아 지도 출력은 0건(지지 발 부족 99회/pose 지연 1회).
  정지 위치 0 유지가 정확도를 증명하는 것은 아니다.
  기록: `captures/leg_bridge_20260915T172829/summary.json`.
- 이전 기립 기록을 leg로 재생하면 수평 위치가 약 8초 동안 2.12cm 이동하는
  드리프트가 남는다. 같은 기록의 raw+leg GPU 지도는 80 tick 중 78개 처리했고,
  50Hz 합성 시간 ONNX 재생은 393 tick 중 383회 추론(입력 미도착 7/중복 3 차단).
  실기 보행 정확도나 실시간 추론 주기를 검증한 결과는 아니다.
  기록: `captures/leg_odometry_integration/`.
- cloud_base 선택 분석: 기립 기록은 raw의 32.48% 유지/67.52% 제외,
  엎드린 기록은 16.28% 유지/83.72% 제외. 두 기록 모두 base 원점으로부터
  0.25m 이내 점을 모두 제외했고, 3m 이상 점은 모두 유지했다. 정확한 내부
  필터 구현은 확인하지 못했다. `captures/cloud_selection_analysis/` 참조.
- 이번 변경은 로컬 코드와 저장된 기록으로 검증했다. go2_ctrl/정책 모터 송신은
  실행하지 않았으며 sport mode를 다시 켜지 않았다.

### 수신·추론 전용 실행 (데스크톱)

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/shadow_go2_policy.py \
  --interface enp42s0 --odom leg --duration 30 \
  --output-dir "captures/leg_shadow_$(date +%Y%m%dT%H%M%S)"
```

모터 명령을 보내지 않는다. 엎드려 지지 접촉이 없으면 추론도 차단된다.
`--leg-contact-threshold` 기본값은 원시 foot_force 기준 20이며 하드웨어 보정은
아직 하지 않았다. 발 반지름도 기존 추정기 기본값을 사용한다.

### 방향·정지 입력의 현재 구현

Go2 컨트롤러에 `--keyboard`를 지정하면 해당 프로세스의 데스크톱 터미널에서
키를 읽는다. sim2sim의 키보드는 MuJoCo가 lowstate.wireless_remote에 값을
넣었던 방식이며, 새 실기 입력은 수신 LowState를 위조하지 않고 정책 명령과
FSM 전환에 직접 연결한다. 옵션이 없으면 조종기 입력을 사용한다.

| 키 | 요청 |
|---|---|
| `1` | 현재 측정 관절각에서 기립 목표로 보간 |
| `2` | 기립 완료·목표 관절 오차·자세·지도 조건을 통과하면 Parkour 진입 |
| `Space` | 정책 중이면 기립으로 전환; 속도/회전 입력 초기화 |
| `0` | Passive 감쇠 제어로 전환; 엎드리기 동작과 다름 |
| `w/s` | 전진 속도 입력 증가/감소 |
| `q/e` | 좌/우 회전 입력 변경 |

`3`은 미할당이다. 별도 LieDown 상태는 제거했으며 `0`의 Passive 동작은 유지한다.

방향/속도 입력은 키를 놓아도 유지되며 기립/Passive 진입 시 초기화된다.
정책의 전진 속도 범위 0.3~0.8m/s는
유지하며, 후진·횡이동 입력은 없다. `Space`는 정책에 속도 0을 넣는 방식이 아니다.
기립 보간 시간은 기본 2초이며, 정책에서 기립으로 갈 때 기존의 엎드린
중간 자세를 거치지 않는다. 이것은 실기에서 즉시 정지하거나 보행 중 균형을
보장한다는 의미가 아니며, 실제 모터 동작은 별도 검증이 필요하다.

키 인식만 점검하는 명령(DDS 및 모터 명령 없음):

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --keyboard-check
```

실제 제어 실행 형식은 `./go2_ctrl --network enp42s0 --keyboard`다. 이 실행은
다음 시작 절차를 통과한 뒤 LowCmd를 생성한다.

1. `CheckMode()` 성공 여부와 활성 controller를 확인한다.
2. 활성 mode가 있으면 `StandDown()`을 호출하고 반환 코드 0을 요구한다.
3. 새 LowState에서 엎드림 목표 관절각, 낮은 관절속도, 정상 IMU 자세가
   0.5초 유지되는지 확인한다. 엎드림 대기 시간은 약 15초로 제한한다.
4. mode 재조회 및 현재 자세 재확인 후 `ReleaseMode()`를 호출한다.
5. mode 비활성, 다른 LowCmd 스트림 중단, 최신 엎드림 자세를 확인한다.
6. 자체 LowCmd와 Passive FSM을 시작한다. 키보드 `1`/`2`는 그 이후 사용한다.

관절 허용 오차는 hip 0.50rad, thigh/calf 0.25rad, 관절속도는 0.20rad/s,
IMU 기울기는 0.60rad 이하다. 기존 실측 엎드림의 뒷다리 hip 벌어짐 약 0.38rad를
허용하면서 기립의 thigh/calf 각도와는 구분한다. 이는 센서 기반 관절 자세 판정이며
몸통이 실제 바닥에 지지되었음을 직접 측정하는 것은 아니다.
조회/RPC 오류, 자세 확인 실패, 센서 정지, 취소, 제어 mode 변경/재활성화,
다른 LowCmd 잔존 시 자체 모터 publisher 생성 전에 종료한다.
처음부터 mode가 비활성이어도 엎드림 확인은 수행한다.

이번 구현/검증에서는 실제 controller를 실행하지 않았으므로 자동 StandDown 및
ReleaseMode의 실제 로봇 동작 검증은 남아 있다. Ctrl+C는 프로그램 종료이며
자세 전환 명령이 아니다. `3`번 LieDown 상태를 다시 추가한 변경도 아니다.
시작 절차를 포함한 CTest 4/4와 C++ 빌드가 통과했다. 실제 과거 엎드림 관절값은
판정을 통과하고 기립/기울어진 자세, RPC 실패, 센서 정지/역행, 제어 주체 변경,
재활성화, 경쟁 LowCmd 및 취소는 모의 입력 테스트에서 거부되는 것을 확인했다.

MotionSwitcher 서비스가 없는 MuJoCo는 명시적으로
`./go2_ctrl --network lo --sim --keyboard`를 사용한다. `--sim`은 loopback `lo`에서만
허용되며 실제 로봇 인터페이스에서는 DDS 초기화 전에 거부된다.

### raw + leg 지도 → 컨트롤러 DDS 연결

`go2_sensor_bridge.py --publish-scandots`가 raw cloud를 측정 변환으로 base에 옮기고,
자체 leg odometry로 GPU 지도를 갱신한 뒤 `rt/parkour/scandots`에
`unitree_go::msg::dds_::HeightMap_`를 발행한다. 12×11의 정규화된 132개 값과
x-fast 순서는 기존 정책 계약을 따른다. 옵션을 생략하면 기존처럼 센서 수신만 하며,
shadow 정책 실행에도 DDS 발행이 자동으로 추가되지 않는다.

터미널 1 — 센서 수신·지도 발행(모터 명령 없음):

bridge의 `--duration` 기본값은 0(시간 제한 없음)이다. 수신 점검을 일정 시간만
진행하려면 `--duration 60`처럼 지정한다. shadow runner는 별도의 기본 30초를 유지한다.

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour
/home/seo-jinwoo/miniconda3/envs/env_isaaclab/bin/python tools/go2_sensor_bridge.py \
  --interface enp42s0 --odom leg --publish-scandots --duration 0 --summary-only
```

터미널 2 — 실제 모터 컨트롤러:

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network enp42s0 --keyboard
```

`1`로 기립을 요청하고 기립·관절·자세·지도 조건이 충족되면 `2`로 정책에 진입한다.
엎드린 기록의 foot force는 접촉 임계값 20보다 낮았으므로 그 상태에서 지도 발행이
차단되는 것은 정상이다. 실제 기립 후에도 접촉 조건 충족 여부를 확인해야 한다.
요약의 scan 카운터는 생산/발행 횟수이며 컨트롤러의 수신 확인 응답은 아니다.

원본 cloud 수신 후 200ms, 새로운 LowState 수신 후 20ms를 넘거나 leg 지지가
유효하지 않으면 발행을 중단한다. GPU 계산 중 발생한 오류도 결과 발행 전에 재검사한다.
이미 발행한 지도는 빈 메시지로 무효화하며, C++ 수신기는 빈 값·잘못된 길이·NaN·범위
초과를 받으면 기존 지도를 즉시 무효화한다. 정책은 신선도와 값을 같은 잠금 아래 읽고,
유효한 지도가 없으면 추론하지 않는다. DDS 단절 시에는 기존 시간 초과 감시가 적용된다.

검증: Python 64개 테스트 통과. 저장된 실제 센서를 `lo`, DDS domain 179에서 재생해
유효한 지도 74개와 입력 종료 후 무효화 1개를 수신했다
(`captures/scandots_dds_replay_01/summary.json`). 실제 C++ 수신기로 정상 수신 5회와
빈 값·NaN·범위 초과·길이 오류·종료에 대한 무효화 5회를 확인했다
(`captures/scandots_cpp_check/summary.json`). 이 검증에서는 로봇 연결이나 모터 명령을
사용하지 않았다. 실제 보행, 이동 중 leg odometry 오차, raw 점군의 자기 몸체 제거 및
기록된 약 35mm 높이 차이의 물리적 검증은 아직 남아 있다.

검증: go2_ctrl 빌드, CTest 3/3(키/상태/보간·비TTY 거부·실제 PTY 입력)을 통과했다.
PTY에서는 무입력 중 누적값 유지, Delete escape sequence의 숫자 명령 오인 방지,
종료 시 터미널 복원을 확인했다. 로봇의 실제 보행→기립 전환은 실행하지 않았다.

### 점군 출처 비교 뷰어

`/home/seo-jinwoo/바탕화면/2026-09-17/lidar_observed_base.html`에 출처 선택을 추가했다.
기존 파일은 이름과 달리 이미 raw를 측정 고정변환으로 바꾼 점군이었다.
이제 실제 발행 `cloud_base`(163,572점)와 `cloud → base_link`(513,956점)를
명시적으로 선택한다. 동일 기록 123프레임에서 전자의 모든 점을 후자와 대응시켰고
최대 좌표 오차는 2.45µm였다. 두 모드의 로봇 모델·축·점 수를 브라우저에서 검증했다.
모드 전환 시 페이지를 다시 불러오므로 카메라/필터 설정은 초기화된다.
갱신 도구는 `tools/build_lidar_observed_base_modes.py`이며 기본 재실행은 멱등적이다.

## 이전 단계의 실행 기록

### 2026-09-16 커밋 전 재검증

- `tools/tests` unittest 65개, `em_sidecar/tests` unittest 10개 및 기록 분석 unittest 4개 통과.
- Go2 C++ 빌드 및 CTest 3/3 통과. 실제 controller/모터 출력은 실행하지 않았다.
- 기록 센서 → GPU 지도 → DDS 재생은 `lo`, domain 179에서 정상 scan 75개와
  종료 후 무효화 1개를 수신했다. 값은 모두 finite 및 [-1,1] 범위였다.
  원본 기록과 로그는 로컬 `captures/prepush_dds_reset_20260916/`에 있으며 Git에서 제외한다.
- GPU 계산 이후 거절된 갱신이 누적 지도에 남지 않도록 map clear를 추가했다.
  지연·입력 오류·scan 계약 위반 회귀 테스트 및 실제 GPU layer/variance 초기화를 확인했다.
- 별도 정확도 검사 `python -m em_sidecar.tests.test_leg_odometry`는 **FAIL**이다.
  첫 에피소드의 0.1초 증분 수평 오차 평균 0.92cm가 검사 기준
  `0.05 * (path_len / n_ticks) + 0.005 m`(약 0.88cm)를 넘었다.
  첫 에피소드 scale -1.69%, 3.2m 창 p95 19.1cm는 각 기준을 통과했고,
  전체 z 드리프트는 21.3cm였다. 두 번째 에피소드는 세 검사 기준을 통과했다.
  해당 추정기와 정확도 검사 코드는 이번 변경에서 수정하지 않았다.
- 따라서 입력/통신 검증 통과를 odometry 정확도 검증 완료로 해석하면 안 된다.
  실기 발 접촉 임계값, 발 반지름, 미끄러짐·도약 구간 및 IMU yaw 오차 검증이 남아 있다.

- 원본 15초 센서 기록 및 재연결 시 8초의 raw/base/odom 점군·상태 기록을 확보했다.
- 잘못된 시뮬레이션 장착 가정에 기반한 비교 화면·중간 지도·진단 도구는 정리했다.
- raw→cloud_base의 실제 publisher 변환은 별도 프레임 검증으로 재현했다.
- policy FK와 Jetson URDF는 일치하지만 점군 바닥과 모델 발바닥 사이 약 35mm 차이가 남는다.
- LowState와 LiDAR odometry는 이번 정지 기록에서 world yaw 기준이 약 1.787° 달랐다.
- cloud_base 직접 입력 + robot_odom 기반 지도·132 scan 오프라인 재생 완료.
  10Hz 80 tick 중 79개 처리(최초 odom 미도착 1개 제외), 모든 scan finite/범위 정상.
  마지막 scan은 직접 관측 근거 110칸, 상한만 11칸, 미관측 11칸이다.
  결과는 `captures/cloud_base_replay` 세 파일에 저장했다.
- 50Hz 실제 상태/scan의 오프라인 ONNX 재생 완료: 7.84초 393회 추론,
  prop53/history530/action12 전체 finite, action clip 초과 0개.
  C++ golden 100프레임 재검증 통과: prop 최대오차 2.384e-7, history/지연 0,
  ONNX 최대오차 2.027e-6. 결과는 `captures/policy_replay` 세 파일에 저장했다.
- 실패 입력 주입·차단 검증, 관절 목표 점검, 수신·추론 전용 경로 준비 완료.
  새 `shadow_go2_policy.py` 경로에서 399 tick 중 초기 입력 미도착 6개를 차단하고
  393회 추론했다. LowState 중복 tick 7개를 검출했다. 기존 모터 제어기의 실패
  처리까지 변경한 것은 아니며, 실제 DDS 수신 시험은 재연결 후 수행한다.
- GPU 지도 반복 실행 원인·영향 분석 완료: 6회 실행에서 scan 최대 차이 0.08620,
  관절 목표 최대 차이 0.01080rad(0.619°). 점 단위 병렬 갱신 순서의 영향은
  진단용 직렬 실행으로 확인했다. 비결정성을 제거한 것은 아니며 backend는 유지한다.
- 다음 실행은 재연결 후 모터 송신 없는 30초 센서·지도·정책 추론 검증이다.
  실기 구동 전 물리 높이 차이와 frame 기준·정지/복구 절차를 별도로 확정한다.
- 실기 명령/정책 실행은 아직 수행하지 않았다. 물리 높이 기준과 모터 경로의 실패 처리는 미검증이다.

원본·검증 사실: [offline_preparation_2026-09-15.md](offline_preparation_2026-09-15.md).
세부 검증 수치는 `captures/frame_inspection_20260915/verified_findings.json`에 통합했다.

## 연결의 의미

- **오프라인**: Go2와 Jetson의 전원을 꺼도 수행 가능한 PC 파일/코드 작업.
- **실기 DDS 연결**: Go2가 켜져 있고 PC `enp42s0`에서 로봇 DDS에 접근 가능한 상태.
  SSH 로그인 자체는 정책 실행이나 DDS 수신의 필수 조건이 아니다.
- **Jetson SSH 연결**: Jetson의 서비스, 로그, 설정 확인/관리 시 필요하다.
  현재 알려진 경로는 PC `192.168.123.99/24` ↔ Jetson `192.168.123.18/24`이다.
  Jetson 전원을 끈 채 로봇 DDS가 계속 접근 가능한지는 확인하지 않았으며 전제하지 않는다.

## 전체 순서와 통과 기준

| 단계 | 연결 필요 | 작업 | 다음 단계로 넘어갈 근거 |
|---|---|---|---|
| A. 지금 자료 확보 | DDS; 기존 SSH 조사 자료 재사용 | LowState, SportModeState, 원본 PointCloud2를 공통 PC 단조 시각과 함께 짧게 저장; 환경/정책 계약 보존 | 세 토픽 파일이 모두 있고 전체 점군을 재디코딩 가능; 누락/시각 범위 확인 |
| B. 오프라인 입력 검증 | 없음 | 원본 점군 디코딩, 정지 IMU/관절/발 센서 분포, 토픽 간 수신 시각 차이 분석 | 오류와 미확정 교정값을 수치로 구분; 잘못된 입력을 정상으로 표시하지 않음 |
| C. 오프라인 지도·관측 | 없음 | cloud_base + robot_odom → elevation map → scan 132; prop 53/history 530 회귀 | 추가 LiDAR 회전 없이 점군 처리; 평면 높이 미확정 항목과 관측 범위를 명시; history와 순서/스케일을 golden trace로 검증 |
| D. 오프라인 정책·실패 처리 | 없음 | 기록 재생으로 ONNX 출력/관절 목표 분석; stale/NaN/누락/재시작 처리 설계 및 테스트 | 모터 송신 없는 재현 가능한 결과; 목표/상태 이상 시 실행 차단; 테스트 통과 |
| E. 재연결: 송신 없는 실시간 검증 | DDS, 조사 시 SSH | 현재 상태 재조회; 실시간 센서→관측→추론을 출력 저장만 하며 실행 | 수신/추론 신선도·지도 품질 확인; 실제 전송 경로와 분리된 실행파일 |
| F. 제어권·정지 준비 | DDS + 필요한 경우 SSH, 현장 준비 | error_code=100 의미 확인; 자동 기립/외부 명령 경로 확인; 정지·복구 수단 확보; 기존 제어기 정리 절차 확정 | 명령 주체 하나, 저수준 제어에서도 작동하는 중단 경로, 전환/복구 절차 확인 |
| G. 제한된 첫 모터 시험 | 실제 로봇/현장 필요 | 검증한 제어권 전환 후 자세 유지 등 작은 범위부터 시험; 명령 제한/중단 동작 확인 | 목표 추종·중단·복구 실측 확인 후 확대 |
| H. 평지 보행 → 지형 | 실제 로봇/현장 필요 | 작은 보행 범위부터; 각 시험 로그를 PC에 저장 후 전원을 끄고 분석 | 이전 단계의 실측 오류를 해결한 뒤 난도 상승 |

B–D는 PC에서 진행한다. D의 기록 기반 추론은 출력 action이 실제 로봇에 적용되지
않은 **가정 조건의 계산**이다. last_action/history가 이어지더라도 실제 폐루프
동작이나 보행 성공을 검증하는 것은 아니다. cmd_vx=0 입력만으로 정지 성능을
보장하지 않으며 현재 Parkour 경로의 최소 전진 지령도 별도로 수정/검증해야 한다.

## 지금 확보할 자료와 한계

- LowState 전체: 모터 상태, IMU, tick, 발 센서, 배터리, remote bytes 등.
- SportModeState 전체: 자세/위치/속도, mode/error, 메시지 시각 등.
- PointCloud2: frame, fields, byte order, point/row stride, 메시지 시각, **원본 data**.
- 모든 콜백의 동일 PC steady clock 수신 시각. 이는 센서 측정 시각이나 네트워크
  편도 지연의 정답이 아니다. 서로 다른 센서 시계의 동기화도 별도로 확인한다.
- 이전 SSH 조사: Jetson 모델/OS, IP, 시작 서비스와 자동 기립 관련 소스 경로.
- PC 코드 revision/dirty 상태, 정책 ONNX와 deploy.yaml/geometry의 해시.
- 사용자가 확인한 현재 환경: **평평한 바닥에 서 있음**.

정지 자세 한 번으로 관절 방향, 발 센서의 비접촉 분포, 이동 중 지연/드리프트,
LiDAR의 6자유도 외부 파라미터, 제어권 해제 후 sport 상태 지속 여부를 모두
측정할 수는 없다. 평면은 방향/높이 오류를 찾는 데 유용하지만 수평 위치와 yaw
교정을 유일하게 정하지 못한다. 이 항목은 재연결 검증으로 남긴다.

## 오프라인에서 먼저 해결할 코드 항목

1. 실제 32-byte PointCloud2를 metadata로 읽는 수정과 원본 기록 회귀.
2. SDK 관절 enum과 `il_to_sdk` 교차 확인; 발 순서는 관절 enum과 별도로 검증.
3. 실기 후보 설정 분리: mount pose, odometry offset, 접촉 판정, 지도 갱신 주기.
   시뮬레이터 값을 확인 없이 실기 기본값으로 승격하지 않는다.
4. pose/cloud 정렬과 신선도 검사. 수신 시각 기준 baseline을 만들되 시계 차이와
   측정 지연을 구분한다. 15 Hz 입력과 학습의 10 Hz 지도 갱신 차이도 평가한다.
5. 송신 없는 재생·추론 경로. 기존 `go2_ctrl`은 시작 시 제어권 전환이 있고
   Passive에서도 LowCmd를 발행하므로 진단용으로 사용하지 않는다.
6. 추후 모터 시험 전 action 범위, 목표 관절 한계, gain, 정책 중단/통신 단절 처리,
   사용자 명령/정지 입력을 별도로 검증한다. 학습 액추에이터 설정은 실기 보증값이 아니다.

## 다음 오프라인 작업의 우선순위

| 순서 | 작업 | 오프라인 완료 기준 |
|---|---|---|
| 1 (완료) | 저장한 LowState + scan의 50Hz ONNX 재생 | 미래 표본을 쓰지 않고 prop53/history530/action12를 생성; golden trace와 관측 규약 비교; 결과의 유한성 확인 |
| 2 (완료) | 실패 입력 주입 및 추론 차단 검증 | 새 receive-only 경로에서 NaN/Inf, 잘못된 길이, LowState/scan 누락, 오래된 지도 반복 수신, 재시작을 넣어 차단·history 초기화 동작 확인 |
| 3 (완료) | action에서 관절 목표까지 점검 | SDK 순서, 지연·clip·scale, 목표 범위와 프레임 간 변화량 검사; URDF 기준 위반 0개, 최초 목표 점프 최대 0.11756rad |
| 4 (준비 완료) | 실기용 수신·추론 전용 경로 | cloud_base/robot_odom 전용 bridge + ONNX 프로세스; 실제 backend 입력 일치 확인; 제어권 RPC 및 LowCmd publisher 없음; 실제 DDS 시험은 미실행 |
| 분석 완료 | GPU 지도 재현성 원인·정책 영향 | 79프레임×6회 및 정책 393 step×6회 비교; 최초 6프레임의 점 갱신 직렬화 3회 오차 0; 직접 관측 셀에도 차이 발생; static 기록의 목표 차이 최대 0.619° |

현 코드의 검사 범위: `ScandotsSubscriber`는 길이 132와 수신 후 경과 시간을
검사하는 기반을 제공하고, Parkour FSM은 scan 0.5초 timeout 시 Passive로 전환한다.
상위 `FSMState`에는 LowState timeout 전환도 있다(설치 SDK 기본값 1000ms;
현재 deploy 코드에서 timeout 재설정 없음). 따라서 timeout 처리가 없는 상태는 아니다.
다만 scan의 NaN/Inf·범위 검사, 원본 지도 시각 기반 신선도 및 오래된 지도의
반복 수신 검출은 이 subscriber에서 확인되지 않는다. FSM 전환이 정책 스레드의
해당 step 출력을 원자적으로 차단하는지도 별도 시험 항목이다.

현 Go2 설정의 중립 지령은 `cmd_vx_min=0.3 m/s`, action 지연은
`action_delay_override=0`이다. 학습 계약에는 지연 1 step이 있으므로 두 값을
구분해 기록한다. 실제 통신/액추에이터 지연은 오프라인 설정값으로 확정할 수 없다.

새 receive-only runner의 guard는 LowState 20ms, scan 수신/원본 점군 수신 이후
500ms를 각각 검사하는 진단 기본값이다. 실제 측정 시각과 DDS 편도 지연을
알아낸 값이 아니다. 이 guard를 기존 `State_Parkour` 모터 경로에 연결하고
통신 단절 시 모터의 실제 반응을 검증하는 작업은 첫 모터 시험 전 별도 단계다.

## 재연결 때만 필요한 체크리스트

- 수신 상태와 error_code 재확인; 현장 담당자에게 정확한 SKU/펌웨어 및 코드 100 문의.
- 자동 기립 원인과 기존 RAL/외부 명령 경로 확인. 현재 원인은 미확정.
- LiDAR 모델/실측 장착 방향·위치, 좌우/전방 비대칭 표적으로 지도 방향 확인.
- 안전하게 준비된 별도 절차에서 발 접촉/비접촉 데이터, 이동 중 odometry 확인.
- 저수준 전환 후 sport odometry가 유지되는지 확인. 유지되지 않으면 검증된 별도
  odometry가 필요하며 정지 시 수신 결과만으로 대체할 수 없다.
- 모터 시험 전 제어권 충돌 제거, 독립 정지·복구 수단 및 감독/시험 공간 준비.

## 전원과 작업 종료

수집 파일의 완결성 확인이 끝나면 이번 오프라인 단계는 연결을 요구하지 않는다.
SSH 종료나 Ethernet 분리는 로봇 전원을 끄지 않는다. Jetson의 OS 종료도 로봇
전체 전원 종료와 동일하다고 볼 수 없다. 서 있는 로봇의 전원 종료는 확인된
제조사 절차와 현장 조작 수단에 따라 수행한다. 에이전트는 원격 종료/자세 변경을
실행하지 않는다.

상세 이전 조사: [hardware_check_2026-09-15.md](hardware_check_2026-09-15.md).

### 2026-09-16: leg odometry startup gyro calibration

`go2_sensor_bridge.py --odom leg` now waits for a 10 s stationary calibration
window before publishing valid leg poses/terrain. Keep the robot supported and
stationary with all four feet loaded during initialization. A bridge restart
requires calibration again. This is sensor processing only; it sends no sport or
motor commands. Policy startup must wait for valid scandots as before.

The window requires all raw foot forces above the configured threshold, joint
excursion <= 0.005 rad, quaternion angular distance from the window start <=
0.01 rad, per-axis |gyro| <= 0.1 rad/s, and gyro standard deviation <= 0.025
rad/s. These are initial deployment thresholds, not a hardware-calibrated proof
of rest. Motion/contact loss restarts the window. Time-weighted gyro mean is
subtracted only in the leg pose adapter; LowState sent to the policy and the
shared simulation estimator are unchanged. Once calibrated, bias remains fixed
so subsequent real rotations are preserved. No automatic bias relearning during
walking, zero-velocity constraint, or gyro replacement is introduced.

`--summary-only` includes `leg_calibration`: `gyro_calibrated`,
`gyro_bias_rad_s`, and `gyro_calibration_elapsed_s` (0 after completion).
If calibration does not finish, inspect contact and sensor stability; maps stay
invalid rather than using an uncalibrated position. Gyro/attitude inconsistency
is evidenced by the saved static record, but firmware preprocessing, sensor
frame alignment, and timestamp latency still need independent verification.
Coherent foot sliding, very slow rotation below the gate, or a frozen/incorrect
quaternion cannot be excluded by these gates. Bias temperature drift and moving
accuracy remain unvalidated.

Offline reproduction (no robot/DDS):

```bash
cd /home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/check_go2_gyro_replay.py captures/live_standing_map_20260916/events.jsonl
```

On this stationary saved subset, ready at 10.093 s, evaluation 49.867 s:
uncorrected replay horizontal displacement 16.257 cm; corrected replay 1.671 cm.
Both are the same replay interval, not the full live 500 Hz stream. Earlier
1.28 cm was a counterfactual correction of the logged live trajectory; it was
not the result of replaying the implemented estimator. Unit tests cover bias
removal, preserving real angular velocity, contact/joint motion restarting the
window, sustained-turn rejection, quaternion sign invariance, and vibration.

### Keyboard 3: controlled low-level StandDown

`3` now requests a separate `StandDown` FSM state from settled `FixStand` only.
This uses SDK2 LowCmd joint targets, not SportClient.StandDown; sport mode remains
released. The controller captures current measured joint positions and uses a
3 s quintic blend (zero endpoint velocity/acceleration) to `FixStand.qs[1]`:
SDK order `[0, 1.36, -2.65]` per leg. Duration is configurable with
`FSM.FixStand.standdown_duration`. Existing stand gains are retained, and the
final down pose remains actively held. No automatic transition to Passive.

Entry requires completed stand interpolation and 0.5 s of advancing LowState
with joint error <= `standdown_tolerance` (0.15 rad), tilt <= 0.3 rad,
and all |joint dq| <= 0.2 rad/s. Source gaps >100 ms reset this dwell. Scandots are not
required. An early press is rejected, not queued: press 3 again once settled.
From Policy press 1 (or space), wait for settled Stand, then press 3. From down,
1 stands again; 2 cannot enter Policy directly. Repeated 3 does not restart the
motion. 0 remains immediate damping-only Passive and interrupts the descent;
it is not a controlled lowering command. DDS timeout retains the existing
Passive fallback. These software checks do not prove ground support or physical
stability; the new motion has not yet been exercised on hardware.

Verification: C++ build, routing/interpolation regression tests, terminal input
PTY tests. No controller execution against the robot during implementation.

Live gyro follow-up: `captures/gyro_live_20260916T131641/summary.json` records
10.024 s calibration, then 60 s horizontal displacement 1.682 mm (maximum
excursion 1.923 mm), LowState 498.48 Hz, terrain 10.016 Hz. There were 100
expected pre-calibration pose rejections and one cold map processing timeout;
no subsequent map faults. IMU yaw changed -0.738 degrees, so orientation drift
and walking accuracy remain open despite the improved stationary position.

StandDown tolerance follow-up: measured SDK joint 11 settled near -1.602 rad
against -1.500 rad target, exceeding the former 0.1 rad cap despite low speed
and <1 degree tilt. Down entry now has its own 0.15 rad configurable tolerance;
policy entry retains stand_tolerance. Rejection logs report worst joint q/target/
error/limit, largest |dq|/limit, tilt/limit, finite status and dwell readiness.
The running controller must be restarted to use this compiled change; never
terminate or restart it automatically while it supports a standing robot.
