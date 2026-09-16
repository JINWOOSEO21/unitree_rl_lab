# Go2 오프라인 준비: 현재 보존 자료와 판정

## 보존한 원본

- `captures/20260915T022128Z/`: 최초 15초의 cloud.bin/cloud.jsonl,
  lowstate/sportmodestate.jsonl, manifest/provenance 및 당시 코드·정책 snapshot.
- `captures/frame_inspection_20260915/dds_capture/`: 재연결 시 8초 기록.
  raw/cloud_base/cloud_deskewed 각 123프레임, LowState/SportModeState,
  robot_odom/robot_pose 및 manifest.
- `captures/frame_inspection_20260915/jetson_go2_description.urdf`와 출처·해시.
- `captures/frame_inspection_20260915/verified_findings.json`: 좌표 변환과
  별도 프레임 검증 수치, 기구학·바닥 비교, 자세 기준 차이를 통합한 근거.

사용자 요청으로 임시 뷰어, 눈대중 보정 후보, 잘못된 시뮬레이션 장착 가정의
중간 지도, 중복 진단 보고서·스크립트를 제거했다. 원본 14개 파일의 해시를
정리 전후 비교해 동일함을 확인했다.

## 확인한 사실

- 실제 cloud는 point_step=32이며 x/y/z 필드와 stride를 읽어야 한다.
  `em_sidecar/pointcloud.py`의 메타데이터 기반 decoder는 유지한다.
- cloud_base는 base_link 좌표다. raw→base 변환을 다시 곱하지 않는다.
- robot_odom은 odom→base_link이며, 이번 기록에서 sportmodestate와 거의 같다.
- LowState 자세와는 world yaw 약 1.787° 차이가 있어 같은 world pose로 혼합하지 않는다.
- cloud_deskewed는 odom 좌표이며 이번 기록의 모든 프레임 첫 10,000개 XYZ가 0이다.
- 모델 FK와 URDF 기구학은 일치하지만 발바닥–바닥 후보 높이 차이가 약 35mm 남는다.
  이 차이를 임의로 보정하거나 실제 정책 입력의 정확도가 검증됐다고 주장하지 않는다.
- 기존 ONNX golden trace 100프레임 재생은 action 최대오차 2.03e−6으로 통과했다.
  실제 센서 closed-loop 또는 50Hz shadow policy 검증을 대신하지 않는다.

## 시각·안전 범위

최초 기록의 cloud callback 시각은 binary write 이후 기록되어 정확한 센서
동기화 지연을 나타내지 않는다. 재연결 기록은 callback 입구의 공통 PC 단조 시각이다.
Jetson 당시 wall clock은 잘못되어 PC 시각과 직접 비교하지 않는다.

Sport error_code=100의 의미, 실제 로봇 버전과 독립 정지/복구 절차는 미확정이다.
`go2_ctrl`은 초기화 시 제어권 전환/LowCmd 발행이 가능하므로 진단용으로 실행하지 않는다.

## 다음 단계

지도·132 scan 생성, 50Hz ONNX 기록 재생, 입력 오류 차단, 관절 목표 검사,
수신·추론 전용 경로 준비와 GPU 지도 재현성 원인·정책 영향 분석을 완료했다.
다음은 재연결 후 수신·추론 검증이다. 추가 ray self-filter와 미확정 z 보정은 넣지 않았다.
이 경로의 동작 검증과 물리 높이 정확도 검증은 별개다.

전체 실행 순서는 [real_robot_roadmap.md](real_robot_roadmap.md)를 따른다.

## cloud_base 오프라인 재생

재생 도구는 raw LiDAR 장착 회전을 적용하지 않고, `cloud_base`와 같은 시각의
이전 `robot_odom`으로만 점군을 odom에 옮긴다. 지도 visibility 계산에는 관측된
센서 원점만 사용하며, 점 좌표 변환에는 이 원점 이동이 상쇄된다.

```bash
/home/seo-jinwoo/miniconda3/envs/env_isaaclab/bin/python tools/replay_go2_base_scan.py \
  captures/frame_inspection_20260915/dds_capture \
  --emcupy-root /home/seo-jinwoo/workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy \
  --rate fixed10 --output-dir captures/cloud_base_replay
```

실행 위치는 `unitree_rl_lab/deploy/parkour`다. 결과는
`captures/cloud_base_replay/{summary.json,replay.npz,scan_and_map.png}` 세 파일이다.
`fixed10`은 기록 시각에 따른 100ms 간격의 오프라인 스케줄이며 GPU 처리의
실시간 10Hz 보장을 의미하지 않는다. `--rate native`는 비교용 원래 수신 주기다.

최종 실행: 입력 123프레임, 고정 10Hz tick 80개, 최초 odometry 부재 1개를 제외한
79개 scan 생성. 형상 `(79,132)`, 전체 finite 및 [-1,1] 범위 통과.
마지막 scan은 직접 관측 근거 110칸/상한만 11칸/미관측 11칸이다.
새 경로 테스트 5개와 저장 결과의 인과성·100ms 간격·점군 재사용 방지 검사를 통과했다.

## 50Hz 정책 오프라인 재생

`tools/replay_go2_policy.py`가 최신 과거 LowState와 지도 tick 기준 최신 scan을
선택하고 prop53/history530을 만든 뒤 ONNX를 실행한다. 현재 Go2 설정의 중립
전진 지령 0.3m/s와 action 지연 override 0 step을 사용한다. 학습 계약의 지연은
1 step이며 실제 통신 지연과는 별개다. history는 현재 프레임을 제외한 과거 10개다.
`policy_meta.json`의 오래된 `[t-9 ... t]` 설명도 실제 코드 규약에 맞게 수정했다.

결과는 `captures/policy_replay/{summary.json,replay.npz,policy_replay.png}`다.
추론 결과를 다시 만드는 명령(parkour 디렉터리에서 실행):

```bash
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/replay_go2_policy.py \
  --lowstate captures/frame_inspection_20260915/dds_capture/lowstate.jsonl \
  --scan-replay captures/cloud_base_replay/replay.npz \
  --output-dir captures/policy_replay --skip-plot
```

이 환경에는 matplotlib이 없어 그래프는 env_isaaclab에서 같은 도구의
`plot_replay` 함수로 저장한 NPZ를 읽어 생성했다. 새 의존성 설치는 하지 않았다.

7.84초 동안 393회 추론했으며 관측값/action은 모두 finite, clip 초과는 0개였다.
LowState 수신 후 경과 시간은 중앙값 1.154ms, 최대 5.188ms이고,
scan의 합성 지도 tick 이후 경과 시간은 0–80ms다. 실제 지도 계산 및 DDS 지연은
포함하지 않으므로 실시간 50Hz 처리 성능을 증명하지 않는다.

C++ golden 검사를 다시 빌드·실행한 결과 100프레임 통과:
prop 최대오차 2.384e-7, history/지연·스케일 오차 0, ONNX 최대오차 2.027e-6.
history golden 비교는 학습/배포의 초기 버퍼 채우기 차이가 사라진 t>=11 구간이다.
정책 재생 테스트 6개와 기존 scan 재생 테스트 9개가 통과했다. 저장 결과에서도
20ms 간격, 미래 상태/scan 미사용, 이전 raw action 입력, history의 현재 프레임
제외 및 heading 마스킹, delay0 관절 목표 계산을 별도로 확인했다.

로봇은 기록 동안 정지해 있었고 계산한 action은 로봇에 적용되지 않았다.
따라서 last_action을 재생 내부에서 갱신해도 폐루프 보행 검증은 아니다.
약 35mm 높이 차이, 발 센서 raw threshold 2의 실기 교정은 아직 미확정이다.

## 비정상 입력과 관절 목표 검증 결과

- `tools/policy_input_guard.py`: 관측 필드/형상/finite/quaternion norm 검사,
  수신/원본 시각별 age 검사, 중복 및 역행 source ID 차단. 오류 뒤에는
  history/이전 action/heading을 초기화한다. 시각 역행은 runner 재시작까지 차단한다.
- `tools/shadow_go2_policy.py`: 위 검사를 실제 추론 호출 전 적용하고, 추론 중
  입력 오류나 만료가 발생하면 계산한 목표를 결과로 내보내지 않는다.
  bridge가 종료되거나 ONNX 출력이 비정상이면 종료한다.
- 기록 재생: 399 tick 중 393회 추론, 최초 입력 미도착 6 tick 차단,
  LowState 중복 tick 7개 거부. 시간 상태 초기화는 최초 진입 포함 6회였다.
  결과: `captures/shadow_validation/{summary.json,inference.jsonl}`.
- `tools/audit_go2_targets.py`: 기존 393-step policy replay의 IL→SDK 매핑,
  clip/scale/delay/offset, URDF 각도·속도 기준을 비교했다.
  각도 및 목표 변화율 기준 초과 0개, 최대 step 변화 0.10508rad,
  최대 목표 변화율 5.254rad/s, 최초 실제 기록 자세 대비 목표 점프 0.11756rad.
  delay0/1의 목표 차이 최대 0.31255rad. kp40/kd1이고 계약 effort와 URDF effort는
  12관절 모두 다르다. URDF 속도/effort는 실제 하드웨어 보증값이 아니다.
  결과: `captures/policy_replay/target_audit.json`.

입력 차단은 새 receive-only runner에 적용했다. 기존 `go2_ctrl`/`State_Parkour`의
모터 송신 경로는 이 작업에서 수정하지 않았으므로 실기 모터 안전 기능을
완료했다고 해석하면 안 된다.

## 수신·추론 전용 경로 실행

구성: DDS LowState/cloud_base/robot_odom → `go2_sensor_bridge.py`(GPU 지도)
→ 로컬 JSONL pipe → `shadow_go2_policy.py`(ONNX). bridge는 세 subscriber만
만들고, 모터 writer나 제어권 서비스 client를 만들지 않는다. 출력 목표는 PC 파일에만 저장한다.
PC에 이미 있는 env_isaaclab의 DDS/torch/cupy와 env_sim2real의 ONNX Runtime을
프로세스로 분리해 사용하며 새 의존성은 설치하지 않았다.

오프라인 재실행(결과 덮어쓰기 방지를 위해 새 output-dir 사용):

```bash
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/shadow_go2_policy.py \
  --recording captures/frame_inspection_20260915/dds_capture \
  --output-dir captures/shadow_recheck
```

재연결 후 수신 전용 실행 명령(이번에는 실행하지 않음):

```bash
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/shadow_go2_policy.py \
  --interface enp42s0 --duration 30 --output-dir captures/live_shadow_first
```

`--bridge-python`, `--emcupy-root`, `--domain`으로 환경 경로/domain을 바꿀 수 있다.
수신 없이도 검사한 항목은 SDK 메시지 타입 import, 세 subscriber만 구성하는
mock 실행, 데이터 오류→추론 차단 통합 시험, 원본 점군으로 GPU 지도 계산이다.
실제 DDS 발견/통신과 두 프로세스의 실시간 일정은 재연결 시 측정해야 한다.

## 지도 backend 검증에서 추가로 발견한 사항

새 bridge와 기존 재생 경로의 backend 입력 7종(점군, sensor/base 자세와 위치,
query XY, base Z)은 79프레임 모두 오차 0으로 일치했다.
새 bridge에서 생성한 scan도 79개 모두 finite 및 [-1,1] 범위를 통과했다.

그러나 기존 GPU backend 자체의 반복 실행은 완전히 동일하지 않았다:

- 기존 코드 두 번 실행: scan 최대 차이 0.02585955.
- 저장해둔 결과와 비교: 한 실행에서 최대 0.05632022.
- 새 bridge의 후속 실행: 저장 결과 대비 최대 0.01911047,
  양쪽 모두 직접 관측 지지율이 1인 셀만 비교하면 최대 0.00116092.

GPU 커널의 공유 셀 갱신/atomic 연산이 원인 후보지만 원인 분리는 완료하지 않았다.
학습과 공유하는 vendored backend를 임의 수정하지 않았고, 정확한 출력 재현성을
통과했다고 주장하지 않는다. 이 초기 측정은 shadow_validation/summary.json에
보존했다. 아래 후속 분석에서 발생 원인과 정책 출력 영향을 확인했다.

## GPU 재현성 오프라인 분석 완료 및 재연결 인계

`tools/check_go2_repeatability.py`로 같은 기록의 지도 79프레임을 6회 생성했다.
결과는 `captures/repeatability/{summary.json,comparison.npz,impact.png}` 세 파일에
통합했다. 원본 기록과 live/vendored backend는 변경하지 않았다.

### 원인과 발생 영역

- 6회 실행 간 scan 범위(max−min)의 최대값은 0.08619577이다.
  이전 2회 비교 수치 0.0259는 일반적인 상한이 아니었다.
- 모든 실행에서 직접 관측 지지율이 1인 셀도 최대 0.08619577 차이를 보였다.
  미관측 보완 영역만의 문제로 한정할 수 없다.
- 혼합/upper-bound/관측 여부 변화 영역의 최대 차이는 0.05922374,
  모든 실행에서 미관측인 영역은 차이 0이다.
- 진단용 backend 인스턴스에서 `_add_points`를 동일한 점 순서로 한 점씩
  실행했을 때 최초 6프레임을 3회 처리한 scan 오차는 0이었다.
  같은 6프레임을 병렬 처리했을 때는 최대 0.05922374였다.
- 따라서 이 실험 구간의 비결정성은 점 단위 병렬 갱신 순서에 의존한다.
  커널에는 같은 셀의 variance atomicAdd, upper/valid/time의 공유 쓰기,
  visibility cleanup의 읽기/쓰기가 함께 있다. 어떤 쓰기가 전체 차이에서
  얼마를 차지하는지까지 분리한 것은 아니다. serial 실험은 전체 79프레임이나
  이동 데이터의 결정성을 증명하지 않는다.

### ONNX에 미치는 영향

각 지도에 대해 동일한 LowState 기록으로 393 step을 재생했다.
한 비교는 prop/history를 고정해 scan만 바꿨고, 다른 비교는 각 실행의 이전
raw action과 history를 독립적으로 갱신했다. 둘 다 로봇 폐루프 시험은 아니다.

| 비교 항목 | 6회 실행 간 최대 범위 |
|---|---:|
| prop/history 고정, scan만 변경한 raw action | 0.04175115 |
| 이전 action/history까지 갱신한 raw action | 0.04318970 |
| 관절 목표 | 0.01079738 rad = 0.61864° |

총 6×393 step의 action/관절 목표는 모두 finite였다. action clip 초과,
URDF 관절 각도 초과, URDF 속도와 비교한 목표 변화율 초과는 각각 0개였다.
이 수치는 정지 기록에서 관측한 범위이며, 이동·다른 지형의 최대 영향이나
실제 로봇의 안정성 보증으로 일반화하지 않는다.

### 결정

수신·추론 전용 현장 검증으로 넘어가기 위한 오프라인 분석은 완료했다.
관측한 차이를 숨기거나 학습 backend를 임의로 바꾸지 않는다. serial 처리는
진단 도구 안에서만 사용하고 실제 bridge에는 적용하지 않았다.
실제 모터 시험 전에는 이동/지형 기록에서 지도 오차와 정책 민감도를 다시
평가하고, 필요하면 학습·배포 양쪽의 지도 갱신 규약을 함께 수정해야 한다.

재현 명령(parkour 디렉터리에서 실행, maps는 기존 분석 결과를 재생성함):

```bash
/home/seo-jinwoo/miniconda3/envs/env_isaaclab/bin/python tools/check_go2_repeatability.py maps
/home/seo-jinwoo/miniconda3/envs/env_sim2real/bin/python tools/check_go2_repeatability.py policy
/home/seo-jinwoo/miniconda3/envs/env_isaaclab/bin/python tools/check_go2_repeatability.py plot
```

전체 tools/tests 테스트 50개 통과. 분석 도구의 실행 간 범위 축과 진단 커널의
점 순서/공유 버퍼 유지도 별도 테스트했다.

### 로봇을 켠 뒤 진행할 순서

1. PC Ethernet 및 Jetson/로봇 접근 상태를 읽기 전용으로 확인한다.
2. 기존 모터 제어기를 시작하지 않고, `shadow_go2_policy.py --interface ...`로
   30초의 수신·추론 전용 결과를 새 디렉터리에 저장한다.
3. 실제 LowState/점군/odom 시각, 지도 계산 후 age, 추론 시간과 차단 이유를 확인한다.
4. 재부팅 후 frame/pose 일관성과 약 35mm 바닥 차이 확인을 위한 자료를 확보한다.
5. 이 검증 후에도 모터 시험은 별도 단계다. contact threshold 교정,
   error 100/자동 기립 원인, 제어권·정지·복구 및 모터 경로 guard 통합이 필요하다.
