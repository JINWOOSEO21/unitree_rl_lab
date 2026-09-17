# Jetson + Galaxy Book4 Pro 이식: manifest와 진행 기록

계획: `../migration_jetson_galaxybook_plan.md`. 이 문서는 실측한 값만 기록한다 (2026-09-17 데스크톱 →
노트북 → Jetson 세션 순서로 누적). 각 절의 제목에 어느 장비/세션의 결과인지 표시한다.
로봇 제어와 DDS 송신은 아직 어느 세션에서도 수행하지 않았다.

## 결정 사항 (2026-09-17 사용자 확인)

| 항목 | 결정 |
|---|---|
| 코드 전달 | 브랜치 `migration` commit + push, 각 장비에서 clone. main 직접 push 없음. (2026-09-17 `migration/jetson-galaxybook` 에서 이름 변경. 옛 이름은 원격에 없다) |
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

## Jetson bridge 기동 실패 — 원인과 수정 (2026-09-17, 단계 C-2)

증상: Jetson 에서 bridge 가 뜬 직후 `fatal: leg LowState gap too large` 로 죽고 scandots 가
한 번도 나오지 않았다. "Jetson 연산 능력이 부족하다" 로 보였으나 실측은 반대였다.

### 측정 (계획 C-5: "느리다고 timeout 부터 늘리지 말고 원인/최악지연을 측정한다")

`gap_probe.py` — lowstate 수신 + 10 Hz map 갱신, 구독 전용, 120 초:

| | map 없이 (90 s) | map 10 Hz (120 s) |
|---|---|---|
| lowstate 간격 p50 / p90 / p99 | 2 / 3 / 3 ms | **2 / 2 / 3 ms** |
| 최대 | 21 ms | **280 ms (1 회)** |
| 100 ms 초과 | 0 | **1** |

map 부하를 걸어도 p50/p90/p99 가 변하지 않는다. 280 ms 스파이크는 측정 루프 시작 **이전**
(at_s = −4.39 s, map 초기화 구간)에 1 회뿐이고 직후 간격은 `[2,2,2,2,2,2,2,1,3,2] ms` —
즉시 회복해 남은 115 초 동안 20 ms 초과가 0 회다. 정상 구간 성능은 충분하다.

`tools/go2_cloud_latency_probe.py` — 실제 lidar cloud + 실제 map 갱신, 60 초:

```
clouds=992 (16.5 Hz)  same_stamp=0  back_stamp=0   lowstate=32245 (537 Hz)
cloud 도착 간격      p50 64.9  p90 66.3  max 163.7 ms
tick 이 집었을 때 나이 p50 31.0  p90 57.2  max  64.5 ms   (200 ms 넘어야 cloud_too_old)
실제 map 갱신        p50 30.7  p90 42.4  max  49.9 ms
cloud 당 점 개수     p50 4172
```

### 원인

Jetson 성능이 아니라 **복구 경로 부재**였다. `go2_leg_pose.update()` 는 간격이 `max_dt_s`
를 넘으면 `last_tick` 을 갱신하지 않은 채 raise 한다. 그래서 이후 모든 표본이 고정된 옛
tick 과 비교되며 영구히 fatal 을 낸다. 같은 저장소의 `LegOdometry.step()` 은 같은 상황에서
그 구간만 건너뛰고 계속 가므로 정책이 두 곳에서 엇갈려 있었다. 데스크톱에서 안 드러난 것은
초기화 스파이크가 100 ms 미만이었기 때문이고, Jetson 은 GPU 가 느려 최초 CUDA/CuPy JIT 가
207 ms 걸려서 걸렸을 뿐이다 (같은 구간 map p50 은 31 ms).

### 수정 두 가지

1. `BaseMapper.warmup()` — 구독자를 만들기 전에 합성 클라우드로 update 를 한 번 돌려 커널을
   컴파일한다. 굶길 리더 스레드가 아직 없는 시점이다. 결과는 버려 실지도는 빈 상태로 시작한다.
   warmup 은 최적화일 뿐 전제조건이 아니므로 실패해도 기동을 막지 않는다. Jetson 실측 **49~77 ms**.
2. 구멍이 나도 복구한다. 간격을 보고만 하고 계속 간다. 구멍 동안 로봇은 움직였는데 odometry 는
   그만큼을 적분하지 못하므로, 그 전에 쌓은 지도는 못 잰 이동거리만큼 어긋나 있다 — 브리지가
   `discard_map()` 으로 버리고 다음 클라우드부터 다시 만든다. 위치 자체는 유지한다 (튀면
   지도의 1 m 불연속 검사에 걸린다). 구멍 직후 발을 곧바로 정지 발로 믿으면 안 되므로
   (`resume_after_gap()`) 20 ms 재안착을 강제한다. 30 초에 4 회를 넘으면 예전처럼 fatal.

**timeout(`max_dt_s`)은 올리지 않았다.** 올리면 잃어버린 표본 구간을 그대로 적분한다.

### 수정 후 Jetson 실측 (60 초, `--publish-scandots --summary-only`)

```
low=24850  scan=0  fault=132  fatal=0        map warmup 77 ms
```

**fatal 0 으로 60 초 완주** (이전에는 즉시 사망). `lowstate gap` fault 는 **0 회** — warmup 이
스파이크 자체를 없앴다. gyro 보정도 완료(`gyro_calibrated: true`).

## scandots 가 안 나오는 원인 — 단계별 특정 (2026-09-17, 단계 C-2)

위 수정으로 브리지는 죽지 않게 됐지만 `scan` 은 여전히 0 이고 fault 는 전부
`cloud_too_old` / `cloud_too_old_after_mapping` 이었다. 브리지를 단계로 쪼개 특정했다.

`tools/go2_bridge_stage_probe.py` — 브리지를 **수정하지 않고** 모듈 최상위 함수만 감싸
각 단계 통과 횟수를 세고, 모든 스레드 스택을 주기적으로 덤프한다. 각 함수는 도달 경로가
하나뿐이라 그대로 그 단계의 카운터가 된다 (`low_row`=LowState 콜백, `stamp_id`=cloud 콜백,
`time.sleep`=tick 루프 1 회전, `preceding_pose`=cloud 채택, `BaseMapper.update`=GPU 도달).

### 브리지 안에서만 모든 것이 느려진다

| | probe (같은 I/O, `leg.update` 없음) | 실제 브리지 | 기대 |
|---|---|---|---|
| lowstate | 537 Hz | **380 Hz** | 500 |
| cloud 콜백 | 16.5 Hz | **3.9 Hz** | 16.5 |
| tick 루프 | 10.0 Hz | **6.5 Hz** | 10 |
| map 갱신 최대 | 49.9 ms | **499 ms** | — |
| cloud 나이 최대 | 64.5 ms | **200 ms 초과** | <200 |

`cloud_too_old` 는 원인이 아니라 **증상**이다. map 갱신이 GPU 가 느려서가 아니라 GIL 을
못 얻어 499 ms 가 걸리고, 끝났을 때 cloud 가 이미 200 ms 를 넘긴 것이다. 첫 실행에서
22 초 후 fault 도 scan 도 없이 조용해진 것도 같은 이유다 — cloud 콜백이 완전히 굶으면
tick 루프는 `item[1] == last_cloud` 로 **아무 것도 emit 하지 않고** `continue` 한다.

### 스택 덤프: 리더 스레드는 항상 같은 곳에 있다

20 초 간격 독립 스냅샷 2 회 모두 LowState 리더 스레드가 leg odometry 안에 있었다.

```
File "em_sidecar/kinematics.py", line 28 in quat_to_mat
File "em_sidecar/leg_odometry.py", line 235 in step
File "tools/go2_leg_pose.py", line 141 in update
File "tools/go2_sensor_bridge.py", line 203 in low_callback
File "unitree_sdk2py/core/channel.py", line 110 in __OnDataAvailable
```

### 원인: 500 Hz 콜백 안에서 전체 FK 를 파이썬으로 돈다

`tools/go2_lowstate_cost.py` — DDS 없이 표본 하나의 비용만 잰다 (n=2000):

| 단계 | Jetson Orin NX | 노트북 x86 |
|---|---|---|
| `validate_lowstate` | 0.121 ms (6.0%) | 0.052 ms (2.6%) |
| `quat_to_mat` | 0.034 ms (1.7%) | 0.015 ms (0.7%) |
| `kin.link_poses_base` (FK) | 0.518 ms (25.9%) | 0.173 ms (8.6%) |
| **`LegPose.update` 전체** | **1.515 ms (75.8%)** | 0.494 ms (24.7%) |

괄호는 500 Hz 예산(표본당 2.00 ms) 대비 코어 점유율이다. **Jetson 에서 LowState 콜백
하나가 예산의 76 % 를 파이썬으로, GIL 을 쥔 채 쓴다.** `low_row`·`emit`·`poses.append`
까지 더하면 예산을 넘겨 실제로 380 Hz 로 떨어진다. 그 뒤에서 cloud 리더와 tick 루프와
GPU 호출이 전부 굶는다. x86 에서는 같은 일이 25 % 라 여유가 있었고, 그래서 데스크톱에서는
한 번도 드러나지 않았다.

덤으로 `validate_lowstate` 는 표본마다 **두 번** 돈다 — `low_callback` 에서 한 번,
`LegPose.update` 안에서 또 한 번. 매번 `GuardConfig()` 를 새로 만든다.

## 수정: 추정기 주기를 DDS 주기에서 분리 (2026-09-17, 단계 C-2/C-3 완료)

`max_dt_s` 나 `cloud_too_old` 의 200 ms 를 늘리는 것은 답이 아니다 — 계획 C-5 가 금지하고,
느린 원인을 둔 채 지연만 받아들이는 것이다. 대신 비싼 쪽을 덜 돌린다.

`LegPose(rate_hz=...)` 가 직전 채택 표본과의 간격이 `1/rate_hz` 미만이면 운동학 전에
`None` 을 돌려준다. 브리지는 `--leg-odom-hz` (기본 **100**) 로 넘긴다. `rate_hz=0` 은 예전
그대로 모든 표본을 돈다 — 기존 호출자(`lio/host.py`, `check_go2_gyro_replay.py`)는 그대로다.

100 Hz 인 이유: 지도는 pose 를 10 Hz 로만 쓰고 `preceding_pose` 는 20 ms 까지 묵은 pose 를
받는다. 10 ms 간격이면 양쪽에 여유가 있다. **오프라인 검수 gate
(`em_sidecar/tests/test_leg_odometry.py`) 자체가 50 Hz(`POLICY_DT = 0.02`) 로 돈다** —
즉 배포 주기가 합격선을 세운 주기보다 촘촘하다. `force_lp_tau_s`(6 ms) 저역통과도
`alpha = min(1, dt/tau)` 라 50 Hz 에서 이미 꺼져 있으므로, 100 Hz 는 gate 가 검증한 상태와
같다. 500 Hz 만 예외적으로 켜져 있던 쪽이었다.

### 실 로봇 30 초 녹화 재생 비교 (`tools/go2_leg_rate_compare.py`, 기립)

| 주기 | steps | 로봇 1초당 비용 | 끝 xy | 전주기 대비 차이 | 접촉발 p50 | valid |
|---|---|---|---|---|---|---|
| 매 표본(500 Hz) | 14960 | **613.0 ms (61.3 %)** | 0.56 cm | — | 2 | 52.3 % |
| 200 Hz | 5023 | 205.5 ms (20.6 %) | 0.64 cm | 0.31 cm | 2 | 51.9 % |
| **100 Hz** | 2963 | **121.5 ms (12.2 %)** | 1.33 cm | **1.03 cm (0.03 cm/s)** | 2 | 51.9 % |
| 50 Hz | 1492 | 61.8 ms (6.2 %) | 1.28 cm | 0.85 cm | 2 | 51.7 % |

비용 **5.0 배 감소**. 30 초 동안 전주기 대비 위치 차이는 **1 cm** 로, 3.2 m 창 합격선
20 cm 의 1/20 이다. 접촉 판정도 전혀 나빠지지 않는다 (접촉발 p50 2 로 동일, valid
52.3 → 51.9 %) — 저역통과가 꺼져 발 힘 채터가 aliasing 될 것이라는 우려는 실측에서
나타나지 않았다.

### 수정 후 브리지 (Jetson, 45 초)

| | 수정 전 | **수정 후** | 기대 |
|---|---|---|---|
| lowstate | 380 Hz | **500 Hz** | 500 |
| cloud 콜백 | 3.9 Hz | **15.5 Hz** | 16.5 |
| tick 루프 | 6.5 Hz | **10.0 Hz** | 10 |
| map 갱신 최대 | 499 ms | **280 ms** | — |
| **scan** | **0** | **초당 10 회 고정** | 10 |

`fatal 0`. gyro 보정 창 10 초 동안만 `leg_odom_no_reliable_support` 가 나오고, 그 뒤로는
끝까지 초당 10 회로 일정하다.

### 단계 C-3: 노트북 수신 확인

Jetson 브리지 발행 ↔ 노트북 `go2_parkour_rx_probe` 수신, 유선 30 초:

```
scandots count=300 rate_hz=9.98 age_ms=85.7 max_gap_ms=150.5 bad=0
gyro_bias=[0.0018867051,-0.0017683068,0.0044762394] seq=217 tick=1018147
```

gyro_bias 값이 Jetson 로그의 `[0.0018867050055884464,-0.0017683068215069838,0.004476239554526312]`
와 일치한다. **단계 C-2, C-3 통과.**

### 로봇은 반드시 기립 상태여야 한다

엎드리면 `foot_force` 가 `[7,5,5,6]` 로 접촉 임계 20 N 아래라 gyro 보정(네 발 모두 임계
초과 + `|gyro| ≤ 0.1` 이 10 초 연속)이 영원히 리셋되고, `pose_valid` 가 계속 false 라
scandots 가 하나도 안 나온다. 기립 시에는 `[29,27,27,30]` 으로 보정 조건 충족률 100 % 다.
전송 문제로 오인하기 쉬우니 scandots 가 0 이면 먼저 발 하중부터 본다.

### 남은 것

- 기립 `foot_force` 가 27~30 N 으로 임계 20 N 에 가깝다. `LegOdomCfg` 주석의
  "실기 foot_force 는 원시 정수라 재보정 필요" 가 아직 미결이다.
- `--duration` 실행이 루프 종료 후 **약 55 초 더 매달려 있다가** `timeout` 에 죽는다.
  `finally` 의 구독 종료/출력 close 경로가 깨끗하게 끝나지 않는다.

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

### A-4 키보드 / X11 — 전 항목 통과

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

#### 포커스 이탈 실측 (PASS)

최초 probe 는 포커스를 잃었을 때도 `a=0 d=0 -> 0` 으로만 찍어서 키 해제와 구분되지 않았다
(probe 설계 실수). `Go2HeldHeadingInput::focused()` 진단용 접근자를 추가하고 probe 가
`=== FOCUS LOST / REGAINED ===` 를 명시적으로 출력하도록 고친 뒤 재실측:

```
a=1 d=0 -> +0.2618 rad (+15.0 deg)
aaaaaaaa…aaaa                        <- 키를 계속 누른 상태 (autorepeat 에코)
=== FOCUS LOST ===
a=0 d=0 -> +0.0000 rad (+0.0 deg)   [unfocused]
=== FOCUS REGAINED ===
```

**키가 물리적으로 눌린 상태에서** 조향이 0 으로 떨어진다. 포커스를 되찾아도 `a=1` 이 다시
찍히지 않는다 — `go2_held_heading_tests` 의 "Focus return alone cannot reactivate held keys"
와 실제 X 서버 동작이 일치한다.

#### autorepeat 중 `XQueryKeymap` 오탐 여부 (문제 없음)

홀드 도중 `+15 → 0 → +15` 로 한 번 튄 구간이 있어 `XQueryKeymap` 이 autorepeat 사이에
키를 "떼짐" 으로 읽는지 검토했다. `xset q`: `auto repeat delay 500, repeat rate 33`.
`a` 에코 64개 ≈ 1.9초이고 probe 는 100Hz 폴링이므로 그 사이 약 190회 읽었다. autorepeat 마다
토글된다면 초당 33회 깜빡였을 것이고, probe 는 값이 바뀔 때만 출력하므로 교대 출력이 수십 줄
나왔어야 한다. 실제로는 autorepeat 약 100회 동안 단 1회 튀었다 → `XQueryKeymap` 은 물리 키
상태를 반영하며 autorepeat 의 영향을 받지 않는다. 그 1회는 손가락이 잠깐 떨어진 것으로 본다.
조향이 떨리는 현상은 없다. (단계 E 의 첫 Policy 주행에서 한 번 더 눈으로 확인하면 좋다.)

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

위 "없음" 판정은 나중에 `jetson_setup.sh`의 버그로 인한 오탐으로 밝혀졌다 (`set -o pipefail` 아래의 `ldconfig -p | grep -q`는 grep이 먼저 끝나면 SIGPIPE로 실패 처리된다. 데스크톱에서 40/40 재현). 수정 후 Jetson에서 `libopenblas.so.0`, `libgfortran.so.5`, `libnuma.so.1`, `libcudnn.so.8` 모두 system으로 판정되고 `ldd`도 `/lib/aarch64-linux-gnu`로 해석된다. 번들 `.deb`는 쓰이지 않으며 `opt/syslibs/lib`는 비어 있다. venv 쪽(ensurepip 없음)은 실제였다.

### Jetson 설치 결과 (2026-09-17, 사용자 실행, `jetson_setup.sh all` → `smoke` → `report`)

```
PASS versions            python 3.8.10 numpy 1.24.4 scipy 1.10.1 torch 2.0.0+nv23.05 cupy 12.3.0
PASS torch-cuda          Orin cuda 11.4
PASS cupy-nvrtc-kernel   runtime 11040 nvrtc ok
PASS torch-cupy-zero-copy
PASS dds-import          libddsc: ~/walking/opt/cyclonedds/lib/libddsc.so.0.10.2
PASS elevation-mapper    scan(132,) valid 132/132, update+sample ms p50 13.4 p95 15.0 max 23.3 first 34
```

- 지도 update+sample p95 15.0ms / max 23.3ms는 bridge의 cloud/map 200ms deadline 대비 약 8배 여유다 (데스크톱 RTX 3060은 p50 2.1ms). 6000점 합성 cloud 기준이며 실제 LiDAR cloud 크기와 DDS 콜백 부하는 포함하지 않는다.
- `libddsc`가 `~/walking/opt`의 0.10.2에서 로드됨을 `/proc/self/maps`로 확인했다. `/usr/local`의 0.11이나 ROS 환경의 것이 끼어들지 않는다.
- **미해결**: 스모크 중 tegrastats에서 CPU 8코어가 모두 100%였다 (1차 조사 유휴 시 5% 안팎). 데스크톱에서는 같은 루프가 연속 1.03코어 / 10Hz 페이스 0.05코어로 재현되지 않는다. aarch64 torch/OpenBLAS 스레드의 busy-wait인지 다른 프로세스인지 구분하기 위해 `gpu_smoke.py`에 `cpu-cost` 단계(이 프로세스 vs 장비 전체 코어 사용량, 10Hz 페이스 포함)와 `top` 스냅샷을 추가했다. 결과를 보기 전에는 `OMP_NUM_THREADS` 등을 바꾸지 않는다.

### 유선 데스크톱 DDS 수신 기준선 (2026-09-17, `tools/dds_rx_probe.py`, enp42s0, 10초, 수신 전용)

| 토픽 | 수신율 | 간격 ms (p50 / p95 / max) | 비고 |
|---|---|---|---|
| `rt/lowstate` | 497.6 Hz | 1.99 / 2.5 / 16.6 | tick 역행 0, 반복 9, 최대 step 15. tick은 ms 단위라 호스트 간 공유 ID로 쓸 수 있다 |
| `rt/utlidar/cloud` | 15.4 Hz | 64.9 / 66.8 / 68.9 | |
| `rt/utlidar/robot_odom` | 149 Hz | 6.6 / 8.7 / 23.9 | |
| `rt/parkour/scandots`, `rt/parkour/gyro_bias` | 미수신 | | bridge를 돌리지 않은 상태라 정상 |

Jetson eth0에서 같은 probe(`jetson_setup.sh rxprobe`)를 돌려 이 표와 비교한다. 노트북은 controller가 C++라 `unitree_sdk2py` 환경이 없을 수 있다. 있으면 `python tools/dds_rx_probe.py --interface <nic>`를 쓰고, 없으면 기존 C++ 수신 도구(`go2_scandots_probe`, `go2_gyro_bias_dds_probe`)를 쓰거나 LowState용 C++ probe를 추가한다 (미정).

`smoke`의 `elevation-mapper` 줄에 나오는 update+sample p95/max가 bridge의 cloud/map 200ms deadline 대비 Jetson의 여유를 보여준다. bridge 실행 전에는 `source ~/walking/env.sh`를 적용한다.

## 알려진 주의점

1. **git-lfs**: `.gitattributes`의 LFS 대상은 MuJoCo `.obj` 17개뿐이고 배포에 불필요하다. git-lfs가 없는 장비에서 전역 LFS 필터가 켜져 있으면 checkout이 실패하므로 `GIT_LFS_SKIP_SMUDGE=1`로 clone한다 (`notebook_setup.sh clone`에 반영).
2. ~~**정적 라이브러리**~~: 해소됨. Ubuntu 22.04 패키지에 `libboost_program_options.a`와 `libyaml-cpp.a`가 모두 있어 링크 방식 변경이 필요 없었다 (2026-09-17 노트북 실측).
3. **Jetson 오프라인 가능성**: Jetson이 인터넷에 못 나가면 wheel/소스를 데스크톱에서 받아 옮겨야 한다. 조사 스크립트가 pypi 도달성을 기록한다.
4. **captures 의존 도구**: `audit_go2_targets.py`, `check_go2_repeatability.py`, `shadow_go2_policy.py` 등 오프라인 분석 도구는 데스크톱 캡처가 필요하다. 이식 대상 장비에서는 실행하지 않는다.
5. **CycloneDDS 버전 혼입 (양쪽 장비 공통)**: 노트북은 ROS 2 Humble이 `LD_LIBRARY_PATH`로 SDK의 `libddsc.so.0`을 가로챘고, `-Wl,--disable-new-dtags`로 고쳤다 (위 "A-2 발견"). Jetson도 같은 종류의 위험이 확인됐다 — 시스템에 `libddsc.so.0.11.0`(0.11 계열)이 있고 ROS launch 서비스들이 상시 구동되며 전역 `CYCLONEDDS_URI`가 설정돼 있는데, `unitree_sdk2_python`은 `cyclonedds==0.10.2`를 고정한다. 그래서 Jetson은 C 0.10.2를 `~/walking/opt`에 별도 prefix로 빌드하고 `env.sh`로만 연결한다. **양쪽 모두, 실행 직전에 실제로 로드되는 DDS 라이브러리를 확인할 것** — 노트북은 `ldd build/go2_ctrl`, Jetson은 bridge venv에서 `python -c "import cyclonedds; ..."` 후 `/proc/<pid>/maps`. 링크가 아니라 **런타임 해석**이 기준이다.
6. **wayland에서는 a/d 조향 불가**: `Go2HeldHeadingInput`은 `XGetInputFocus`/`XQueryKeymap`을 쓴다. wayland 세션에서는 focus가 `None`으로 잡혀 조향이 비활성화된다 (crash 없이 비활성화). 실제 주행 세션은 반드시 "Ubuntu on Xorg"로 로그인한다.

## 다음 단계

1. ~~노트북: clone + 세션 실행, 단계 A-2/A-3~~ 완료.
2. ~~노트북: "Ubuntu on Xorg" 재로그인 + a / d / a+d / 포커스 이탈 확인~~ **완료. 노트북 단계 A 전체 종료.** 세션은 Xorg 로 유지한다 (GDM 이 선택을 기억한다).
3. Jetson: 1차 조사 완료. `~/walking`에서 `bash jetson_survey2.sh`(인터넷/시계, CycloneDDS 버전, 기존 서비스의 LowCmd 사용 여부)를 실행해 결과를 전달하고, 위 "Jetson 설치 절차"를 진행한다.
4. 이후 단계 B(설치) → C(유선 분산, 수신 전용) → D(AP 무선, 수신 전용) → 측정 보고 → E(실제 제어).

### 노트북에서 최초 1회

```
sudo apt update && sudo apt install -y git
mkdir -p ~/workspace/codes && cd ~/workspace/codes
GIT_LFS_SKIP_SMUDGE=1 git clone --branch migration https://github.com/JINWOOSEO21/unitree_rl_lab.git
cd unitree_rl_lab && claude
```

브랜치가 main에 merge된 뒤에는 `--branch main`으로 바꾼다.

### 노트북 세션 요청문

> `deploy/parkour/migration_jetson_galaxybook_plan.md`와 `deploy/parkour/migration/README.md`를 읽고 Galaxy Book4 Pro controller 쪽 이식을 진행해줘. 이 장비가 노트북이다. 단계 A-2~A-4부터 시작해: `deploy/parkour/migration/notebook_setup.sh`의 deps → clone → sdk → build → test → manifest 순서로 실행하고, sudo가 필요한 apt 명령은 나에게 실행을 요청해. 데스크톱 build 디렉터리는 복사하지 말고 새로 빌드해. 세션이 Xorg인지(`echo $XDG_SESSION_TYPE`) 먼저 확인하고, 모터 출력 없이 `--keyboard-check`와 a/d hold/release/동시누름/포커스이탈을 검증해. go2_ctrl 본 실행과 LowCmd 송신은 내가 명시적으로 지시하기 전에는 하지 마. 결과와 manifest 출력을 README에 기록하고 같은 브랜치에 commit/push해줘.

