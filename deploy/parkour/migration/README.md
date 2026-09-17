# Jetson + Galaxy Book4 Pro 이식: manifest와 진행 기록

계획: `../migration_jetson_galaxybook_plan.md`. 이 문서는 2026-09-17 데스크톱 세션에서 실측한 값만 기록한다.
로봇 제어, DDS 송신, Jetson/노트북 설치는 이 세션에서 수행하지 않았다.

## 결정 사항 (2026-09-17 사용자 확인)

| 항목 | 결정 |
|---|---|
| 코드 전달 | 브랜치 `migration/jetson-galaxybook` commit + push, 각 장비에서 clone. main 직접 push 없음 |
| 노트북 작업 | 데스크톱에서 SSH 원격 진행. apt(sudo)와 실물 키보드 hold/release 테스트만 사용자가 직접 수행 |
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

## 알려진 주의점

1. **git-lfs**: `.gitattributes`의 LFS 대상은 MuJoCo `.obj` 17개뿐이고 배포에 불필요하다. git-lfs가 없는 장비에서 전역 LFS 필터가 켜져 있으면 checkout이 실패하므로 `GIT_LFS_SKIP_SMUDGE=1`로 clone한다 (`notebook_setup.sh clone`에 반영).
2. **정적 라이브러리**: go2 CMakeLists는 `libboost_program_options.a`, `libyaml-cpp.a`를 이름으로 링크한다. Ubuntu 22.04 패키지에 `.a`가 있는지는 노트북에서 `notebook_setup.sh deps`로 확인한다. 없으면 그때 링크 방식을 결정한다.
3. **Jetson 오프라인 가능성**: Jetson이 인터넷에 못 나가면 wheel/소스를 데스크톱에서 받아 옮겨야 한다. 조사 스크립트가 pypi 도달성을 기록한다.
4. **captures 의존 도구**: `audit_go2_targets.py`, `check_go2_repeatability.py`, `shadow_go2_policy.py` 등 오프라인 분석 도구는 데스크톱 캡처가 필요하다. 이식 대상 장비에서는 실행하지 않는다.

## 다음 단계

1. 노트북: openssh-server 설치, 현재 IP/사용자명 전달, 데스크톱에서 `ssh-copy-id`, `notebook_setup.sh deps`가 출력하는 apt 한 줄 실행.
2. Jetson: `bash jetson_survey.sh` 실행 후 결과 파일 전달.
3. 이후 단계 B(설치) → C(유선 분산, 수신 전용) → D(AP 무선, 수신 전용) → 측정 보고 → E(실제 제어).
