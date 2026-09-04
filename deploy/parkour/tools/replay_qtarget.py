"""P1 게이트: 같은 관절 목표값을 개루프로 먹여 IsaacLab 과 관절 궤적을 비교한다.

왜 개루프인가
-------------
폐루프에서 로봇이 2~3초 만에 넘어지는데, 관측 53채널은 학습과 같은 규모임을 이미
확인했다(compare_live_obs.py). 그러면 남는 것은 "같은 명령을 받았을 때 몸이 같게
움직이는가" 다. 정책을 루프에서 빼면 그것만 따로 잰다.

무엇을 먹이나
-------------
골든 트레이스의 액션에서 학습과 같은 식으로 관절 목표값을 만든다:

    q_target[t] = clip(actions[t-1], ±4.8) * 0.25 + default_joint_pos

이것은 IsaacLab 이 그 스텝에 실제로 관절에 준 목표값이다(계약 §1.6, 골든 게이트
[3] 에서 오차 0 으로 검증됨). 같은 것을 MuJoCo 에 50 Hz 로 흘려보내고 관절이
어떻게 따라가는지 본다.

절차
----
  A. 안정화 : 트레이스 첫 프레임의 관절자세를 1.5초 유지 → 같은 출발 자세를 만든다
  B. 재생   : q_target[1..99] 를 50 Hz 로 흘리며 lowstate 를 기록
  C. 비교   : MuJoCo 관절 궤적 vs IsaacLab 기록,
              그리고 각 시뮬의 **추종 오차**(q_target − q) 를 나란히 놓는다

추종 오차를 비교하는 이유: 두 시뮬이 같은 목표를 받았으므로, 오차의 크기·위상이
다르면 그것이 곧 액추에이터/접촉 모델의 차이다.

주의: go2_ctrl 이 떠 있으면 lowcmd 를 두 프로세스가 다투므로 **끄고** 돌린다.

    python tools/replay_qtarget.py --out /tmp/p1.npz
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

TRACE = HERE.parent / "contract" / "golden_trace.npz"
DEPLOY = HERE.parent / "contract" / "deploy.yaml"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", default=str(TRACE))
    ap.add_argument("--deploy", default=str(DEPLOY))
    ap.add_argument("--out", required=True)
    ap.add_argument("--settle", type=float, default=1.5, help="출발 자세 유지 시간 [s]")
    ap.add_argument("--iface", default="lo")
    a = ap.parse_args()

    import yaml

    from em_sidecar.dds_compat import init_dds

    cfg = yaml.safe_load(open(a.deploy))
    il_to_sdk = np.asarray(cfg["index_maps"]["il_to_sdk"], dtype=int)
    default_q = np.asarray(cfg["default_joint_pos"]["isaaclab"], dtype=np.float64)
    scale = float(cfg["action"]["scale"])
    clip_lo, clip_hi = cfg["action"]["clip"]
    kp = np.asarray(cfg["actuator"]["stiffness"], dtype=np.float64)
    kd = np.asarray(cfg["actuator"]["damping"], dtype=np.float64)
    step_dt = float(cfg["timing"]["step_dt"])

    d = np.load(a.trace)
    actions = d["actions"][:, 0, :].astype(np.float64)      # (T,12) IsaacLab 순서
    q_ref = d["joint_pos"][:, 0, :].astype(np.float64)      # IsaacLab 이 실제로 간 관절각
    T = actions.shape[0]

    # 학습과 같은 식: t 스텝에 적용되는 것은 t-1 의 액션
    q_target = np.zeros((T, 12))
    for t in range(T):
        src = actions[t - 1] if t >= 1 else actions[0]
        q_target[t] = np.clip(src, clip_lo, clip_hi) * scale + default_q

    init_dds(0, a.iface)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
    from unitree_sdk2py.utils.crc import CRC

    lock = threading.Lock()
    rec: list = []
    pos = {"p": None}

    def on_sport(m):
        with lock:
            pos["p"] = np.array(list(m.position), dtype=np.float64)

    def on_low(m):
        with lock:
            rec.append((time.time(),
                        [m.motor_state[i].q for i in range(12)],
                        [m.motor_state[i].dq for i in range(12)],
                        None if pos["p"] is None else pos["p"].copy()))

    ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(on_sport, 10)
    ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0], cmd.head[1] = 0xFE, 0xEF
    cmd.level_flag = 0xFF
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0

    def send(q_il):
        for i in range(12):
            s = int(il_to_sdk[i])
            cmd.motor_cmd[s].q = float(q_il[i])
            cmd.motor_cmd[s].kp = float(kp[i])
            cmd.motor_cmd[s].kd = float(kd[i])
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    print(f"A. 출발 자세로 안정화 ({a.settle:.1f}s) — 트레이스 첫 프레임의 관절각")
    t0 = time.time()
    while time.time() - t0 < a.settle:
        send(q_ref[0])
        time.sleep(0.002)

    with lock:
        rec.clear()
    print(f"B. q_target 재생 {T} 스텝 @ {1/step_dt:.0f} Hz ({T*step_dt:.1f}s)")
    start = time.time()
    for t in range(T):
        send(q_target[t])
        nxt = start + (t + 1) * step_dt
        while time.time() < nxt:
            time.sleep(0.0005)
    # 마지막 명령 유지 (급격한 힘 빼기 방지)
    for _ in range(100):
        send(q_target[-1])
        time.sleep(0.002)

    with lock:
        R = list(rec)
    if not R:
        print("[RESULT] FAILED — lowstate 를 못 받았다")
        return 1

    tt = np.array([r[0] for r in R]) - start
    qq = np.array([r[1] for r in R])          # SDK 순서
    dqq = np.array([r[2] for r in R])
    pp = np.array([r[3] if r[3] is not None else [np.nan] * 3 for r in R])

    # SDK → IsaacLab 순서
    sdk_to_il = np.argsort(il_to_sdk)
    q_mj = qq[:, il_to_sdk]
    dq_mj = dqq[:, il_to_sdk]

    # 트레이스 시각(50Hz)에 맞춰 리샘플
    t_ref = np.arange(T) * step_dt
    q_mj_rs = np.stack([np.interp(t_ref, tt, q_mj[:, j]) for j in range(12)], axis=1)
    dq_mj_rs = np.stack([np.interp(t_ref, tt, dq_mj[:, j]) for j in range(12)], axis=1)

    np.savez(a.out, t=t_ref, q_target=q_target, q_mj=q_mj_rs, dq_mj=dq_mj_rs,
             q_isaac=q_ref, dq_isaac=d["joint_vel"][:, 0, :],
             pos_mj=np.stack([np.interp(t_ref, tt, pp[:, k]) for k in range(3)], axis=1),
             pos_isaac=d["root_pos_w"][:, 0])
    print(f"   기록 {len(R)} 샘플 → {a.out}")

    # ----------------------------------------------------------------- 비교
    n = min(T, 100)
    err_mj = q_mj_rs[:n] - q_target[:n]
    err_il = q_ref[:n] - q_target[:n]
    names = [f"{g}{leg}" for g in ("hip", "thigh", "calf")
             for leg in ("FL", "FR", "RL", "RR")]
    # IsaacLab 순서는 hip x4, thigh x4, calf x4
    names = ([f"{leg}_hip" for leg in ("FL", "FR", "RL", "RR")]
             + [f"{leg}_thigh" for leg in ("FL", "FR", "RL", "RR")]
             + [f"{leg}_calf" for leg in ("FL", "FR", "RL", "RR")])

    print(f"\n{'관절':10s} {'추종오차 RMS [rad]':>20s}   {'궤적 차이':>10s}")
    print(f"{'':10s} {'MuJoCo':>9s} {'IsaacLab':>10s}   {'RMS [rad]':>10s}")
    for j in range(12):
        print(f"  {names[j]:8s} {np.sqrt((err_mj[:, j]**2).mean()):9.4f} "
              f"{np.sqrt((err_il[:, j]**2).mean()):10.4f}   "
              f"{np.sqrt(((q_mj_rs[:n, j]-q_ref[:n, j])**2).mean()):10.4f}")

    print(f"\n  전체 추종오차 RMS : MuJoCo {np.sqrt((err_mj**2).mean()):.4f} rad, "
          f"IsaacLab {np.sqrt((err_il**2).mean()):.4f} rad")
    print(f"  전체 궤적 차이 RMS : {np.sqrt(((q_mj_rs[:n]-q_ref[:n])**2).mean()):.4f} rad")

    # 구간별 (초반은 접촉이 비슷하고 후반은 자세가 갈린다)
    print("\n  구간별 궤적 차이 RMS [rad]")
    for lo, hi in ((0, 25), (25, 50), (50, 75), (75, 100)):
        print(f"    {lo*step_dt:.1f}~{hi*step_dt:.1f}s : "
              f"{np.sqrt(((q_mj_rs[lo:hi]-q_ref[lo:hi])**2).mean()):.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
