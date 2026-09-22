"""MuJoCo 폐루프 주행 한 번을 띄우고 사이드카 기록(npz)을 남긴다 — render_video.py 의 입력.

    python eval/mujoco_walk_record.py --odom mit --scene ramp   --out /tmp/em_mit_ramp.npz
    python eval/mujoco_walk_record.py --odom mit --scene stairs --out /tmp/em_mit_stairs.npz
    python eval/render_video.py /tmp/em_mit_ramp.npz --scene ramp --out videos/mujoco_ramp_mitodom.mp4

기동 순서 (X 디스플레이 필요, 시뮬레이터 키 입력은 pty 로 넣는다)
  1. unitree_mujoco          — pty 포그라운드 (KeyboardJoystick 이 터미널을 읽는다)
  2. python -m em_sidecar --record — scandots + (mit) gyro bias 하트비트
  3. 접힌(down) 자세 만들기  — rt/lowcmd 로 [0, 1.36, −2.65]×4 까지 접는다. 새 State_Go2Pose 는
     실기처럼 **접힌 자세에서 출발**한다고 보고 현재 관절각 → 기립 자세로 곧장 보간한다.
     시뮬레이터의 로봇은 home 자세에서 무제어로 떨어져 널브러져 있어, 그대로 세우면 뒤집힌다.
  3'. go2_ctrl --sim         — '1' 로 기립
  4. 첫 scandots 를 기다린다. mit 는 **기립 후** 네 발 하중 + 10 s 정지 보정이 끝나야 나온다.
     sport/leg 는 사이드카가 --sim-gyro-bias 로 bias 0 하트비트를 낸다 (시뮬레이터 전용).
  5. '2' 로 Policy 진입 → 'w' 로 속도 명령(--vx) → --duration 초 주행 → 사이드카에 SIGINT (기록 저장)
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PARKOUR = HERE.parent
CODES = Path.home() / "workspace/codes"
sys.path.insert(0, str(CODES / "unitree_mujoco/example"))
from test_keyboard_pty import drain, kill_tree, proc_state, spawn_sim_on_pty  # noqa: E402

SIM_DIR = CODES / "unitree_mujoco/simulate/build"
CTRL_DIR = PARKOUR.parent / "robots/go2/build"
SCENE_ARGS = {
    "ramp": "",
    "stairs": "-s scene_parkour_stairs.xml",
    "ramp3x": "-s scene_parkour_ramp3x.xml",
    "ramp15": "-s scene_parkour_ramp15.xml",
}
# 마지막 goal 에서 주행을 끝내는 장면 (IsaacLab 의 goal 도달 종료와 같은 뜻): 장면 → (meta, 지형 이름)
SCENE_GOAL = {
    "ramp3x": (
        PARKOUR / "terrain/assets/terrain/terrain_meta_ramp3x.npz",
        "parkour_trapezoid_ramp",
    ),
    "ramp15": (
        PARKOUR / "terrain/assets/terrain/terrain_meta_ramp15.npz",
        "parkour_trapezoid_ramp",
    ),
    "stairs": (
        PARKOUR / "terrain/assets/terrain/terrain_meta.npz",
        "parkour_trapezoid_stairs",
    ),
}


def final_goal_x(scene: str) -> float | None:
    """마지막 goal 의 x (MuJoCo 월드 = IsaacLab 스폰 기준). 종료 지점이 없는 장면은 None."""
    if scene not in SCENE_GOAL:
        return None
    import numpy as np

    path, name = SCENE_GOAL[scene]
    m = np.load(path, allow_pickle=True)
    col = int(np.where(m["terrain_names"][0, :, 0] == name)[0][0])
    return float(
        m["goals"][0, col, -1, 0]
        + m["terrain_origins"][0, col, 0]
        - m["robot_spawn_pos_w"][0]
    )


# rt/parkour/scandots 에 유효한(132) 메시지가 올 때까지 기다린다. 사이드카 stdout 은 신뢰하지 않는다.
WAIT = r"""
import sys, time
sys.path.insert(0, "@P@")
from em_sidecar.dds_compat import init_dds
init_dds(0, "lo")
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_
got = []
sub = ChannelSubscriber("rt/parkour/scandots", HeightMap_)
sub.Init(lambda m: got.append(1) if len(m.data) == 132 else None, 10)
t0 = time.time()
while len(got) < 5 and time.time() - t0 < @T@:
    time.sleep(0.2)
print("READY" if len(got) >= 5 else "TIMEOUT")
"""

# go2_ctrl 을 띄우기 전에 실기 시작 자세(접힌 down pose)를 만든다. SDK 모터 순서, 다리마다 같다.
FOLD = r"""
import sys, time
import numpy as np
sys.path.insert(0, "@P@")
from em_sidecar.dds_compat import init_dds
init_dds(0, "lo")
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
low = [None]
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(lambda m: low.__setitem__(0, m), 10)
pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
t0 = time.time()
while low[0] is None and time.time() - t0 < 20:
    time.sleep(0.01)
if low[0] is None:
    print("FOLD_NO_LOWSTATE"); raise SystemExit
q0 = np.array([low[0].motor_state[i].q for i in range(12)])
q1 = np.array([0.0, 1.36, -2.65] * 4)
cmd, crc = unitree_go_msg_dds__LowCmd_(), CRC()
cmd.head[0], cmd.head[1], cmd.level_flag, cmd.gpio = 0xFE, 0xEF, 0xFF, 0
t0 = time.time()
while time.time() - t0 < 4.0:                       # 2 s 접기 + 2 s 유지
    a = min(1.0, (time.time() - t0) / 2.0)
    for i in range(12):
        m = cmd.motor_cmd[i]
        m.mode, m.q, m.dq, m.tau = 0x01, float((1 - a) * q0[i] + a * q1[i]), 0.0, 0.0
        m.kp, m.kd = (60.0 if i % 3 == 0 else 80.0), 5.0
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    time.sleep(0.002)
q = np.array([low[0].motor_state[i].q for i in range(12)])
print(f"FOLD_ERR={np.abs(q - q1).max():.3f}")
"""

# 주행 중 GT 위치를 본다 (결과 요약용).
WATCH = r"""
import sys, time
import numpy as np
sys.path.insert(0, "@P@")
from em_sidecar.dds_compat import init_dds
init_dds(0, "lo")
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
rows = []
sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
sub.Init(lambda m: rows.append((time.time(), *m.position)), 10)
t_end = time.time() + @D@
while time.time() < t_end:
    time.sleep(0.02)
    if rows and rows[-1][1] >= @X@:          # base(imu site) 가 마지막 goal 의 x 를 넘었다 → 종료
        print(f"REACHED={rows[-1][0] - rows[0][0]:.2f}")
        break
p = np.array(rows)
if len(p) < 10:
    print("POS_NONE"); raise SystemExit
t = p[:, 0] - p[0, 0]
print(f"T={t[-1]:.2f}"); print(f"DX={p[-1,1]-p[0,1]:.3f}"); print(f"DY={p[-1,2]-p[0,2]:.3f}")
print(f"Z0={p[0,3]:.3f}"); print(f"Z1={p[-1,3]:.3f}"); print(f"ZMAX={p[:,3].max():.3f}")
for k in range(4):
    a, b = int(len(p)*k/4), int(len(p)*(k+1)/4)-1
    print(f"SEG{k}={(p[b,1]-p[a,1])/max(t[b]-t[a],1e-9):.3f}")
"""


def py_snippet(python: str, code: str, timeout: float) -> str:
    r = subprocess.run(
        [python, "-u", "-c", code], capture_output=True, text=True, timeout=timeout
    )
    return r.stdout + r.stderr


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", choices=["mit", "sport", "leg"], default="mit")
    ap.add_argument("--scene", choices=list(SCENE_ARGS), default="ramp")
    ap.add_argument("--out", required=True, help="사이드카 기록 npz")
    ap.add_argument("--duration", type=float, default=25.0, help="Policy 주행 시간 [s]")
    ap.add_argument(
        "--vx",
        type=float,
        default=0.675,
        help="전진 속도 명령 [m/s]. go2_ctrl 은 cmd_vx = min + ly·(max − min) (config 0.3~0.8) 이고 "
        "시뮬레이터 'w' 키 한 번이 ly += 0.25 라서 0.3/0.425/0.55/0.675/0.8 중 가까운 값이 된다. "
        "학습 기준 0.67. 키를 안 누르면 0.3 이다.",
    )
    ap.add_argument(
        "--mit-skip-bad-pose",
        action="store_true",
        help="사이드카에 그대로 넘긴다 (무효 pose 에서 scandots 무효화 대신 tick 건너뜀)",
    )
    ap.add_argument("--wait", type=int, default=180, help="첫 scandots 대기 한도 [s]")
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":1"))
    ap.add_argument(
        "--python", default=str(Path.home() / "miniconda3/envs/env_isaaclab/bin/python")
    )
    ap.add_argument(
        "--log-dir", default=None, help="사이드카/go2_ctrl/시뮬레이터 로그 폴더"
    )
    a = ap.parse_args()
    out = Path(a.out).resolve()
    log_dir = Path(a.log_dir) if a.log_dir else out.parent
    log_dir.mkdir(parents=True, exist_ok=True)

    os.environ["SIM_ARGS"] = SCENE_ARGS[a.scene]
    for attempt in range(3):  # 직전 실행 직후에는 시뮬레이터가 가끔 바로 죽는다
        master, sim, leader = spawn_sim_on_pty(True, SIM_DIR, a.display)
        time.sleep(4.0)
        buf: list[bytes] = []
        drain(master, buf)
        if proc_state(sim) != "gone":
            break
        print(
            f"시뮬 기동 실패 (시도 {attempt + 1}/3):",
            b"".join(buf).decode(errors="replace")[-200:],
        )
        kill_tree(leader)
        os.close(master)
        time.sleep(5.0)
    else:
        return 1

    side_log = open(log_dir / f"{out.stem}_sidecar.log", "w")
    ctrl_log = open(log_dir / f"{out.stem}_ctrl.log", "w")
    # env python 을 직접 쓴다 — conda run 은 SIGINT 를 가로채 save_record 전에 자식을 죽인다.
    side = subprocess.Popen(
        [
            a.python,
            "-u",
            "-m",
            "em_sidecar",
            "--odom",
            a.odom,
            "--record",
            str(out),
            "--record-map",
        ]
        + (["--mit-skip-bad-pose"] if a.mit_skip_bad_pose else [])
        # sport/leg 는 go2_ctrl 의 gyro bias 게이트를 채울 발행자가 없다 (시뮬레이터 전용).
        # sport 에 --leg-shadow 를 붙이지 않는다: 그림자 leg 계산이 점군 tick 을 굶겨
        # GT 모드만 불리해졌던 적이 있다 (2026-09-22).
        + ([] if a.odom == "mit" else ["--sim-gyro-bias"]),
        cwd=PARKOUR,
        stdout=side_log,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
    )
    fold = py_snippet(a.python, FOLD.replace("@P@", str(PARKOUR)), 60)
    print(
        "[run] 접힌 자세:",
        [ln for ln in fold.splitlines() if ln.startswith("FOLD")],
        flush=True,
    )
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = "/usr/local/lib:" + env.get("LD_LIBRARY_PATH", "")
    ctrl = subprocess.Popen(
        ["./go2_ctrl", "--sim", "--network", "lo"],
        cwd=CTRL_DIR,
        env=env,
        stdout=ctrl_log,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
    )

    def cleanup():
        if side.poll() is None:
            side.send_signal(signal.SIGINT)  # finally: save_record
            try:
                side.wait(timeout=30)
            except subprocess.TimeoutExpired:
                side.kill()
        if ctrl.poll() is None:
            ctrl.kill()  # 시뮬레이터라 통제된 하강을 기다리지 않는다
        kill_tree(sim)
        kill_tree(leader)
        drain(master, buf)
        os.close(master)
        (log_dir / f"{out.stem}_sim.log").write_bytes(b"".join(buf))
        side_log.close()
        ctrl_log.close()

    try:
        time.sleep(6.0)
        if ctrl.poll() is not None or side.poll() is not None:
            print("go2_ctrl 또는 사이드카가 즉시 종료 — 로그 확인:", log_dir)
            return 1
        os.write(master, b"1")  # FixStand
        print("[run] 기립 — MIT 보정(10 s 정지)과 첫 scandots 를 기다린다", flush=True)
        t0 = time.time()
        w = py_snippet(
            a.python,
            WAIT.replace("@P@", str(PARKOUR)).replace("@T@", str(a.wait)),
            a.wait + 60,
        )
        if "READY" not in w:
            print(
                f"[run] scandots 가 오지 않았다 ({time.time() - t0:.0f} s):\n{w[-600:]}"
            )
            return 1
        print(
            f"[run] 첫 scandots 까지 {time.time() - t0:.1f} s → Policy 진입", flush=True
        )
        # 기립 보간 2 s + 정착 게이트가 끝나기 전의 '2' 는 거부된다 (sport/leg 는 scandots 가 바로 온다).
        time.sleep(max(1.0, 5.0 - (time.time() - t0)))
        os.write(master, b"2")  # Policy (start)
        n_w = int(round(min(max((a.vx - 0.3) / 0.5, 0.0), 1.0) / 0.25))
        for _ in range(n_w):  # ly += 0.25 씩
            time.sleep(0.15)
            os.write(master, b"w")
        print(f"[run] 속도 명령 {0.3 + 0.125 * n_w:.3f} m/s ('w' x{n_w})", flush=True)
        time.sleep(1.0)
        stop_x = final_goal_x(a.scene)
        if stop_x is not None:
            print(
                f"[run] 종료 지점: x ≥ {stop_x:.2f} m (마지막 goal) 또는 {a.duration:.0f} s",
                flush=True,
            )
        res = py_snippet(
            a.python,
            WATCH.replace("@P@", str(PARKOUR))
            .replace("@D@", str(a.duration))
            .replace("@X@", "float('inf')" if stop_x is None else repr(stop_x)),
            a.duration + 120,
        )
        vals = dict(
            ln.split("=", 1)
            for ln in res.splitlines()
            if "=" in ln and ln.split("=")[0].isupper()
        )
        print(f"[run] GT 주행 결과: {vals}")
    finally:
        cleanup()

    ctrl_out = (log_dir / f"{out.stem}_ctrl.log").read_text(errors="replace")
    for key in ("Policy entry rejected", "Passive", "gyro bias latched"):
        hits = [ln for ln in ctrl_out.splitlines() if key in ln]
        if hits:
            print(f"[ctrl] '{key}' {len(hits)}건, 마지막: {hits[-1].strip()[-160:]}")
    print(f"[run] 기록: {out} ({'있음' if out.exists() else '없음'})  로그: {log_dir}")
    return 0 if out.exists() else 1


if __name__ == "__main__":
    sys.exit(main())
