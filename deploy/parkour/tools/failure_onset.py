"""성공/실패 실행을 나란히 놓고 **실패가 언제 어디서 시작되는가** 를 찾는다.

같은 설정에서 절반쯤 성공한다. 편차가 아니라 이산적 원인이라면 실패 시점 직전에
공통된 신호가 있어야 한다. 각 실행에 대해:
  - 실패 개시 시각: 전진속도가 0.2 m/s 아래로 떨어져 회복하지 못하는 첫 시점
  - 그때의 x, 지형 높이, 몸 높이 여유, roll/pitch, 발 파묻힘
을 뽑아 성공 실행과 비교한다.
"""
import glob
import sys

import numpy as np

PARKOUR = "/home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour"
sys.path.insert(0, PARKOUR)
from em_sidecar.kinematics import Go2Kinematics, quat_to_mat  # noqa: E402
from em_sidecar.tests.test_live_terrain import META, Terrain  # noqa: E402

import yaml  # noqa: E402

c = yaml.safe_load(open(f"{PARKOUR}/contract/deploy.yaml"))
il_to_sdk = np.asarray(c["index_maps"]["il_to_sdk"], dtype=int)
IMU_OFF = np.array([-0.02557, 0.0, 0.04232])
FOOT_R = 0.022
kin = Go2Kinematics(f"{PARKOUR}/contract/em_geometry.npz")
FK = [str(s) for s in np.load(f"{PARKOUR}/contract/em_geometry.npz")["fk_names"]]
IDX = [FK.index(f) for f in ("FL_foot", "FR_foot", "RL_foot", "RR_foot")]
terr = Terrain(META)


def yaw_arr(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def analyze(path):
    r = np.load(path)
    n = min(len(r["t"]), len(r["quat"]), len(r["q_sdk"]))
    tq = r["t"][:n] - r["sp_t"][0]
    quat = r["quat"][:n]
    q = r["q_sdk"][:n][:, il_to_sdk]
    sp_t = r["sp_t"] - r["sp_t"][0]
    pos = np.stack([np.interp(tq, sp_t, r["sp_pos"][:, k]) for k in range(3)], axis=1)
    for k in range(n):
        pos[k] -= quat_to_mat(quat[k]) @ IMU_OFF
    # 0.5초 창 전진속도
    w = int(0.5 / np.median(np.diff(tq)))
    vx = np.full(n, np.nan)
    vx[w:] = (pos[w:, 0] - pos[:-w, 0]) / (tq[w:] - tq[:-w])
    h = terr.height(pos[:, 0], pos[:, 1])
    clear = pos[:, 2] - h
    yw = yaw_arr(quat)

    # 실패 개시: vx < 0.2 이 1초 이상 이어지고 이후 회복 못 함
    onset = None
    hold = int(1.0 / np.median(np.diff(tq)))
    for i in range(w, n - hold):
        if np.all(vx[i:i + hold] < 0.2) and np.nanmax(vx[i:]) < 0.4:
            onset = i
            break
    ok = onset is None
    tag = "성공" if ok else "실패"
    top = (h > 0.30)
    line = (f"{path.split('/')[-1]:14} {tag}  "
            f"x {pos[-1,0]-pos[0,0]:+6.2f}  |y| {np.abs(pos[:,1]-pos[0,1]).max():5.2f}  "
            f"최고지형 {np.nanmax(h)*100:5.1f}cm  고원체류 {top.mean()*100:3.0f}%")
    if not ok:
        i = onset
        # 발 파묻힘 (개시 직전 0.5초)
        j0 = max(0, i - w)
        buried = 0
        for k in range(j0, i, max(1, (i - j0) // 20)):
            pb, _ = kin.link_poses_base(q[k])
            fw = pos[k] + pb[IDX] @ quat_to_mat(quat[k]).T
            g = fw[:, 2] - FOOT_R - terr.height(fw[:, 0], fw[:, 1])
            buried = max(buried, int((g < -0.015).sum()))
        line += (f"\n{'':14} → 개시 t={tq[i]:5.2f}s  x={pos[i,0]:+.2f}  "
                 f"지형 {h[i]*100:5.1f}cm  여유 {clear[i]:.3f}  "
                 f"yaw {np.degrees(yw[i]):+6.1f}°  파묻힌발 {buried}")
    print(line)
    return ok, (np.nanmax(h) if np.isfinite(np.nanmax(h)) else 0)


tag = sys.argv[1] if len(sys.argv) > 1 else "w"
files = sorted(glob.glob(f"/home/seo-jinwoo/.claude/jobs/c76485cb/tmp/traj_{tag}*.npz"))
print(f"{'파일':14} {'판정':4}  {'전진':>8}  {'|y|':>6}  {'최고지형':>9}  {'고원체류':>8}")
res = [analyze(f) for f in files]
n_ok = sum(1 for o, _ in res if o)
print(f"\n성공 {n_ok}/{len(res)}")
climbed = sum(1 for _, hm in res if hm > 0.35)
print(f"램프 정상(35cm 이상) 도달 {climbed}/{len(res)}")
