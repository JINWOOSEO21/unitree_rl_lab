"""3-5 게이트: MuJoCo 파쿠르 지형이 IsaacLab 지형과 같은 자리에 있는가.

test_live_flat 은 평지라서 **방향 오류를 못 잡는다** (격자를 90° 돌려도 답이 같다).
파쿠르 지형을 넣은 지금이 그것을 잡을 수 있는 유일한 시점이다. 오프셋이 틀리거나
격자가 전치돼 있으면 로봇은 조용히 다른 지형 위를 달리게 되고, 증상은 "정책이
이상하게 못 한다" 로만 나타난다.

두 가지를 본다.

[A] 점군 → 월드 : 바닥 높이가 지형 격자와 맞는가
    L1 점군을 사이드카와 같은 변환으로 월드에 올리고, 각 점의 (x, y) 에서
    terrain_meta.npz 를 쌍선형 보간한 값과 z 를 비교한다. 로봇 주변 수 m 를
    직접 재는 것이라 오프셋·전치·부호가 전부 드러난다.

[B] scandots : 정책이 보는 132 값이 지형 정답과 맞는가
    정답 = clip(base_z − terrain(px, py) − 0.3, ±1).
    직접 관측된 셀만 잰다 (상한 대체 셀은 학습에서도 다른 값이다 — test_live_flat 참조).

    ./unitree_mujoco &            # config: robot_scene: scene_parkour.xml
    python -m em_sidecar.run &
    python -m em_sidecar.tests.test_live_terrain
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent))

from em_sidecar.dds_compat import init_dds  # noqa: E402
from em_sidecar.kinematics import quat_to_mat, yaw_from_quat  # noqa: E402
from em_sidecar.sidecar import IMU_SITE_IN_BASE, MOUNT_POS, MOUNT_QUAT  # noqa: E402

META = HERE.parent.parent / "mujoco" / "assets" / "terrain" / "terrain_meta.npz"


class Terrain:
    """terrain_meta.npz 의 높이 격자를 MuJoCo 월드 좌표에서 쌍선형 보간한다.

    MuJoCo 씬은 IsaacLab 스폰이 원점에 오도록 지형을 옮겼으므로
    (terrain_to_mjhfield.py), 월드 (x, y) → 격자 좌표는 spawn 만큼 되더한다.
    """

    def __init__(self, path: Path):
        m = np.load(path)
        self.H = m["hfield"].astype(np.float64)      # (ny, nx)
        self.res = float(m["res"])
        self.x0, self.y0 = float(m["x0"]), float(m["y0"])
        self.spawn = m["robot_spawn_pos_w"].astype(np.float64)
        self.ny, self.nx = self.H.shape

    def height(self, wx, wy):
        """월드 (x, y) 배열 → 지형 z. 격자 밖은 NaN."""
        gx = (np.asarray(wx) + self.spawn[0] - self.x0) / self.res
        gy = (np.asarray(wy) + self.spawn[1] - self.y0) / self.res
        i0 = np.floor(gx).astype(int)
        j0 = np.floor(gy).astype(int)
        fx = gx - i0
        fy = gy - j0
        ok = (i0 >= 0) & (i0 < self.nx - 1) & (j0 >= 0) & (j0 < self.ny - 1)
        i0c = np.clip(i0, 0, self.nx - 2)
        j0c = np.clip(j0, 0, self.ny - 2)
        h = ((1 - fx) * (1 - fy) * self.H[j0c, i0c]
             + fx * (1 - fy) * self.H[j0c, i0c + 1]
             + (1 - fx) * fy * self.H[j0c + 1, i0c]
             + fx * fy * self.H[j0c + 1, i0c + 1])
        return np.where(ok, h, np.nan)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--topic", default="rt/parkour/scandots")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--tol-ground-cm", type=float, default=2.0)
    ap.add_argument("--tol-scan-cm", type=float, default=3.0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LowState_, SportModeState_

    ter = Terrain(META)
    print(f"지형 격자 {ter.ny} x {ter.nx} (res {ter.res} m), "
          f"IsaacLab 스폰 {ter.spawn.tolist()} → 월드 원점")

    init_dds(a.domain, a.iface)
    lock = threading.Lock()
    st: dict = {"pos": None, "quat": None, "clouds": [], "maps": [], "valid": None}

    def on_sport(m):
        with lock:
            st["pos"] = np.array(list(m.position), dtype=np.float64)

    def on_low(m):
        with lock:
            st["quat"] = np.array(list(m.imu_state.quaternion), dtype=np.float64)

    def on_cloud(m):
        n = int(m.width)
        if n <= 0:
            return
        pts = np.frombuffer(bytes(m.data), dtype=np.float32)[: n * 3].reshape(n, 3)
        with lock:
            if st["pos"] is not None and st["quat"] is not None:
                st["clouds"].append((pts.astype(np.float64), st["pos"].copy(),
                                     st["quat"].copy()))

    def on_valid(m):
        with lock:
            st["valid"] = np.asarray(m.data, dtype=np.float64)

    def on_map(m):
        with lock:
            if st["pos"] is None or st["quat"] is None:
                return
            st["maps"].append((np.asarray(m.data, dtype=np.float64),
                               st["pos"].copy(), st["quat"].copy()))

    subs = [ChannelSubscriber("rt/sportmodestate", SportModeState_),
            ChannelSubscriber("rt/lowstate", LowState_),
            ChannelSubscriber("rt/utlidar/cloud", PointCloud2_),
            ChannelSubscriber(a.topic, HeightMap_),
            ChannelSubscriber(a.topic + "_valid", HeightMap_)]
    for s, cb, q in zip(subs, (on_sport, on_low, on_cloud, on_map, on_valid),
                        (10, 10, 1, 10, 10)):
        s.Init(cb, q)

    print(f"{a.duration:.0f}초 수신 …", flush=True)
    time.sleep(a.duration)
    with lock:
        clouds, maps, valid = list(st["clouds"]), list(st["maps"]), st["valid"]

    fails: list[str] = []

    # ------------------------------------------------------------------ [A]
    print(f"\n[A] 점군 → 월드 vs 지형 격자  (프레임 {len(clouds)}개)")
    if not clouds:
        print("    FAILED — 점군을 못 받았다")
        return 1
    errs = []
    for pts, p_rep, quat in clouds[-5:]:
        R_base = quat_to_mat(quat)
        base = p_rep - R_base @ IMU_SITE_IN_BASE
        R_s = R_base @ quat_to_mat(MOUNT_QUAT)
        t_s = base + R_base @ MOUNT_POS
        pw = pts @ R_s.T + t_s
        h = ter.height(pw[:, 0], pw[:, 1])
        m = np.isfinite(h)
        # 로봇 자신을 맞힌 점은 지형보다 훨씬 위에 있다 — 상위 꼬리를 잘라낸다
        e = pw[m, 2] - h[m]
        e = e[e < 0.15]
        errs.append(e)
    e = np.concatenate(errs)
    print(f"    점 {e.size}개  평균 {e.mean()*100:+.2f} cm  중앙값 {np.median(e)*100:+.2f} cm  "
          f"표준편차 {e.std()*100:.2f} cm")
    if abs(np.median(e)) * 100 > a.tol_ground_cm:
        fails.append(f"바닥 높이 중앙값이 {np.median(e)*100:+.2f} cm — 지형 정렬 의심")

    # ------------------------------------------------------------------ [B]
    print(f"\n[B] scandots vs 지형 정답  (프레임 {len(maps)}개)")
    if not maps:
        print("    FAILED — scandots 를 못 받았다")
        return 1
    data, p_rep, quat = maps[-1]
    R_base = quat_to_mat(quat)
    base = p_rep - R_base @ IMU_SITE_IN_BASE
    off = np.load(HERE.parent.parent / "contract" / "em_geometry.npz")["scan_offsets_xy"]
    yaw = yaw_from_quat(quat)
    cy, sy = np.cos(yaw), np.sin(yaw)
    px = base[0] + cy * off[:, 0] - sy * off[:, 1]
    py = base[1] + sy * off[:, 0] + cy * off[:, 1]
    gt = ter.height(px, py)
    want = np.clip(base[2] - gt - 0.3, -1.0, 1.0)

    if valid is None:
        print("    valid_frac 토픽이 없다 — 직접관측 셀을 못 가린다")
        fails.append("valid_frac 없음")
    else:
        direct = (valid > 0.999) & np.isfinite(want)
        print(f"    base ({base[0]:+.2f}, {base[1]:+.2f}, {base[2]:.3f}), "
              f"지형 높이 범위 [{np.nanmin(gt):+.3f}, {np.nanmax(gt):+.3f}] m")
        print(f"    직접관측 셀 {int(direct.sum())}/132")
        if direct.sum() < 20:
            fails.append(f"직접관측 셀이 {int(direct.sum())}개뿐 — 판정 불가")
        else:
            err = np.abs(data[direct] - want[direct])
            print(f"    오차 평균 {err.mean()*100:.3f} cm  최대 {err.max()*100:.3f} cm")
            if err.mean() * 100 > a.tol_scan_cm:
                fails.append(f"scandots 오차 {err.mean()*100:.3f} cm")

    print()
    for f in fails:
        print(f"  FAIL: {f}")
    print(f"[RESULT] {'OK' if not fails else 'FAILED'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
