"""라이브 게이트 — 돌고 있는 unitree_mujoco 를 상대로 배관 전체를 잰다.

오프라인 게이트(test_pipeline_offline)는 수식 지형에 합성 점군을 먹였다. 이건
실제 경로를 전부 지난다: MuJoCo 레이캐스트 → PointCloud2 → DDS → 사이드카 →
HeightMap_ → 여기.

두 가지를 본다. 나누는 이유가 있다.

[A] 절대 게이트 — 점군을 월드로 올려 바닥 높이를 잰다.
    scene.xml 의 바닥은 z = 0 인 plane 이다. 점군을 사이드카와 **같은 변환**으로
    월드에 올렸을 때 바닥 점들이 0 에 모이면, pose 해석(‑ imu site 오프셋 포함),
    마운트, 쿼터니언 규약이 전부 맞다는 뜻이다.

    이 검사가 왜 따로 필요한가: scandots 는 h = base_z − h_map − 0.3 이라
    base_z 에 상수 오차 δ 가 있으면 지도도 δ 만큼 밀려 **상쇄된다**. 즉 [B] 만으로는
    pose 오차를 못 잡는다. 바닥의 절대 높이를 재야 잡힌다.

[B] scandots 게이트 — 평지 셀의 h 가 base_z − 0.3 인가, 그리고 셀 간에 균일한가.

주의: 기본 씬은 평지가 아니다. x = 1.1~1.3 과 1.5~1.7 에 8 cm 상자가 있고
x >= 2.1 부터 계단이다. 상자 근처 셀은 [B] 에서 뺀다 (그 검증은 지형을 바꾸는
3-5 단계에서 한다).

    ./unitree_mujoco &                          # simulate/build
    python -m em_sidecar.run &
    python -m em_sidecar.tests.test_live_flat
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

# unitree_robots/go2/scene.xml 의 장애물 — (x중심, x반폭, 윗면 z).
# 상자는 y 로 ±2 m 라 사실상 벽이므로 x 만 본다.
SCENE_BOXES = [(1.2, 0.1, 0.08), (1.6, 0.1, 0.08),
               (2.3, 0.2, 0.17), (2.6, 0.22, 0.32), (2.8, 0.23, 0.47),
               (3.0, 0.24, 0.62), (3.2, 0.25, 0.77), (3.4, 0.26, 0.92)]


def scene_height(x: np.ndarray) -> np.ndarray:
    z = np.zeros_like(np.asarray(x, dtype=np.float64))
    for cx, hw, top in SCENE_BOXES:
        z = np.where(np.abs(np.asarray(x) - cx) <= hw, np.maximum(z, top), z)
    return z


def near_box(x: np.ndarray, margin: float = 0.25) -> np.ndarray:
    m = np.zeros_like(np.asarray(x, dtype=np.float64), dtype=bool)
    for cx, hw, _ in SCENE_BOXES:
        m |= np.abs(np.asarray(x) - cx) <= hw + margin
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--topic", default="rt/parkour/scandots")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--tol-ground-cm", type=float, default=1.0)
    ap.add_argument("--tol-scan-cm", type=float, default=2.0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LowState_, SportModeState_

    init_dds(a.domain, a.iface)

    lock = threading.Lock()
    st: dict = {"pos": None, "quat": None, "cloud": None, "maps": [],
                "valid": None, "n_sport": 0, "n_cloud": 0}

    def on_sport(m):
        with lock:
            st["pos"] = np.array(list(m.position), dtype=np.float64)
            st["n_sport"] += 1

    def on_low(m):
        with lock:
            st["quat"] = np.array(list(m.imu_state.quaternion), dtype=np.float64)

    def on_cloud(m):
        n = int(m.width)
        if n <= 0:
            return
        pts = np.frombuffer(bytes(m.data), dtype=np.float32)[: n * 3].reshape(n, 3)
        with lock:
            st["cloud"] = (pts.astype(np.float64), st["pos"], st["quat"])
            st["n_cloud"] += 1

    def on_valid(m):
        with lock:
            st["valid"] = np.asarray(m.data, dtype=np.float64)

    def on_map(m):
        with lock:
            if st["pos"] is None or st["quat"] is None:
                return
            st["maps"].append((np.asarray(m.data, dtype=np.float64),
                               st["pos"].copy(), st["quat"].copy(),
                               int(m.width), int(m.height), float(m.resolution)))

    subs = [
        ChannelSubscriber("rt/sportmodestate", SportModeState_),
        ChannelSubscriber("rt/lowstate", LowState_),
        ChannelSubscriber("rt/utlidar/cloud", PointCloud2_),
        ChannelSubscriber(a.topic, HeightMap_),
        ChannelSubscriber(a.topic + "_valid", HeightMap_),
    ]
    for s, cb, q in zip(subs, (on_sport, on_low, on_cloud, on_map, on_valid),
                        (10, 10, 1, 10, 10)):
        s.Init(cb, q)

    print(f"{a.duration:.0f}초 동안 수신 …", flush=True)
    time.sleep(a.duration)

    with lock:
        maps = list(st["maps"])
        valid = st["valid"]
        cloud = st["cloud"]
        n_sport, n_cloud = st["n_sport"], st["n_cloud"]

    print(f"sportmodestate {n_sport}건, 점군 {n_cloud}건, scandots {len(maps)}건\n")
    fails: list[str] = []

    # ---------------------------------------------------------------- [A]
    print("[A] 점군 → 월드 : 바닥이 z = 0 에 오는가")
    if cloud is None or cloud[1] is None or cloud[2] is None:
        print("    FAILED — 점군/상태를 못 받았다")
        return 1
    pts_s, p_rep, quat = cloud
    R_base = quat_to_mat(quat)
    base_pos = p_rep - R_base @ IMU_SITE_IN_BASE  # 사이드카와 같은 보정
    R_s = R_base @ quat_to_mat(MOUNT_QUAT)
    t_s = base_pos + R_base @ MOUNT_POS
    pw = pts_s @ R_s.T + t_s

    # 바닥 후보: 장애물 x 구간 밖의 점들.
    flat_zone = ~near_box(pw[:, 0], margin=0.0)
    zg = pw[flat_zone, 2]
    # 자기 몸이 섞여 있을 수 있으므로 최빈 구간(1 cm bin)을 바닥으로 본다.
    hist, edges = np.histogram(zg, bins=np.arange(-0.5, 1.0, 0.01))
    k = int(hist.argmax())
    band = (zg >= edges[k] - 0.02) & (zg <= edges[k + 1] + 0.02)
    z_ground = float(np.median(zg[band]))
    print(f"    점 {pts_s.shape[0]}개 (평지 구간 {int(flat_zone.sum())}개)")
    print(f"    보정 전 위치 z {p_rep[2]:+.4f} → base 원점 z {base_pos[2]:+.4f}")
    print(f"    바닥 z = {z_ground*100:+.3f} cm  (정답 0.000 cm, "
          f"모인 점 {int(band.sum())}개)")
    if abs(z_ground) * 100 > a.tol_ground_cm:
        fails.append(f"바닥 z 가 {z_ground*100:+.3f} cm — pose/마운트 해석 의심")

    # ---------------------------------------------------------------- [B]
    print("\n[B] scandots : 평지 셀이 base_z − 0.3 인가")
    if not maps:
        print("    FAILED — scandots 를 못 받았다 (사이드카가 떠 있는가?)")
        return 1
    data, p_rep, quat, w, h, res = maps[-1]
    if data.size != 132 or w * h != 132:
        print(f"    FAILED — 132 가 아니다 ({data.size}, {w}x{h})")
        return 1

    R_base = quat_to_mat(quat)
    base_pos = p_rep - R_base @ IMU_SITE_IN_BASE
    off = np.load(HERE.parent.parent / "contract" / "em_geometry.npz")["scan_offsets_xy"]
    yaw = yaw_from_quat(quat)
    cy, sy = np.cos(yaw), np.sin(yaw)
    px = base_pos[0] + cy * off[:, 0] - sy * off[:, 1]

    want_flat = float(np.clip(base_pos[2] - 0.0 - 0.3, -1.0, 1.0))
    nonzero = data != 0.0
    away = ~near_box(px)
    print(f"    base_z {base_pos[2]:.4f} → 평지 정답 h {want_flat:+.4f}")

    if valid is None:
        print("    (valid_frac 진단 토픽 없음 — 직접관측/상한대체를 못 가른다)")
        groups = [("전체", nonzero & away)]
    else:
        direct = valid > 0.999   # 4이웃이 전부 직접 관측된 셀
        part = (valid > 1e-6) & ~direct
        ub = (valid <= 1e-6) & nonzero
        groups = [("직접관측(4/4)", direct & away),
                  ("부분관측", part & away),
                  ("상한대체", ub & away)]

    for name, m in groups:
        if m.sum() == 0:
            print(f"    {name:14s}   0셀")
            continue
        err = np.abs(data[m] - want_flat)
        print(f"    {name:14s} {int(m.sum()):3d}셀  h평균 {data[m].mean():+.4f}  "
              f"오차 평균 {err.mean()*100:6.3f} cm  최대 {err.max()*100:6.3f} cm")

    # 게이트는 **직접 관측된 셀**에만 건다. 상한대체 셀은 학습 때도 있던
    # 정상 동작(cascade 2단계)이라 정답과 다른 것이 당연하다.
    key = groups[0][1]
    if key.sum() < 20:
        fails.append(f"직접관측 셀이 {int(key.sum())}개뿐 — 판정 불가")
    else:
        err = np.abs(data[key] - want_flat)
        if err.mean() * 100 > a.tol_scan_cm:
            fails.append(f"직접관측 셀 오차 {err.mean()*100:.3f} cm")

    if len(maps) >= 5:
        recent = np.stack([m[0] for m in maps[-5:]])
        print(f"    최근 5프레임 셀별 표준편차 최대 "
              f"{recent.std(axis=0)[nonzero].max()*100:.3f} cm")

    print()
    for f in fails:
        print(f"  FAIL: {f}")
    print(f"[RESULT] {'OK' if not fails else 'FAILED'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
