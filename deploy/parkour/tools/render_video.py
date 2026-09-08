"""MuJoCo 주행 기록 → [로봇 3인칭 | GT elevation map | estimated elevation map] 영상.

IsaacLab 의 `play.py --multicam --with_scandots` 를 MuJoCo 판으로 옮긴 것이다. 라이브
화면을 캡처하지 않고 **기록으로 다시 그린다** — 사이드카 기록(`--record --record-map`,
shadow 모드)에 sim 시각 기준으로 관절각·자세(lowstate_raw, 500 Hz)와 EM 전체 지도
(tick 별 34×34)가 함께 있으므로 프레임 시각을 정확히 맞출 수 있다.

    python tools/render_video.py em_ticks_vr_1.npz --scene ramp --out videos/mujoco_ramp.mp4
    python tools/render_video.py em_ticks_vs_2.npz --scene stairs --out videos/mujoco_stairs.mp4

패널
----
  왼쪽  : MuJoCo 오프스크린 렌더 (추적 카메라). 기록의 sport 위치(imu site → base 원점),
          IMU 쿼터니언, 관절각으로 qpos 를 직접 놓는다 (물리 재실행 아님).
  가운데: GT elevation map — EM 지도와 **같은 격자·같은 중심**에서 terrain_meta 를 샘플.
  오른쪽: estimated elevation map — 사이드카 EM 지도 (직접 관측 셀만, 상한 셀은 흐리게).
          둘 다 로봇 base z 기준 상대 높이, 같은 색범위, 정책이 실제로 먹는 scandots 132
          점을 점으로 겹친다. 상단에 관측 셀 평균 |est − GT| 를 적는다.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PARKOUR = HERE.parent
sys.path.insert(0, str(PARKOUR))
os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402

from em_sidecar.kinematics import Go2Kinematics, quat_to_mat, yaw_from_quat  # noqa: E402
from em_sidecar.sidecar import IMU_SITE_IN_BASE  # noqa: E402
from em_sidecar.tests.test_live_terrain import META, Terrain  # noqa: E402

MJ_ROBOTS = Path.home() / "workspace/codes/unitree_mujoco/unitree_robots/go2"
SCENES = {
    # 씬 파일, terrain_meta 기준 스폰(월드 원점이 되는 meta 좌표), 월드 z 보정
    "ramp": ("scene_parkour.xml", None, 0.0),
    "stairs": ("scene_parkour_stairs.xml", (-11.0, 2.0), -0.012),
}
# IsaacLab 관절 순서(hip×4, thigh×4, calf×4; 다리 FL FR RL RR) → MJCF qpos 순서(다리별 hip,thigh,calf)
IL_TO_MJ = [4 * j + leg for leg in range(4) for j in range(3)]


def load_record(path: Path):
    r = np.load(path)
    need = ["em_elev", "em_center", "lowstate_raw"]
    for k in need:
        if k not in r.files:
            raise SystemExit(f"{path}: {k} 없음 — shadow 모드 + --record-map 으로 기록해야 한다")
    return r


def raw_streams(r):
    raw = r["lowstate_raw"]
    t = raw[:, 0]
    keep = np.r_[True, np.diff(t) > 0]
    raw = raw[keep]
    t = raw[:, 0]
    q_il = raw[:, 1:13]
    quat = raw[:, 13:17]
    ps = raw[:, 24:27]
    st = raw[:, 27]
    # sport 위치를 stamp 로 tick 에 정렬 (수신 지터 제거)
    ok = np.isfinite(st) & np.isfinite(ps).all(1)
    us, ui = np.unique(st[ok], return_index=True)
    psu = ps[ok][ui]
    pos_imu = np.stack([np.interp(t, us, psu[:, k]) for k in range(3)], 1)
    R = quat_to_mat(quat)
    base = pos_imu - np.einsum("nij,j->ni", R, IMU_SITE_IN_BASE)
    return t, q_il, quat, base


def make_camera(model):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # 로봇 뒤-왼쪽 위에서 진행 방향(+x)을 내려다본다 — 앞의 램프/계단이 화면에 들어온다
    cam.distance = 3.0
    cam.azimuth = 205.0
    cam.elevation = -28.0
    return cam


def height_panel(h_rel, valid, dim, yaw, scan_xy, size, res, vmin, vmax, title, sub, robot_off=(0.0, 0.0)):
    """(n,n) [ix, iy] 상대 높이 → 로봇 중심 top-down 패널 (위가 +x, 왼쪽이 +y)."""
    n = h_rel.shape[0]
    img = np.full((n, n, 3), 40, np.uint8)
    norm = np.clip((h_rel - vmin) / (vmax - vmin), 0, 1)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    img[valid] = color[valid]
    if dim is not None:
        img[dim & ~valid] = (color[dim & ~valid] * 0.45).astype(np.uint8)
    # 배열 [ix, iy] → 화면 [row, col]: row = n−1−ix (위가 +x), col = n−1−iy (왼쪽이 +y)
    img = img[::-1, ::-1]
    img = cv2.resize(img, (size, size), interpolation=cv2.INTER_NEAREST)
    s = size / n  # 픽셀/셀
    cx = cy = size / 2.0

    def to_px(dx, dy):  # 지도 중심 기준 월드 오프셋 (dx, dy) → 픽셀
        return int(cx - dy / res * s), int(cy - dx / res * s)

    # scandots (yaw 정렬 격자) 와 로봇·heading. 로봇은 지도 중심에서 (rx, ry) 만큼 떨어져 있다
    rx, ry = robot_off
    cy_, sy_ = np.cos(yaw), np.sin(yaw)
    for ox, oy in scan_xy:
        px, py = to_px(rx + cy_ * ox - sy_ * oy, ry + sy_ * ox + cy_ * oy)
        cv2.circle(img, (px, py), 2, (255, 255, 255), -1)
    bx, by = to_px(rx, ry)
    hx, hy = to_px(rx + 0.35 * cy_, ry + 0.35 * sy_)
    cv2.arrowedLine(img, (bx, by), (hx, hy), (0, 0, 255), 2, tipLength=0.3)
    cv2.circle(img, (bx, by), 5, (0, 0, 255), -1)
    cv2.putText(img, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, sub, (8, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def colorbar(size, vmin, vmax):
    bar = np.linspace(1, 0, size).reshape(-1, 1)
    img = cv2.applyColorMap((bar * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    img = np.repeat(img, 22, axis=1)
    cv2.putText(img, f"{vmax:+.1f}", (0, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    cv2.putText(img, f"{vmin:+.1f}", (0, size - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return img


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--scene", choices=list(SCENES), default="ramp")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--start", type=float, default=None, help="영상 시작 sim 시각 [s] (기본: 로봇이 움직이기 1 s 전)")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=520)
    ap.add_argument("--vmin", type=float, default=-0.5, help="패널 색범위 [m], base z 기준")
    ap.add_argument("--vmax", type=float, default=0.3)
    a = ap.parse_args()

    scene_file, spawn, zfix = SCENES[a.scene]
    r = load_record(Path(a.record))
    t_raw, q_il, quat, base = raw_streams(r)
    t_em = r["stamp"].astype(np.float64)
    n_em = min(len(t_em), len(r["em_elev"]))
    em_elev = r["em_elev"][:n_em].astype(np.float32)
    em_valid = r["em_valid"][:n_em]
    em_isub = r["em_is_ub"][:n_em]
    em_ub = r["em_ub"][:n_em].astype(np.float32)
    em_center = r["em_center"][:n_em].astype(np.float64)
    res = float(r["em_res"])
    ncell = em_elev.shape[1]

    terr = Terrain(META)
    if spawn is not None:
        terr.spawn[:2] = spawn
    kin = Go2Kinematics(PARKOUR / "contract" / "em_geometry.npz")
    scan_xy = kin.scan_offsets_xy

    model = mujoco.MjModel.from_xml_path(str(MJ_ROBOTS / scene_file))
    # 오프스크린 버퍼 기본값(640×480)보다 큰 프레임을 그리려면 먼저 키워야 한다
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, a.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, a.height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=a.height, width=a.width)
    cam = make_camera(model)
    opt = mujoco.MjvOption()

    # 영상 구간: 서서(base z > 0.28) 0.2 m/s 이상으로 걷기 시작한 시각 −1 s 부터
    speed = np.linalg.norm(np.gradient(base[:, :2], t_raw, axis=0), axis=1)
    walk = np.where((base[:, 2] > 0.28) & (speed > 0.2))[0]
    t0 = a.start if a.start is not None else (t_raw[walk[0]] - 1.0 if len(walk) else t_raw[0])
    t1 = t_raw[-1] if a.duration is None else min(t_raw[-1], t0 + a.duration)
    frames_t = np.arange(t0, t1, 1.0 / a.fps)
    print(f"구간 {t0:.2f}~{t1:.2f} s, {len(frames_t)} 프레임 @ {a.fps} fps")

    size = a.height
    out_tmp = Path(a.out).with_suffix(".raw.mp4")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    W = a.width + 2 * size + 22
    vw = cv2.VideoWriter(str(out_tmp), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, size))
    errs = []
    for k, tf in enumerate(frames_t):
        i = int(np.searchsorted(t_raw, tf))
        i = min(max(i, 0), len(t_raw) - 1)
        j = int(np.searchsorted(t_em, tf)) - 1
        j = min(max(j, 0), n_em - 1)
        # --- 로봇 ---
        data.qpos[:3] = base[i]
        data.qpos[3:7] = quat[i]
        data.qpos[7:] = q_il[i][IL_TO_MJ]
        mujoco.mj_forward(model, data)
        cam.lookat[:] = base[i] + np.array([0.0, 0.0, 0.05])
        renderer.update_scene(data, cam, opt)
        rgb = renderer.render()
        left = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.putText(left, f"{a.scene}  t={tf - t0:5.2f}s  x={base[i, 0]:+.2f} z={base[i, 2]:.2f}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # --- 지도 (EM 격자 기준) ---
        c = em_center[j]
        ix = np.arange(ncell)
        cell_x = c[0] + (ix - 0.5 * ncell + 0.5) * res
        cell_y = c[1] + (ix - 0.5 * ncell + 0.5) * res
        gx, gy = np.meshgrid(cell_x, cell_y, indexing="ij")            # [ix, iy]
        gt_abs = terr.height(gx, gy) + zfix
        gt_valid = np.isfinite(gt_abs)
        est_abs = em_elev[j] + c[2]
        ub_abs = em_ub[j] + c[2]
        v = em_valid[j]
        est_show = np.where(v, est_abs, ub_abs)
        bz = base[i, 2]
        yaw = yaw_from_quat(quat[i])
        both = v & gt_valid
        err = float(np.abs(est_abs[both] - gt_abs[both]).mean()) if both.any() else float("nan")
        errs.append(err)
        # 지도 중심은 tick 시점의 base 위치라 프레임 시점의 로봇과 조금 어긋난다 — 그만큼 옮겨 그린다
        roff = (float(base[i, 0] - c[0]), float(base[i, 1] - c[1]))
        gt_p = height_panel(np.nan_to_num(gt_abs - bz), gt_valid, None, yaw, scan_xy, size, res,
                            a.vmin, a.vmax, "GT elevation map",
                            f"terrain_meta @ EM grid {ncell}x{ncell}, {res} m", roff)
        est_p = height_panel(est_show - bz, v, em_isub[j], yaw, scan_xy, size, res, a.vmin, a.vmax,
                             "estimated elevation map (EM sidecar)",
                             f"valid {int(v.sum())} cells   mean|est-GT| {err * 100:.1f} cm", roff)
        frame = np.concatenate([left, gt_p, est_p, colorbar(size, a.vmin, a.vmax)], axis=1)
        vw.write(frame)
        if k % 60 == 0:
            print(f"  프레임 {k}/{len(frames_t)}  err {err * 100:.1f} cm", flush=True)
    vw.release()
    renderer.close()  # 인터프리터 종료 시 EGL 컨텍스트 해제 오류를 피한다
    # h264 로 재인코딩 (플레이어 호환)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(out_tmp), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "20", str(a.out)], check=True)
    out_tmp.unlink()
    e = np.array(errs)
    print(f"저장: {a.out}  ({len(frames_t)} 프레임)  |est−GT| 평균 {np.nanmean(e) * 100:.1f} cm  "
          f"p95 {np.nanpercentile(e, 95) * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
