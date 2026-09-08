"""MuJoCo 주행 기록 → [로봇 3인칭 | GT scandots | estimated scandots] 영상.

IsaacLab 의 `play.py --multicam --with_scandots` 를 MuJoCo 판으로 옮긴 것이다. 라이브
화면을 캡처하지 않고 **기록으로 다시 그린다** — 사이드카 기록(`--record`, shadow 모드)에
sim 시각 기준으로 관절각·자세(lowstate_raw, 500 Hz)와 tick 별 scandots(정책 입력 132)
가 함께 있으므로 프레임 시각을 정확히 맞출 수 있다.

    python tools/render_video.py em_ticks_vr3_1.npz --scene ramp --out videos/mujoco_ramp.mp4
    python tools/render_video.py em_ticks_vs3_3.npz --scene stairs --out videos/mujoco_stairs.mp4

패널
----
  왼쪽  : MuJoCo 오프스크린 렌더 (추적 카메라). 기록의 sport 위치(imu site → base 원점),
          IMU 쿼터니언, 관절각으로 qpos 를 직접 놓는다 (물리 재실행 아님).
  가운데: GT scandots — 정책이 먹는 격자(x 12 × y 11, 0.15 m, yaw 정렬)의 **같은 132 위치**
          에서 terrain_meta 를 샘플해 학습과 같은 식 clip(base_z − h − 0.3, ±1) 으로 만든 값.
  오른쪽: estimated scandots — 사이드카가 실제로 발행한 obs[53:185] 그대로.
  두 패널 모두 값의 부호를 뒤집어 그린다 (−h_obs = 지형이 base 보다 높을수록 밝게).
  아래에 132 점 평균 |est − GT| 를 적는다. 정책 입력 밖의 지도는 그리지 않는다.
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
HEIGHT_OFFSET = 0.3  # 학습 scan 식의 오프셋


def load_record(path: Path):
    r = np.load(path)
    for k in ["stamp", "scan", "base_pos", "base_quat", "lowstate_raw"]:
        if k not in r.files:
            raise SystemExit(f"{path}: {k} 없음 — shadow 모드 + --record 로 기록해야 한다")
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


def make_camera():
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # 로봇 뒤-왼쪽 위에서 진행 방향(+x)을 내려다본다 — 앞의 램프/계단이 화면에 들어온다
    cam.distance = 3.0
    cam.azimuth = 205.0
    cam.elevation = -28.0
    return cam


class ScanGrid:
    """scan_offsets_xy (132,2) → (row, col) 격자 배치. 위가 +x(앞), 왼쪽이 +y."""

    def __init__(self, scan_xy: np.ndarray):
        self.xs = np.unique(np.round(scan_xy[:, 0], 4))   # 12
        self.ys = np.unique(np.round(scan_xy[:, 1], 4))   # 11
        self.nx, self.ny = len(self.xs), len(self.ys)
        ix = np.searchsorted(self.xs, np.round(scan_xy[:, 0], 4))
        iy = np.searchsorted(self.ys, np.round(scan_xy[:, 1], 4))
        self.row = self.nx - 1 - ix       # 큰 x 가 위
        self.col = self.ny - 1 - iy       # 큰 y 가 왼쪽
        # 로봇(0,0) 의 격자 위치 — 격자에 정확히 0 이 있으면 그 칸, 없으면 보간 위치
        self.r0 = self.nx - 1 - np.interp(0.0, self.xs, np.arange(self.nx))
        self.c0 = self.ny - 1 - np.interp(0.0, self.ys, np.arange(self.ny))

    def image(self, values: np.ndarray) -> np.ndarray:
        img = np.full((self.nx, self.ny), np.nan)
        img[self.row, self.col] = values
        return img


def scan_panel(grid: ScanGrid, values, cell, vmin, vmax, title, sub):
    """values(132) → 패널 이미지. 값은 −h_obs (지형이 base 보다 높을수록 큼)."""
    a = grid.image(values)
    norm = np.clip((np.nan_to_num(a, nan=vmin) - vmin) / (vmax - vmin), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    img[np.isnan(a)] = 40
    img = cv2.resize(img, (grid.ny * cell, grid.nx * cell), interpolation=cv2.INTER_NEAREST)
    # 셀 경계선
    for i in range(1, grid.nx):
        cv2.line(img, (0, i * cell), (img.shape[1], i * cell), (25, 25, 25), 1)
    for j in range(1, grid.ny):
        cv2.line(img, (j * cell, 0), (j * cell, img.shape[0]), (25, 25, 25), 1)
    # 로봇 위치와 heading (+x = 위)
    bx = int((grid.c0 + 0.5) * cell)
    by = int((grid.r0 + 0.5) * cell)
    cv2.arrowedLine(img, (bx, by), (bx, by - int(1.2 * cell)), (0, 0, 255), 2, tipLength=0.35)
    cv2.circle(img, (bx, by), 5, (0, 0, 255), -1)
    # 제목 띠
    top = np.full((30, img.shape[1], 3), 20, np.uint8)
    cv2.putText(top, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    bot = np.full((26, img.shape[1], 3), 20, np.uint8)
    cv2.putText(bot, sub, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return np.concatenate([top, img, bot], axis=0)


def colorbar(height, vmin, vmax):
    bar = np.linspace(1, 0, height).reshape(-1, 1)
    img = cv2.applyColorMap((bar * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    img = np.repeat(img, 26, axis=1)
    cv2.putText(img, f"{vmax:+.1f}", (0, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    cv2.putText(img, f"{vmin:+.1f}", (0, height - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return img


def fit_height(img, h):
    if img.shape[0] == h:
        return img
    pad = np.full((h - img.shape[0], img.shape[1], 3), 20, np.uint8) if img.shape[0] < h else None
    return np.concatenate([img, pad], axis=0) if pad is not None else img[:h]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--scene", choices=list(SCENES), default="ramp")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--start", type=float, default=None, help="영상 시작 sim 시각 [s] (기본: 걷기 시작 1 s 전)")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=520)
    ap.add_argument("--vmin", type=float, default=-0.4, help="패널 색범위 [m] (−h_obs: base−0.3 기준 지형 높이)")
    ap.add_argument("--vmax", type=float, default=0.8)
    a = ap.parse_args()

    scene_file, spawn, zfix = SCENES[a.scene]
    r = load_record(Path(a.record))
    t_raw, q_il, quat, base = raw_streams(r)
    t_em = r["stamp"].astype(np.float64)
    scan_est = r["scan"].astype(np.float64)              # (T,132) 정책 입력 그대로
    base_em = r["base_pos"].astype(np.float64)           # tick 시점 base 원점 (sport GT)
    quat_em = r["base_quat"].astype(np.float64)
    n_em = min(len(t_em), len(scan_est), len(base_em), len(quat_em))

    terr = Terrain(META)
    if spawn is not None:
        terr.spawn[:2] = spawn
    kin = Go2Kinematics(PARKOUR / "contract" / "em_geometry.npz")
    scan_xy = kin.scan_offsets_xy
    grid = ScanGrid(scan_xy)

    model = mujoco.MjModel.from_xml_path(str(MJ_ROBOTS / scene_file))
    # 오프스크린 버퍼 기본값(640×480)보다 큰 프레임을 그리려면 먼저 키워야 한다
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, a.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, a.height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=a.height, width=a.width)
    cam = make_camera()
    opt = mujoco.MjvOption()

    # 영상 구간: 서서(base z > 0.28) 0.2 m/s 이상으로 걷기 시작한 시각 −1 s 부터
    speed = np.linalg.norm(np.gradient(base[:, :2], t_raw, axis=0), axis=1)
    walk = np.where((base[:, 2] > 0.28) & (speed > 0.2))[0]
    t0 = a.start if a.start is not None else (t_raw[walk[0]] - 1.0 if len(walk) else t_raw[0])
    t1 = t_raw[-1] if a.duration is None else min(t_raw[-1], t0 + a.duration)
    frames_t = np.arange(t0, t1, 1.0 / a.fps)
    print(f"구간 {t0:.2f}~{t1:.2f} s, {len(frames_t)} 프레임 @ {a.fps} fps")

    cell = (a.height - 56) // grid.nx           # 패널 높이를 왼쪽 렌더에 맞춘다
    out_tmp = Path(a.out).with_suffix(".raw.mp4")
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    W = a.width + 2 * (grid.ny * cell) + 26 + 30
    vw = cv2.VideoWriter(str(out_tmp), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, a.height))
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
        left = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        cv2.putText(left, f"{a.scene}  t={tf - t0:5.2f}s  x={base[i, 0]:+.2f} z={base[i, 2]:.2f}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        # --- scandots (tick 시점 base·yaw 로 사이드카와 같은 위치) ---
        bz = base_em[j, 2]
        yaw = yaw_from_quat(quat_em[j])
        cy_, sy_ = np.cos(yaw), np.sin(yaw)
        px = base_em[j, 0] + cy_ * scan_xy[:, 0] - sy_ * scan_xy[:, 1]
        py = base_em[j, 1] + sy_ * scan_xy[:, 0] + cy_ * scan_xy[:, 1]
        h_gt = terr.height(px, py) + zfix
        gt_obs = np.clip(bz - h_gt - HEIGHT_OFFSET, -1.0, 1.0)      # 학습 scan 식
        est_obs = scan_est[j]
        okm = np.isfinite(gt_obs)
        err = float(np.abs(est_obs[okm] - gt_obs[okm]).mean()) if okm.any() else float("nan")
        errs.append(err)
        gt_p = scan_panel(grid, -gt_obs, cell, a.vmin, a.vmax, "GT scandots (12x11)",
                          "terrain_meta @ same 132 pts, 0.15 m")
        est_p = scan_panel(grid, -est_obs, cell, a.vmin, a.vmax, "estimated (EM sidecar)",
                           f"policy obs[53:185]  |est-GT| {err * 100:.1f} cm")
        gap = np.full((a.height, 10, 3), 20, np.uint8)
        frame = np.concatenate([left, gap, fit_height(gt_p, a.height), gap, fit_height(est_p, a.height),
                                gap, colorbar(a.height, a.vmin, a.vmax)], axis=1)
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
    print(f"저장: {a.out}  ({len(frames_t)} 프레임)  scandots |est−GT| 평균 {np.nanmean(e) * 100:.1f} cm  "
          f"p95 {np.nanpercentile(e, 95) * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
