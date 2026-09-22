"""terrain_meta.npz 의 사다리꼴 램프를 **기울기는 그대로, 높이만 N 배**로 바꾼 sim2sim 시험용 지형.

    python terrain/make_tall_ramp.py                       # 3 배, 마지막 goal = 내리막 끝 + 0.4 m
    python terrain/terrain_to_mjhfield.py --meta terrain/assets/terrain/terrain_meta_ramp3x.npz \
        --out-dir ~/workspace/codes/unitree_mujoco/unitree_robots/go2 \
        --hfield-name parkour_terrain_ramp3x.hfield --scene-name scene_parkour_ramp3x.xml \
        --probe-name parkour_terrain_ramp3x_probe.npz

왜 IsaacLab 에서 다시 굽지 않나
------------------------------
원본은 IsaacLab 생성기(parkour_trapezoid_ramp_terrain) → export_terrain.py 로 구웠다. 그 생성기는
학습 지형 분포이기도 해서 (plateau_height_range 0.4~0.8 m, 마지막 goal = 내리막 끝 + 0.8~1.5 m)
시험용 지형 하나 때문에 바꾸면 학습이 달라진다. 여기서는 구워 둔 높이 격자를 직접 고친다:

    H_new = H_old − ideal_old + ideal_new

ideal_* 은 생성기와 **같은 식**의 사다리꼴이다 (픽셀당 상승 = rint(i · hs · tan θ / vs) · vs, 0.1 m 픽셀
사이는 삼각망처럼 선형). 잔차 H_old − ideal_old 는 roughness 노이즈라 그대로 얹힌다 — 발 디딤 조건이
원본과 같다. 램프가 길어져 새로 램프가 되는 자리의 잔차도 원래 그 자리(평지)의 노이즈다.

goal 도 생성기의 _lay_goals_over_trapezoid 와 같은 규칙으로 다시 깐다. 다른 것은 마지막 goal 하나 —
내리막 끝 + U(0.8, 1.5) m 가 아니라 **내리막 끝 + final_goal_offset** (기본 0.4 m: base 가 거기 오면
뒷발이 막 램프를 벗어난 때다). mujoco_walk_record.py 는 base 가 마지막 goal 의 x 를 넘으면 주행을 끝낸다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
META = HERE / "assets" / "terrain" / "terrain_meta.npz"


def trapezoid(
    px: np.ndarray,
    up_start: int,
    run_len: int,
    plateau_len: int,
    rise_per_px: float,
    vs: float,
) -> np.ndarray:
    """생성기와 같은 픽셀 높이(0.1 m 격자)를 만들고 px(실수 픽셀 좌표)에서 선형 보간한다 [m]."""
    n = int(np.ceil(px.max())) + 2
    h = np.zeros(n)
    ramp = np.rint(np.arange(1, run_len + 1) * rise_per_px)
    top = ramp[-1]
    up_end, down_start = up_start + run_len, up_start + run_len + plateau_len
    h[up_start:up_end] = ramp
    h[up_end:down_start] = top
    h[down_start : down_start + run_len] = top - ramp
    return np.interp(px, np.arange(n), h) * vs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", default=str(META))
    ap.add_argument("--out", default=None, help="기본: terrain_meta_ramp<scale>x.npz")
    ap.add_argument(
        "--scale", type=float, default=3.0, help="램프 높이 배율 (기울기 유지)"
    )
    ap.add_argument(
        "--slope-deg",
        type=float,
        default=None,
        help="새 램프 기울기 [도]. 기본은 생성기 값(10 + 27·difficulty = 28.9°)",
    )
    ap.add_argument(
        "--height",
        type=float,
        default=None,
        help="새 램프 높이 [m]. 주면 --scale 대신 이 높이(기울기에 맞춰 길이 결정)",
    )
    ap.add_argument(
        "--tag",
        default=None,
        help="출력 이름 terrain_meta_<tag>.npz (기본 ramp<scale>x)",
    )
    ap.add_argument(
        "--final-goal-offset",
        type=float,
        default=0.4,
        help="[m] 내리막 끝 → 마지막 goal",
    )
    a = ap.parse_args()

    m = dict(np.load(a.meta, allow_pickle=True))
    H = m["hfield"].astype(np.float64)
    res, x0, y0 = float(m["res"]), float(m["x0"]), float(m["y0"])
    hs, vs = float(m["hs"]), float(m["vs"])
    col = int(np.where(m["terrain_names"][0, :, 0] == "parkour_trapezoid_ramp")[0][0])
    origin = m["terrain_origins"][0, col]
    tile_len, tile_w = m["tile_size"]
    tile_x0 = origin[0] - tile_len / 2  # 타일 시작 x (월드)
    goals = m["goals"][0, col].copy()  # 타일 원점 기준 [m]

    # --- 원본 사다리꼴을 goal 에서 되읽는다 (생성기 규칙의 역산) -------------------------------
    gpx = np.rint((goals[:, 0] + tile_len / 2 - hs / 2) / hs).astype(int)  # goal → 픽셀
    up_start = gpx[0] + 1  # goals[0] = platform_len − 1
    down_end = gpx[-2]  # 마지막 중간 goal = course_end
    slope = np.tan(np.deg2rad(10 + 27 * float(m["difficulty_range"][0])))
    rise_per_px = hs * slope / vs
    xs = x0 + np.arange(H.shape[1]) * res  # 격자 x (월드)
    px0 = (xs - tile_x0 - hs / 2) / hs  # 격자 x → goal 과 같은 픽셀 좌표
    ys = y0 + np.arange(H.shape[0]) * res
    in_tile = np.abs(ys - origin[1]) < tile_w / 2
    # run_len: 잔차가 가장 작아지는 값을 고른다 (plateau 높이는 난수라 meta 에 없다)
    prof = np.median(H[in_tile][:, :], axis=0)
    # 높이 꼭짓점과 goal 은 반 픽셀쯤 어긋나 있다 (삼각망 꼭짓점 위치 규약) — 어긋남도 함께 맞춘다.
    best = None
    for shift in (-1.0, -0.5, 0.0, 0.5):
        for run in range(2, 40):
            plateau = down_end - up_start - 2 * run
            if plateau < 2:
                break
            ideal = trapezoid(
                (px0 + shift).clip(0), up_start, run, plateau, rise_per_px, vs
            )
            e = np.abs(prof - ideal)[(px0 > up_start - 5) & (px0 < down_end + 5)].mean()
            if best is None or e < best[0]:
                best = (e, run, plateau, shift)
    err, run_old, plateau_len, shift = best
    px = px0 + shift
    ideal_old = trapezoid(px.clip(0), up_start, run_old, plateau_len, rise_per_px, vs)
    h_old = np.rint(run_old * rise_per_px) * vs
    # 코스 폭: 꼭대기 구간에서 ideal 의 절반보다 높은 y
    top_cols = (px > up_start + run_old + 1) & (
        px < up_start + run_old + plateau_len - 1
    )
    # 가장자리 줄(옆면이 반쯤 깎인 곳)은 잔차가 사다리꼴과 무관하므로 제외한다 (0.9·h 이상인 줄만).
    on = in_tile & (np.median(H[:, top_cols], axis=1) > 0.9 * h_old)
    print(
        f"원본 램프: up_start px {up_start}, run {run_old} px ({run_old * hs:.1f} m), 꼭대기 {plateau_len * hs:.1f} m, "
        f"높이 {h_old:.3f} m, 기울기 {np.degrees(np.arctan(slope)):.1f}°, 코스 폭 y [{ys[on][0]:.2f}, {ys[on][-1]:.2f}]"
    )
    print(
        f"  사다리꼴 적합 잔차(프로파일 중앙값) {err * 1000:.1f} mm  (꼭짓점 어긋남 {shift:+.1f} px)"
    )

    # --- 새 사다리꼴 --------------------------------------------------------------------------
    slope_new = slope if a.slope_deg is None else np.tan(np.deg2rad(a.slope_deg))
    rise_new = hs * slope_new / vs
    target_h = a.height if a.height is not None else h_old * a.scale
    run_new = int(round(target_h / (hs * slope_new)))
    ideal_new = trapezoid(px.clip(0), up_start, run_new, plateau_len, rise_new, vs)
    h_new = np.rint(run_new * rise_new) * vs
    down_end_new = up_start + 2 * run_new + plateau_len
    if (down_end_new + 2) * hs + a.final_goal_offset > tile_len - 0.5:
        raise SystemExit("새 램프가 타일을 넘친다")
    Hn = H.copy()
    # 잔차는 roughness(±2.5 cm 안팎)여야 한다 — 삼각망 모서리 반칸 오차가 새 램프에 그대로 실리지 않게 자른다.
    resid = np.clip(H[on] - ideal_old[None, :], -0.03, 0.03)
    Hn[on] = ideal_new[None, :] + resid
    print(
        f"새 램프  : run {run_new} px ({run_new * hs:.1f} m), 높이 {h_new:.3f} m (x{h_new / h_old:.2f}), "
        f"기울기 {np.degrees(np.arctan(slope_new)):.1f}°, 내리막 끝 px {down_end_new}"
    )
    # 결과 검증: 새 오르막의 실제 기울기와, 행별 프로파일 둘레의 roughness 가 평지와 같은지
    core = np.where(on)[0][3:-3]
    new_prof = np.median(Hn[core], axis=0)
    up = (px > up_start + 1) & (px < up_start + run_new - 1)
    flat = px > down_end_new + 5
    deg = np.degrees(np.arctan(np.polyfit(xs[up], new_prof[up], 1)[0]))
    print(
        f"  검증: 오르막 {deg:.1f}° / 꼭대기 {new_prof[(px > up_start + run_new + 1) & (px < up_start + run_new + plateau_len - 1)].mean():.3f} m, "
        f"roughness std 오르막 {(Hn[core][:, up] - new_prof[up]).std() * 1000:.1f} mm vs 평지 "
        f"{(Hn[core][:, flat] - new_prof[flat]).std() * 1000:.1f} mm"
    )

    # --- goal: _lay_goals_over_trapezoid 와 같은 규칙, 마지막만 offset 고정 -----------------------
    def to_m(p):
        return p * hs + hs / 2 - tile_len / 2

    n = len(goals)
    new = goals.copy()  # y 흔들림은 원본 그대로
    new[1 : n - 1, 0] = to_m(
        np.linspace(up_start + round(0.5 / hs), down_end_new, n - 2)
    )
    new[-1, 0] = to_m(down_end_new) + a.final_goal_offset
    m["goals"] = m["goals"].copy()
    m["goals"][0, col] = new
    m["hfield"] = Hn.astype(np.float32)
    spawn_x = float(m["robot_spawn_pos_w"][0])
    print(
        f"goal x (스폰 기준): {np.round(new[:, 0] + origin[0] - spawn_x, 2).tolist()}"
    )
    print(
        f"  내리막 끝 {to_m(down_end_new) + origin[0] - spawn_x:.2f} m → 마지막 goal "
        f"{new[-1, 0] + origin[0] - spawn_x:.2f} m (원본은 내리막 끝 + "
        f"{goals[-1, 0] - goals[-2, 0]:.2f} m)"
    )

    tag = a.tag or f"ramp{a.scale:g}x"
    out = Path(a.out) if a.out else Path(a.meta).with_name(f"terrain_meta_{tag}.npz")
    np.savez_compressed(out, **m)
    print(f"저장: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
