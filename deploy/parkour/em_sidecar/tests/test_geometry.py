"""사이드카 기하 모듈 게이트 — IsaacLab 원본과 직접 대조한다.

두 가지를 본다.

1. 순기구학: contract/em_geometry.npz 에 함께 저장된 IsaacLab 실측 링크 위치를
   kinematics.Go2Kinematics 가 재현하는가. (IsaacLab 없이 돌아간다 — 기준값이
   npz 안에 있다.)

2. self-filter: selffilter.capsule_self_hits 가 학습 원본
   parkour_isaaclab/sensors/l1_scan_ray_caster.ray_capsule_penetrates 와 같은
   판정을 내는가. 원본을 **베껴 적지 않고 그대로 불러와** 비교한다 — 베껴 적으면
   같은 실수를 두 번 하게 된다. (그래서 이 검사는 Isaaclab_Parkour 체크아웃과
   torch 가 있어야 돌고, 없으면 SKIP 한다.)

사용법:
    python -m em_sidecar.tests.test_geometry [--parkour-repo <path>]
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG.parent))  # deploy/parkour 를 import 루트로

from em_sidecar.kinematics import Go2Kinematics  # noqa: E402
from em_sidecar.selffilter import capsule_self_hits  # noqa: E402

CONTRACT = PKG.parent / "contract" / "em_geometry.npz"


def load_original_capsule_fn(parkour_repo: Path):
    """isaaclab 을 스텁으로 끼우고 학습 원본 모듈을 그대로 적재한다."""
    src = parkour_repo / "parkour_isaaclab" / "sensors" / "l1_scan_ray_caster.py"
    if not src.exists():
        return None

    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any:
        def __init__(self, *a, **k):
            pass

        def __call__(self, *a, **k):
            return self

        def __class_getitem__(cls, item):
            return cls

    stub("isaaclab")
    stub("isaaclab.sensors", MultiMeshRayCaster=_Any, MultiMeshRayCasterCfg=_Any)
    stub("isaaclab.sensors.ray_caster")
    stub("isaaclab.sensors.ray_caster.patterns", LidarPatternCfg=_Any)
    stub("isaaclab.utils", configclass=lambda c: c)
    stub("isaaclab.utils.math", quat_apply=lambda *a, **k: None)

    spec = importlib.util.spec_from_file_location("l1_orig_capsule", src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.ray_capsule_penetrates


def test_fk(kin: Go2Kinematics) -> bool:
    print("[1] 순기구학 vs IsaacLab 실측 링크 위치 (base 프레임)")
    worst = 0.0
    for k in range(kin.q_test.shape[0]):
        P, _ = kin.link_poses_base(kin.q_test[k])
        err = np.linalg.norm(P - kin.pos_ref[k], axis=1)
        worst = max(worst, err.max())
        print(f"    자세 {k}: max {err.max()*1e3:8.5f} mm")
    ok = worst * 1e3 < 0.05
    print(f"    최대 {worst*1e3:.5f} mm → {'PASS' if ok else 'FAIL'}\n")
    return ok


def test_selffilter(kin: Go2Kinematics, parkour_repo: Path) -> bool | None:
    print("[2] self-filter vs 학습 원본 ray_capsule_penetrates")
    try:
        import torch
    except ImportError:
        print("    SKIP — torch 없음\n")
        return None
    orig = load_original_capsule_fn(parkour_repo)
    if orig is None:
        print(f"    SKIP — 원본 없음: {parkour_repo}\n")
        return None

    rng = np.random.default_rng(0)
    # 실제와 비슷한 조건: 센서 원점 기준 임의 방향 x 임의 거리.
    m = 4000
    d = rng.normal(size=(m, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    t = rng.uniform(0.05, 3.0, size=m)
    pts = d * t[:, None]

    # 캡슐은 임의 관절각의 실제 로봇 기하에서 가져온다 (센서 프레임).
    q = kin.q_test[3]
    base_pos = np.zeros(3)
    R_base = np.eye(3)
    a_w, b_w, r = kin.capsule_endpoints(q, base_pos, R_base)
    # 마운트: base 프레임 (0.28, 0, 0.10), 180° about x  (GO2_LIDAR_CFG)
    t_s = np.array([0.28, 0.0, 0.10])
    R_s = np.diag([1.0, -1.0, -1.0])
    a_s = (a_w - t_s) @ R_s
    b_s = (b_w - t_s) @ R_s

    # 원본은 캡슐 하나씩 받는다 — 하나씩 대조하고 마지막에 OR 결과도 대조한다.
    o_t = torch.zeros(1, m, 3, dtype=torch.float64)
    d_t = torch.tensor(pts / np.linalg.norm(pts, axis=1, keepdims=True)).unsqueeze(0)
    t_t = torch.tensor(np.linalg.norm(pts, axis=1)).unsqueeze(0)

    combined_ref = np.zeros(m, dtype=bool)
    n_bad = 0
    for c in range(len(r)):
        ref = orig(
            o_t, d_t, t_t,
            torch.tensor(a_s[c]).unsqueeze(0),
            torch.tensor(b_s[c]).unsqueeze(0),
            float(r[c]),
        )[0].numpy()
        got = capsule_self_hits(pts, a_s[c : c + 1], b_s[c : c + 1], r[c : c + 1])
        combined_ref |= ref
        n_bad += int((ref != got).sum())

    combined_got = capsule_self_hits(pts, a_s, b_s, r)
    n_bad_comb = int((combined_ref != combined_got).sum())
    hit_rate = combined_ref.mean()

    ok = n_bad == 0 and n_bad_comb == 0
    print(f"    점 {m}개 x 캡슐 {len(r)}개 — 개별 불일치 {n_bad}, 통합 불일치 {n_bad_comb}")
    print(f"    (자기 몸 판정 비율 {hit_rate*100:.1f}% — 0 이나 100 이면 시험이 무의미하다)")
    print(f"    → {'PASS' if ok else 'FAIL'}\n")
    return ok and 0.01 < hit_rate < 0.99


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--parkour-repo",
        default=str(Path.home() / "workspace/codes/Isaaclab_Parkour"),
    )
    a = ap.parse_args()

    kin = Go2Kinematics(CONTRACT)
    print(f"계약 파일: {CONTRACT}")
    print(f"링크 {len(kin.link_names)}개, 캡슐 {len(kin.capsules)}개, "
          f"scandots {kin.scan_offsets_xy.shape[0]}개\n")

    results = [test_fk(kin), test_selffilter(kin, Path(a.parkour_repo))]
    hard = [r for r in results if r is not None]
    ok = all(hard)
    print(f"[RESULT] {'OK' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
