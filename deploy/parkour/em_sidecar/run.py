"""EM 사이드카 실행 진입점.

    python -m em_sidecar.run                      # 기본값 (domain 0, lo)
    python -m em_sidecar.run --duration 30        # 30초만
    python -m em_sidecar.run --device cpu         # cupy 없이 (느림, 진단용)

unitree_mujoco 시뮬레이터가 먼저 떠 있어야 한다 (config.yaml 에 enable_lidar: 1).
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .sidecar import EmSidecar, SidecarCfg

HERE = Path(__file__).resolve().parent
CONTRACT = HERE.parent / "contract"
DEFAULT_EMCUPY = Path.home() / "workspace/codes/Isaaclab_Parkour/elevation_mapping_cupy"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contract-dir", default=str(CONTRACT))
    p.add_argument("--emcupy-root", default=str(DEFAULT_EMCUPY),
                   help="elevation_mapping_cupy 클론 위치 (Isaaclab_Parkour 서브모듈)")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--domain", type=int, default=0)
    p.add_argument("--iface", default="lo")
    p.add_argument("--topic", default="rt/parkour/scandots")
    p.add_argument("--duration", type=float, default=None, help="초 (미지정이면 무한)")
    p.add_argument("--train-noise", action="store_true",
                   help="학습과 같은 EM 입력 노이즈를 얹는다 (sim2sim 실험용).")
    p.add_argument("--train-noise-odom-mult", type=float, default=1.0,
                   help="odometry 노이즈(scale σ, walk, bias 범위) 배율 — 정책의 내성 측정용")
    p.add_argument("--train-noise-scale-bias", type=float, default=None,
                   help="odometry scale bias 를 고정값으로 (예: -0.06). 추정기 계통 편향 흉내")
    p.add_argument("--record", default=None,
                   help="tick 마다 (시각, base pose, scan, valid) 를 npz 로 남긴다. "
                        "주행 중에도 지도가 맞는지 지형과 대조하기 위한 것.")
    p.add_argument("--record-map", action="store_true",
                   help="--record 와 함께: tick 마다 EM 전체 지도도 남긴다 (영상용)")
    p.add_argument("--odom", choices=["sport", "leg"], default="sport",
                   help="base 위치 출처. sport=rt/sportmodestate, "
                        "leg=다리 운동학+IMU 자체 적분 (실기 저수준 제어용 후보).")
    p.add_argument("--leg-contact-thr", type=float, default=None, help="[N] leg 접촉 임계")
    p.add_argument("--leg-foot-radius", type=float, default=None, help="[m] leg 발 구름 보정")
    p.add_argument("--leg-no-seed", action="store_true",
                   help="leg 시작점을 sportmodestate 에 맞추지 않고 0 에서 시작 (실기 조건)")
    p.add_argument("--leg-shadow", action="store_true",
                   help="지도는 sport 로 만들고 leg 추정기는 옆에서 기록만 (추정기 단독 평가)")
    a = p.parse_args()

    leg_cfg = None
    if a.odom == "leg" or a.leg_shadow:
        from .leg_odometry import LegOdomCfg
        leg_cfg = LegOdomCfg()
        if a.leg_contact_thr is not None:
            leg_cfg.contact_force_thr = a.leg_contact_thr
        if a.leg_foot_radius is not None:
            leg_cfg.foot_radius = a.leg_foot_radius

    noise_cfg = None
    if a.train_noise and (a.train_noise_odom_mult != 1.0 or a.train_noise_scale_bias is not None):
        from .train_noise import TrainNoiseCfg
        k = a.train_noise_odom_mult
        d = TrainNoiseCfg()
        noise_cfg = TrainNoiseCfg(
            odom_scale_var=d.odom_scale_var * k * k,
            odom_pos_walk_std=d.odom_pos_walk_std * k,
            odom_scale_bias_max=d.odom_scale_bias_max * k,
            odom_scale_bias_fixed=a.train_noise_scale_bias,
        )

    cfg = SidecarCfg(
        contract_dir=Path(a.contract_dir),
        emcupy_root=Path(a.emcupy_root),
        device=a.device,
        domain_id=a.domain,
        interface=a.iface,
        publish_topic=a.topic,
        record_path=Path(a.record) if a.record else None,
        record_map=a.record_map,
        train_noise=a.train_noise,
        train_noise_cfg=noise_cfg,
        odom_source=a.odom,
        leg_odom=leg_cfg,
        leg_seed_from_sport=not a.leg_no_seed,
        leg_shadow=a.leg_shadow,
    )
    EmSidecar(cfg).run(duration=a.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
