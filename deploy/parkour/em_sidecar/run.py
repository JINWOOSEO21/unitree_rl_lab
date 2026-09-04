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
    p.add_argument("--record", default=None,
                   help="tick 마다 (시각, base pose, scan, valid) 를 npz 로 남긴다. "
                        "주행 중에도 지도가 맞는지 지형과 대조하기 위한 것.")
    a = p.parse_args()

    cfg = SidecarCfg(
        contract_dir=Path(a.contract_dir),
        emcupy_root=Path(a.emcupy_root),
        device=a.device,
        domain_id=a.domain,
        interface=a.iface,
        publish_topic=a.topic,
        record_path=Path(a.record) if a.record else None,
        train_noise=a.train_noise,
    )
    EmSidecar(cfg).run(duration=a.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
