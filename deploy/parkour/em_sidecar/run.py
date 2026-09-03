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
    a = p.parse_args()

    cfg = SidecarCfg(
        contract_dir=Path(a.contract_dir),
        emcupy_root=Path(a.emcupy_root),
        device=a.device,
        domain_id=a.domain,
        interface=a.iface,
        publish_topic=a.topic,
    )
    EmSidecar(cfg).run(duration=a.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
