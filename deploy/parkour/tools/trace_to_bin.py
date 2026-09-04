"""골든 트레이스(npz) → C++ 테스트가 읽는 평탄 바이너리.

C++ 에서 npz 를 직접 읽으려면 zlib+파서를 붙여야 하는데, 배포 바이너리에 넣고
싶지 않은 의존성이다. 검사 전용이므로 단순한 형식으로 한 번 변환해 둔다.

형식 (전부 리틀엔디언):
    char[4] "PKGT"
    int32   T            프레임 수
    float32 prop         [T][53]
    float32 scan         [T][132]
    float32 hist         [T][530]
    float32 actions      [T][12]
    float32 joint_pos    [T][12]     IsaacLab 관절 순서
    float32 joint_vel    [T][12]
    float32 gyro         [T][3]      root_ang_vel_b
    float32 quat         [T][4]      (w,x,y,z)
    float32 contact_now  [T][4][3]   발 FL,FR,RL,RR (접촉센서 순서로 정렬됨)
    float32 contact_prev [T][4][3]

사용:
    python tools/trace_to_bin.py --npz contract/golden_trace.npz --out /tmp/trace.bin
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

ORDER = [
    ("prop", 53),
    ("scan", 132),
    ("hist", 530),
    ("actions", 12),
    ("joint_pos", 12),
    ("joint_vel", 12),
    ("root_ang_vel_b", 3),
    ("root_quat_w", 4),
    ("contact_now", 12),   # 4 x 3
    ("contact_prev", 12),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    d = np.load(a.npz)
    missing = [k for k, _ in ORDER if k not in d.files]
    if missing:
        print(f"npz 에 없는 항목: {missing}")
        print("  dump_golden_trace.py 를 최신판으로 다시 떠야 한다 "
              "(contact_now/contact_prev 는 나중에 추가됐다).")
        return 1

    T = d["prop"].shape[0]
    with open(a.out, "wb") as f:
        f.write(b"PKGT")
        f.write(struct.pack("<i", T))
        for name, dim in ORDER:
            arr = np.asarray(d[name], dtype=np.float32)
            arr = arr[:, 0]                      # env 축 제거 (num_envs=1)
            arr = arr.reshape(T, -1)
            if arr.shape[1] != dim:
                print(f"{name}: 차원 {arr.shape[1]} != 기대 {dim}")
                return 1
            f.write(np.ascontiguousarray(arr).tobytes())

    print(f"{a.out} 기록: {T} 프레임")
    print("  foot 순서:", [str(s) for s in d["foot_body_names"]]
          if "foot_body_names" in d.files else "(기록 없음)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
