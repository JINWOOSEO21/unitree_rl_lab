"""로봇이 실제로 램프/계단을 **넘었는가**, 아니면 옆으로 돌아갔는가.

앞서 나는 "전진 거리 13 m + z 평균 0.44" 만 보고 "0.4 m 단차를 올라 고원 주행" 이라고
했다. 그건 근거가 안 된다. 램프는 y 방향으로 −1.2~+1.2 m 폭에만 있고 그 바깥은
평지라서, y 로 밀려나면 장애물을 **우회**해도 거리는 그대로 나온다.

궤적 (x, y, z) 을 지형과 겹쳐 본다:
  - 로봇 밑 지형 높이 = terrain(x, y)
  - 몸 높이 여유 = base_z − terrain(x, y)   (정상 보행이면 ≈ 0.32 m)
  - 지형이 0.1 m 이상인 곳을 지났는가 = 실제로 올라갔는가
"""
import sys

import numpy as np

sys.path.insert(0, "/home/seo-jinwoo/workspace/codes/unitree_rl_lab/deploy/parkour")
from em_sidecar.kinematics import quat_to_mat  # noqa: E402
from em_sidecar.tests.test_live_terrain import META, Terrain  # noqa: E402

IMU_OFF = np.array([-0.02557, 0.0, 0.04232])
terr = Terrain(META)

path = sys.argv[1]
r = np.load(path)
sp_t = r["sp_t"] - r["sp_t"][0]
pos = r["sp_pos"].astype(float).copy()
# quat 은 lowstate 스트림 → 시각 보간 (자세 보정용, 수평이면 영향 미미)
qt = r["t"] - r["sp_t"][0]
quat = r["quat"]
n = min(len(qt), len(quat))
Q = np.stack([np.interp(sp_t, qt[:n], quat[:n, k]) for k in range(4)], axis=1)
Q /= np.linalg.norm(Q, axis=1, keepdims=True)
for k in range(len(pos)):
    pos[k] -= quat_to_mat(Q[k]) @ IMU_OFF        # imu site → base 원점

h = terr.height(pos[:, 0], pos[:, 1])
clear = pos[:, 2] - h

print(f"{path.split('/')[-1]}   {sp_t[-1]:.1f}초, {len(pos)} 표본")
print(f"{'t[s]':>6} {'x':>7} {'y':>7} {'base z':>8} {'지형 z':>8} {'여유':>7}")
for i in range(0, len(pos), max(1, len(pos) // 22)):
    print(f"{sp_t[i]:6.1f} {pos[i,0]:7.2f} {pos[i,1]:7.2f} {pos[i,2]:8.3f} "
          f"{h[i]:8.3f} {clear[i]:7.3f}")

print()
print(f"전진      x {pos[0,0]:+.2f} → {pos[-1,0]:+.2f}  ({pos[-1,0]-pos[0,0]:+.2f} m)")
print(f"측면 이동 y {pos[0,1]:+.2f} → {pos[-1,1]:+.2f}  (|최대| {np.abs(pos[:,1]).max():.2f} m)")
print(f"로봇 밑 지형 최대 높이 {np.nanmax(h)*100:.1f} cm")
print(f"몸 높이 여유 평균 {np.nanmean(clear):.3f} m (정상 보행 ≈ 0.32)")

# 램프 통과 판정: 램프는 y∈[-1.2,1.2], x∈[1.45,5.5], 정상 38 cm
on_ramp = (h > 0.10)
print()
if on_ramp.any():
    i0 = np.argmax(on_ramp)
    print(f"지형 10 cm 이상 위에 있던 시간 {on_ramp.mean()*100:.0f}%  "
          f"(처음 t={sp_t[i0]:.1f}s, x={pos[i0,0]:.2f})")
    print(f"올라간 최대 높이 {np.nanmax(h[on_ramp])*100:.1f} cm")
    top = h > 0.30
    print(f"고원(30 cm 이상) 체류 {top.mean()*100:.0f}%")
else:
    print("지형 10 cm 이상인 곳을 **한 번도 밟지 않았다** → 장애물을 우회했다")

# 우회 판정: x 가 램프 구간(1.45~5.5)을 지나는 동안 y 가 램프 폭 밖이었는가
seg = (pos[:, 0] > 1.45) & (pos[:, 0] < 5.5)
if seg.any():
    ybar = pos[seg, 1]
    outside = np.abs(ybar) > 1.3
    print(f"\nx 1.45~5.5 구간 통과: {seg.sum()} 표본, y 범위 [{ybar.min():+.2f}, {ybar.max():+.2f}]")
    print(f"  램프 폭(|y|<1.3) 밖에 있던 비율 {outside.mean()*100:.0f}%"
          + ("   ← 우회" if outside.mean() > 0.5 else "   ← 정면 통과"))
else:
    print("\n램프 구간(x 1.45~5.5)에 도달하지 못했다")
