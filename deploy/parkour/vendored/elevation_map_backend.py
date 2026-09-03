# ===========================================================================
# VENDORED — 수정하지 말 것.
#
#   출처  : Isaaclab_Parkour  parkour_isaaclab/envs/mdp/elevation_map_backend.py
#   커밋  : 4e25a85e7fa8589e9f94dfdd00fd3b55c46616a4  (branch sim2sim)
#   sha256: a36bb062b0c9d5f90768a3ccffc8834a75ac9d1758a4397b3e13c18691e09b97
#
# 배포측은 IsaacLab 을 import 하지 않으므로 학습 저장소를 참조할 수 없다. 그렇다고
# 다시 구현하면 "학습과 같은 지도"라는 보장이 사라진다. 그래서 원본을 **글자 그대로**
# 복사해 둔다 (이 헤더만 앞에 붙였다). 원본이 바뀌면 위 sha256 이 어긋나고
# em_sidecar/tests/test_vendored.py 가 그것을 잡아낸다.
#
# elevation_mapping_cupy 클론이 필요하다. 위치는 EMCUPY_ROOT 환경변수로 준다
# (Isaaclab_Parkour 의 서브모듈을 그대로 가리키면 된다).
# ===========================================================================
"""elevation_mapping_cupy 래퍼 — per-env 인스턴스, ROS 없이 코어만 사용.

계획서 docs/emcupy_student_plan.md §2.4. 설계 요점:
- 코어 라이브러리는 순수 python+cupy 라 그대로 쓰되, 확인된 함정들을 여기서 흡수한다:
  1) import 부작용으로 걸리는 프로세스 전역 managed-memory allocator 를 일반
     device pool 로 되돌린다 (IsaacLab/torch 와 VRAM 경합 방지).
  2) input_pointcloud 가 t 를 in-place 로 바꾸므로 복사본을 넘긴다.
  3) move()/move_to() 의 grid shift 부호가 상반 — move_to 만 쓴다.
  4) traversability CNN 은 우리 파이프라인에 불필요 — zero-stub 으로 교체한다.
     (normal map 은 visibility cleanup 의 cos 문턱이 쓰므로 살려 둔다.)
- 이 모듈은 isaaclab 을 import 하지 않는다. cupy/em_cupy 는 지연 import 라
  샘플링 순수 함수(sample_scan_heights)는 CPU 단위 테스트에서 그대로 쓸 수 있다.
"""
from __future__ import annotations

import os
from pathlib import Path

import torch

# (cp, ElevationMap, Parameter, repo_root) — _import_emcupy() 가 채운다.
_EM = None


def _find_emcupy_root() -> Path:
    """elevation_mapping_cupy 클론 위치를 찾는다.

    main checkout 의 repo root 에 클론되어 있고(untracked), 워크트리에서 돌 때는
    거기 없으므로 상위(.claude/worktrees/<name> -> repo root)도 훑는다.
    EMCUPY_ROOT 환경변수가 있으면 그것이 이긴다.
    """
    candidates = []
    if os.environ.get("EMCUPY_ROOT"):
        candidates.append(Path(os.environ["EMCUPY_ROOT"]))
    here = Path(__file__).resolve()
    # parents[3] == <repo root>/parkour_isaaclab/envs/mdp 기준 repo root.
    for up in (3, 4, 5, 6):
        if up < len(here.parents):
            candidates.append(here.parents[up] / "elevation_mapping_cupy")
    for root in candidates:
        if (root / "elevation_mapping_cupy" / "script" / "elevation_mapping_cupy" / "elevation_mapping.py").exists():
            return root
    raise ImportError(
        "elevation_mapping_cupy 클론을 찾지 못했다. repo root 에 클론하거나 "
        "EMCUPY_ROOT 환경변수로 위치를 지정할 것. 찾아본 곳: "
        + ", ".join(str(c) for c in candidates)
    )


def _import_emcupy():
    global _EM
    if _EM is not None:
        return _EM
    import sys

    root = _find_emcupy_root()
    script_dir = str(root / "elevation_mapping_cupy" / "script")
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    import cupy as cp
    from elevation_mapping_cupy import ElevationMap, Parameter  # noqa: E402

    # 함정 1: elevation_mapping.py 가 import 시점에 cupy 전역 allocator 를
    # managed(unified) memory pool 로 바꿔 버린다. torch 와 VRAM 을 두 풀이
    # 나눠 잡는 것은 어쩔 수 없지만, managed pool 은 페이지 폴트 비용까지
    # 얹으므로 일반 device pool 로 되돌린다.
    cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)
    _EM = (cp, ElevationMap, Parameter, root)
    return _EM


def sample_scan_heights(
    maps: torch.Tensor,
    centers: torch.Tensor,
    points_xy: torch.Tensor,
    base_z: torch.Tensor,
    resolution: float,
    cell_n: int,
    height_offset: float = 0.3,
    clip_range: float = 1.0,
):
    """elevation map 을 scandot 위치에서 샘플해 teacher 와 동일한 높이 obs 를 만든다.

    순수 torch 함수 (cupy/isaaclab 무관 — CPU 단위 테스트 대상).

    Args:
        maps: (N, 7, H, W) — em_cupy elevation_map 레이어 스택.
              [0]=elevation(맵 center z 상대값), [2]=is_valid, [5]=upper_bound, [6]=is_upper_bound
        centers: (N, 3) — 각 맵의 center (odom frame).
        points_xy: (N, P, 2) — 샘플 위치 (odom frame).
        base_z: (N,) — 로봇 base 높이 (odom frame).
        resolution, cell_n: 맵 그리드 사양.

    Returns:
        h_obs: (N, P) — clip(base_z − h_abs − height_offset, ±clip_range).
               invalid 셀 cascade: valid → h, 아니면 upper_bound, 아니면 0.
        valid_frac: (N, P) — 직접 관측(valid) 4이웃 가중치 합 (진단용).
        ub_frac: (N, P) — upper_bound 4이웃 가중치 합. 둘 다 0 인 점만
               '완전 미지'(fallback 0 사용)다 — 예: 정지 상태에서 몸 아래처럼
               관측도 상공 통과 ray 도 없는 셀.
    """
    n, p = points_xy.shape[0], points_xy.shape[1]
    # 커널 규약: i = floor((x−c)/res + 0.5·cell_n), 셀 i 의 중심은 연속좌표 i+0.5.
    # bilinear 는 셀 중심 격자 위에서 한다: u 정수값 == 셀 중심.
    u = (points_xy[..., 0] - centers[:, None, 0]) / resolution + 0.5 * cell_n - 0.5
    v = (points_xy[..., 1] - centers[:, None, 1]) / resolution + 0.5 * cell_n - 0.5
    u0 = torch.floor(u).long()
    v0 = torch.floor(v).long()
    fu = (u - u0.to(u.dtype)).unsqueeze(-1)  # (N,P,1)
    fv = (v - v0.to(v.dtype)).unsqueeze(-1)

    # 4 이웃 (dx, dy) 와 bilinear 가중치
    w = torch.cat(
        [(1 - fu) * (1 - fv), (1 - fu) * fv, fu * (1 - fv), fu * fv], dim=-1
    )  # (N,P,4)
    ux = torch.stack([u0, u0, u0 + 1, u0 + 1], dim=-1)  # (N,P,4)
    vy = torch.stack([v0, v0 + 1, v0, v0 + 1], dim=-1)

    # 1셀 보더 링(커널의 is_inside 규약)과 맵 밖은 invalid 취급
    inside = (ux >= 1) & (ux <= cell_n - 2) & (vy >= 1) & (vy <= cell_n - 2)
    ux = ux.clamp(0, cell_n - 1)
    vy = vy.clamp(0, cell_n - 1)

    flat = (ux * cell_n + vy).view(n, -1)  # (N, P*4)

    def gather(layer: int) -> torch.Tensor:
        return maps[:, layer].reshape(n, -1).gather(1, flat).view(n, p, 4)

    h = gather(0)
    valid = (gather(2) > 0.5) & inside
    ub = gather(5)
    is_ub = (gather(6) > 0.5) & inside

    w_valid = w * valid.to(w.dtype)
    w_ub = w * is_ub.to(w.dtype)
    sum_valid = w_valid.sum(-1)
    sum_ub = w_ub.sum(-1)

    eps = 1e-9
    h_valid = (w_valid * h).sum(-1) / sum_valid.clamp_min(eps)
    h_ubval = (w_ub * ub).sum(-1) / sum_ub.clamp_min(eps)

    h_abs_rel = torch.where(sum_valid > eps, h_valid, h_ubval)  # 맵 center z 상대값
    h_abs = h_abs_rel + centers[:, None, 2]
    h_obs = torch.clip(base_z[:, None] - h_abs - height_offset, -clip_range, clip_range)
    # 관측도 상한도 없는 셀: teacher 식 기준 0 (= base 기준 평지 가정)
    h_obs = torch.where((sum_valid > eps) | (sum_ub > eps), h_obs, torch.zeros_like(h_obs))
    return h_obs, sum_valid, sum_ub


class ElevationMapBackend:
    """per-env ElevationMap 인스턴스 묶음.

    성능 노트 (계획서 §2.4/리스크 1): stock 커널은 단일 맵 설계라 여기서는
    python 루프로 env 별 인스턴스를 직렬 갱신한다. Phase 3 벤치마크에서
    이 루프가 병목으로 판정되면 커널 3개(add_points/error_counting/average_map)에
    env 배치 차원을 더한 batched 포트로 교체한다 (인터페이스는 이 클래스 유지).
    """

    # elevation_map 레이어 인덱스 (em_cupy 규약)
    L_ELEV, L_VAR, L_VALID, L_UB, L_IS_UB = 0, 1, 2, 5, 6

    def __init__(
        self,
        num_envs: int,
        device: str,
        resolution: float = 0.1,
        map_length: float = 3.2,
        sensor_noise_factor: float = 0.05,
        min_valid_distance: float = 0.10,
        max_ray_length: float = 3.0,
        enable_visibility_cleanup: bool = True,
    ):
        cp, ElevationMap, Parameter, root = _import_emcupy()
        self.cp = cp
        self.device = device
        self.num_envs = num_envs
        self.resolution = float(resolution)

        cfg_dir = root / "elevation_mapping_cupy" / "config" / "core"
        param = Parameter(
            use_chainer=False,
            weight_file=str(cfg_dir / "weights.dat"),
            plugin_config_file=str(cfg_dir / "plugin_config.yaml"),
        )
        param.resolution = float(resolution)
        param.map_length = float(map_length)
        param.sensor_noise_factor = float(sensor_noise_factor)
        # 캡슐 self-filter(t0=0.12m)가 근거리를 이미 걸렀다 — 내장 필터는 그보다 안쪽만.
        param.min_valid_distance = float(min_valid_distance)
        # GT(+백색잡음) odometry 라 드리프트 보정은 끈다. error_counting 커널은
        # 그래도 돌지만(update_map_with_kernel 고정 순서) 보정 분기는 스킵된다.
        param.enable_drift_compensation = False
        # upper_bound(=Q3 cascade 2단계)를 만들려면 visibility cleanup 이 필요하다.
        # 비용은 점당 max_ray_length/resolution 회 반복 — 기본 yaml(10m/0.04m) 금지.
        param.enable_visibility_cleanup = bool(enable_visibility_cleanup)
        param.max_ray_length = float(max_ray_length)
        # 맵(3.2m)이 clear range(4m 기본)보다 작아 사실상 전맵 소거가 된다 — 끈다.
        param.enable_overlap_clearance = False
        # semantic/이미지 경로 전부 봉인
        param.subscriber_cfg = {}
        param.additional_layers = []
        param.fusion_algorithms = []
        param.update()  # 필수: cell_n 등 파생값 계산
        self.param = param
        self.cell_n = int(param.cell_n)

        self.maps = [ElevationMap(param) for _ in range(num_envs)]
        # traversability CNN 우회: update_map_with_kernel 이 매 갱신 호출하지만
        # 우리는 그 레이어를 안 쓴다. dilation/normal 은 visibility cleanup 이
        # 쓰므로 그대로 두고 CNN forward 만 zero-stub 으로 바꾼다.
        self._trav_zero = cp.zeros((1, 1, self.cell_n - 6, self.cell_n - 6), dtype=cp.float32)
        for m in self.maps:
            m.traversability_filter = self._trav_stub

    def _trav_stub(self, _x):
        return self._trav_zero

    def clear(self, env_ids) -> None:
        """env reset(teleport) 시 해당 맵 초기화. 다음 move_to 가 다시 중심을 맞춘다."""
        for i in env_ids:
            self.maps[int(i)].clear()

    @torch.no_grad()
    def update(
        self,
        points_sensor: list[torch.Tensor],
        R_sensor: torch.Tensor,
        t_sensor: torch.Tensor,
        base_pos: torch.Tensor,
        R_base: torch.Tensor,
    ) -> None:
        """한 tick 의 관측을 전 env 맵에 반영한다.

        Args:
            points_sensor: env 별 (Mi, 3) 센서 프레임 점군 (self-hit 제거 후).
            R_sensor/t_sensor: (N,3,3)/(N,3) — odom frame 기준 센서 pose (노이즈 포함).
            base_pos/R_base: (N,3)/(N,3,3) — odom frame 기준 base pose (노이즈 포함).
        """
        cp = self.cp
        for i, m in enumerate(self.maps):
            # move_to 를 input 보다 먼저: 커널이 "현재 center" 기준으로 셀을 계산한다.
            m.move_to(cp.asarray(base_pos[i]), cp.asarray(R_base[i]))
            pts = points_sensor[i]
            if pts.shape[0] > 0:
                m.input_pointcloud(
                    cp.asarray(pts.contiguous()),
                    ["x", "y", "z"],
                    cp.asarray(R_sensor[i].contiguous()),
                    # 함정 2: t 는 커널 진입 전에 in-place 로 center 를 빼므로 복사본.
                    cp.asarray(t_sensor[i].contiguous()).copy(),
                    0.0,
                    0.0,
                )
            m.update_variance()
            m.update_time()

    def layers(self) -> torch.Tensor:
        """(N, 7, H, W) torch 뷰 스택. as_tensor 는 zero-copy, stack 이 1회 복사."""
        return torch.stack([torch.as_tensor(m.elevation_map, device=self.device) for m in self.maps])

    def centers(self) -> torch.Tensor:
        return torch.stack([torch.as_tensor(m.center, device=self.device) for m in self.maps])

    def sample(self, points_xy: torch.Tensor, base_z: torch.Tensor):
        """scandot 위치 샘플 → (h_obs, valid_frac). 규약은 sample_scan_heights 참조."""
        return sample_scan_heights(
            self.layers(), self.centers(), points_xy, base_z, self.resolution, self.cell_n
        )


# ---------------------------------------------------------------------------
# Batched 포트 (계획서 §2.4 Phase 3′)
#
# 위 ElevationMapBackend(인스턴스 루프)는 tick 당 인스턴스 x 커널 ~8회의 런치
# 오버헤드가 지배해 192 env 실측 250ms/tick 이었다. 아래는 em_cupy 의 커널
# 소스(MIT, Takahiro Miki)를 env 배치 차원이 있는 형태로 수정해 tick 당 커널
# 몇 회로 줄인 것이다. 수식/셀 인덱싱/upper_bound 로직은 stock 과 동일하며,
# 회귀 테스트(scripts/emcupy_check/regression_batched.py)가 동일 입력 → 동일
# 출력을 검증한다. stock 과 맞추기 위한 재현 사항:
#  - error_counting 커널 포함 (newmap 의 점 카운트 레이어 3,4 를 add_points 의
#    wall-sharpening/cleanup 분기가 읽는다 — 드리프트 보정이 꺼져 있어도 필요)
#  - traversability CNN 스텁의 부작용(map layer3 내부=0) 재현 — 다음 tick 의
#    error_counting inlier 판정이 이 레이어를 본다
# ---------------------------------------------------------------------------

import string  # noqa: E402


def _batched_preamble(res, n, sensor_noise_factor, min_valid_distance, max_height_range,
                      ra, rb, rc):
    """map_utils 의 배치판: get_map_idx 가 env 번호 b 를 받는다. width==height==n."""
    return string.Template(
        """
        __device__ float16 clamp(float16 x, float16 min_x, float16 max_x) {
            return max(min(x, max_x), min_x);
        }
        __device__ int get_x_idx(float16 x, float16 center) {
            int i = (x - center) / ${res} + 0.5 * ${n};
            return i;
        }
        __device__ int get_y_idx(float16 y, float16 center) {
            int i = (y - center) / ${res} + 0.5 * ${n};
            return i;
        }
        __device__ bool is_inside(int idx) {
            int idx_x = idx / ${n};
            int idx_y = idx % ${n};
            if (idx_x == 0 || idx_x == ${n} - 1) { return false; }
            if (idx_y == 0 || idx_y == ${n} - 1) { return false; }
            return true;
        }
        __device__ int get_idx(float16 x, float16 y, float16 center_x, float16 center_y) {
            int idx_x = clamp(get_x_idx(x, center_x), 0, ${n} - 1);
            int idx_y = clamp(get_y_idx(y, center_y), 0, ${n} - 1);
            return ${n} * idx_x + idx_y;
        }
        __device__ int get_map_idx(int b, int idx, int layer_n) {
            return (b * 7 + layer_n) * ${n} * ${n} + idx;
        }
        __device__ int get_norm_idx(int b, int idx, int layer_n) {
            return (b * 3 + layer_n) * ${n} * ${n} + idx;
        }
        __device__ float transform_p(float16 x, float16 y, float16 z,
                                     float16 r0, float16 r1, float16 r2, float16 t) {
            return r0 * x + r1 * y + r2 * z + t;
        }
        __device__ float z_noise(float16 z){ return ${sensor_noise_factor} * z * z; }
        __device__ float point_sensor_distance(float16 x, float16 y, float16 z,
                                               float16 sx, float16 sy, float16 sz) {
            return (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
        }
        __device__ bool is_valid(float16 x, float16 y, float16 z,
                               float16 sx, float16 sy, float16 sz) {
            float d = point_sensor_distance(x, y, z, sx, sy, sz);
            float dxy = max(sqrt(x * x + y * y) - ${rb}, 0.0);
            if (d < ${min_valid_distance} * ${min_valid_distance}) { return false; }
            else if (z - sz > dxy * ${ra} + ${rc} || z - sz > ${max_height_range}) { return false; }
            else { return true; }
        }
        __device__ float ray_vector(float16 tx, float16 ty, float16 tz,
                                    float16 px, float16 py, float16 pz,
                                    float16& rx, float16& ry, float16& rz){
            float16 vx = px - tx; float16 vy = py - ty; float16 vz = pz - tz;
            float16 norm = sqrt(vx * vx + vy * vy + vz * vz);
            if (norm > 0) { rx = vx / norm; ry = vy / norm; rz = vz / norm; }
            else { rx = 0; ry = 0; rz = 0; }
            return norm;
        }
        __device__ float inner_product(float16 x1, float16 y1, float16 z1,
                                       float16 x2, float16 y2, float16 z2) {
            return (x1 * x2 + y1 * y2 + z1 * z2);
        }
        """
    ).substitute(res=res, n=n, sensor_noise_factor=sensor_noise_factor,
                 min_valid_distance=min_valid_distance, max_height_range=max_height_range,
                 ra=ra, rb=rb, rc=rc)


class BatchedElevationMapBackend:
    """ElevationMapBackend 와 동일 인터페이스의 batched 구현.

    맵을 (B,7,n,n) 단일 cupy 배열로 유지하고 tick 당 커널 5회
    (error_counting/add_points/average/dilation/normal)로 전 env 를 갱신한다.
    move_to(그리드 roll)와 시간/분산 갱신은 torch 배치 연산으로 대체.
    """

    def __init__(
        self,
        num_envs: int,
        device: str,
        resolution: float = 0.1,
        map_length: float = 3.2,
        sensor_noise_factor: float = 0.05,
        min_valid_distance: float = 0.10,
        max_ray_length: float = 3.0,
        enable_visibility_cleanup: bool = True,
    ):
        cp, _ElevationMap, Parameter, root = _import_emcupy()
        self.cp = cp
        self.device = device
        self.num_envs = int(num_envs)
        self.resolution = float(resolution)

        # 파라미터는 stock Parameter 의 기본값 체계를 그대로 따른다 (값 비교 회귀의 전제)
        p = Parameter()
        p.resolution = float(resolution)
        p.map_length = float(map_length)
        p.update()
        self.param = p
        self.cell_n = int(p.cell_n)
        n, B = self.cell_n, self.num_envs

        self.initial_variance = float(p.initial_variance)
        self.time_variance = float(p.time_variance)
        self.time_interval = float(p.time_interval)

        self.maps = cp.zeros((B, 7, n, n), dtype=cp.float32)
        self.maps[:, 1] += self.initial_variance
        self.maps[:, 3] += 1.0
        self.newmap = cp.zeros((B, 7, n, n), dtype=cp.float32)
        self.norm = cp.zeros((B, 3, n, n), dtype=cp.float32)
        self.dil = cp.zeros((B, n, n), dtype=cp.float32)
        self.dil_mask = cp.zeros((B, n, n), dtype=cp.float32)
        self.centers = torch.zeros(B, 3, device=device)
        self._zeros_b = cp.zeros(B, dtype=cp.float32)  # center_x/center_y (사전 차감이라 0)
        self._err = cp.zeros(B, dtype=cp.float32)
        self._err_cnt = cp.zeros(B, dtype=cp.float32)

        pre = _batched_preamble(
            p.resolution, n, p.sensor_noise_factor, float(min_valid_distance),
            p.max_height_range, p.ramped_height_range_a, p.ramped_height_range_b,
            p.ramped_height_range_c,
        )

        # --- add_points (stock custom_kernels.add_points_kernel 의 배치판) ---
        self._add_points = cp.ElementwiseKernel(
            in_params="raw U center_x, raw U center_y, raw U R, raw U t, raw U norm_map, raw int32 env_id",
            out_params="raw U p, raw U map, raw T newmap",
            preamble=pre,
            operation=string.Template(
                """
                int b = env_id[i];
                U rx = p[i * 3];
                U ry = p[i * 3 + 1];
                U rz = p[i * 3 + 2];
                U x = transform_p(rx, ry, rz, R[b*9+0], R[b*9+1], R[b*9+2], t[b*3+0]);
                U y = transform_p(rx, ry, rz, R[b*9+3], R[b*9+4], R[b*9+5], t[b*3+1]);
                U z = transform_p(rx, ry, rz, R[b*9+6], R[b*9+7], R[b*9+8], t[b*3+2]);
                U v = z_noise(rz);
                int idx = get_idx(x, y, center_x[b], center_y[b]);
                if (is_valid(x, y, z, t[b*3+0], t[b*3+1], t[b*3+2])) {
                    if (is_inside(idx)) {
                        U map_h = map[get_map_idx(b, idx, 0)];
                        U map_v = map[get_map_idx(b, idx, 1)];
                        U num_points = newmap[get_map_idx(b, idx, 4)];
                        if (abs(map_h - z) > (map_v * ${mahalanobis_thresh})) {
                            atomicAdd(&map[get_map_idx(b, idx, 1)], ${outlier_variance});
                        }
                        else {
                            if (${enable_edge_shaped} && (num_points > ${wall_num_thresh}) && (z < map_h - map_v * ${mahalanobis_thresh} / num_points)) {
                              // continue;
                            }
                            else {
                                T new_h = (map_h * v + z * map_v) / (map_v + v);
                                T new_v = (map_v * v) / (map_v + v);
                                atomicAdd(&newmap[get_map_idx(b, idx, 0)], new_h);
                                atomicAdd(&newmap[get_map_idx(b, idx, 1)], new_v);
                                atomicAdd(&newmap[get_map_idx(b, idx, 2)], 1.0);
                                map[get_map_idx(b, idx, 2)] = 1;
                                map[get_map_idx(b, idx, 4)] = 0.0;
                                map[get_map_idx(b, idx, 5)] = new_h;
                                map[get_map_idx(b, idx, 6)] = 0.0;
                            }
                        }
                    }
                }
                if (${enable_visibility_cleanup}) {
                    float16 ray_x, ray_y, ray_z;
                    float16 ray_length = ray_vector(t[b*3+0], t[b*3+1], t[b*3+2], x, y, z, ray_x, ray_y, ray_z);
                    ray_length = min(ray_length, (float16)${max_ray_length});
                    int last_nidx = -1;
                    for (float16 s=${ray_step}; s < ray_length; s+=${ray_step}) {
                        U nx = t[b*3+0] + ray_x * s;
                        U ny = t[b*3+1] + ray_y * s;
                        U nz = t[b*3+2] + ray_z * s;
                        int nidx = get_idx(nx, ny, center_x[b], center_y[b]);
                        if (last_nidx == nidx) {continue;}
                        else {last_nidx = nidx;}
                        if (!is_inside(nidx)) {continue;}

                        U nmap_h = map[get_map_idx(b, nidx, 0)];
                        U nmap_v = map[get_map_idx(b, nidx, 1)];
                        U nmap_valid = map[get_map_idx(b, nidx, 2)];
                        U non_updated_t = map[get_map_idx(b, nidx, 4)];
                        U nmap_upper = map[get_map_idx(b, nidx, 5)];
                        U nmap_is_upper = map[get_map_idx(b, nidx, 6)];

                        float16 d = (x - nx) * (x - nx) + (y - ny) * (y - ny) + (z - nz) * (z - nz);
                        if (d < 0.1 || !is_valid(x, y, z, t[b*3+0], t[b*3+1], t[b*3+2])) {continue;}

                        if (nmap_valid < 0.5) {
                          if (nz < nmap_upper || nmap_is_upper < 0.5) {
                            map[get_map_idx(b, nidx, 5)] = nz;
                            map[get_map_idx(b, nidx, 6)] = 1.0f;
                          }
                          continue;
                        }
                        if (non_updated_t < 0.5) {continue;}

                        if (nmap_h > nz + 0.01 - min(nmap_v, 1.0) * 0.05) {
                            U norm_x = norm_map[get_norm_idx(b, nidx, 0)];
                            U norm_y = norm_map[get_norm_idx(b, nidx, 1)];
                            U norm_z = norm_map[get_norm_idx(b, nidx, 2)];
                            float product = inner_product(ray_x, ray_y, ray_z, norm_x, norm_y, norm_z);
                            if (fabs(product) < ${cleanup_cos_thresh}) {continue;}
                            U num_points = newmap[get_map_idx(b, nidx, 3)];
                            if (num_points > ${wall_num_thresh} && non_updated_t < 1.0) {continue;}

                            atomicAdd(&map[get_map_idx(b, nidx, 2)], -${cleanup_step}/(ray_length / ${max_ray_length}));
                            atomicAdd(&map[get_map_idx(b, nidx, 1)], ${outlier_variance});
                            if (nz < nmap_upper || nmap_is_upper < 0.5) {
                                map[get_map_idx(b, nidx, 5)] = nz;
                                map[get_map_idx(b, nidx, 6)] = 1.0f;
                            }
                        }
                    }
                }
                p[i * 3]= idx;
                p[i * 3 + 1] = is_valid(x, y, z, t[b*3+0], t[b*3+1], t[b*3+2]);
                p[i * 3 + 2] = is_inside(idx);
                """
            ).substitute(
                mahalanobis_thresh=p.mahalanobis_thresh,
                outlier_variance=p.outlier_variance,
                wall_num_thresh=p.wall_num_thresh,
                ray_step=p.resolution / 2 ** 0.5,
                max_ray_length=float(max_ray_length),
                cleanup_step=p.cleanup_step,
                cleanup_cos_thresh=p.cleanup_cos_thresh,
                enable_edge_shaped=int(p.enable_edge_sharpen),
                enable_visibility_cleanup=int(enable_visibility_cleanup),
            ),
            name="add_points_kernel_batched",
        )

        # --- error_counting (드리프트 보정은 안 쓰지만 newmap 점 카운트가 필요) ---
        self._error_counting = cp.ElementwiseKernel(
            in_params="raw U map, raw U p, raw U center_x, raw U center_y, raw U R, raw U t, raw int32 env_id",
            out_params="raw U newmap, raw T error, raw T error_cnt",
            preamble=pre,
            operation=string.Template(
                """
                int b = env_id[i];
                U rx = p[i * 3];
                U ry = p[i * 3 + 1];
                U rz = p[i * 3 + 2];
                U x = transform_p(rx, ry, rz, R[b*9+0], R[b*9+1], R[b*9+2], t[b*3+0]);
                U y = transform_p(rx, ry, rz, R[b*9+3], R[b*9+4], R[b*9+5], t[b*3+1]);
                U z = transform_p(rx, ry, rz, R[b*9+6], R[b*9+7], R[b*9+8], t[b*3+2]);
                if (!is_valid(x, y, z, t[b*3+0], t[b*3+1], t[b*3+2])) {return;}
                int idx = get_idx(x, y, center_x[b], center_y[b]);
                if (!is_inside(idx)) { return; }
                U map_h = map[get_map_idx(b, idx, 0)];
                U map_v = map[get_map_idx(b, idx, 1)];
                U map_valid = map[get_map_idx(b, idx, 2)];
                U map_t = map[get_map_idx(b, idx, 3)];
                if (map_valid > 0.5 && (abs(map_h - z) < (map_v * ${mahalanobis_thresh}))
                    && map_v < ${outlier_variance} / 2.0
                    && map_t > ${traversability_inlier}) {
                    T e = z - map_h;
                    atomicAdd(&error[b], e);
                    atomicAdd(&error_cnt[b], 1);
                    atomicAdd(&newmap[get_map_idx(b, idx, 3)], 1.0);
                }
                atomicAdd(&newmap[get_map_idx(b, idx, 4)], 1.0);
                """
            ).substitute(
                mahalanobis_thresh=p.mahalanobis_thresh,
                outlier_variance=p.outlier_variance,
                traversability_inlier=p.traversability_inlier,
            ),
            name="error_counting_kernel_batched",
        )

        # --- average_map: 셀 단위 elementwise (i 가 B*n*n 을 순회) ---
        cell_pre = string.Template(
            """
            __device__ int get_map_idx(int b, int cell, int layer_n) {
                return (b * 7 + layer_n) * ${n} * ${n} + cell;
            }
            """
        ).substitute(n=n)
        self._average_map = cp.ElementwiseKernel(
            in_params="raw U newmap",
            out_params="raw U map",
            preamble=cell_pre,
            operation=string.Template(
                """
                int cell = i % (${n} * ${n});
                int b = i / (${n} * ${n});
                U v = map[get_map_idx(b, cell, 1)];
                U valid = map[get_map_idx(b, cell, 2)];
                U new_h = newmap[get_map_idx(b, cell, 0)];
                U new_v = newmap[get_map_idx(b, cell, 1)];
                U new_cnt = newmap[get_map_idx(b, cell, 2)];
                if (new_cnt > 0) {
                    if (new_v / new_cnt > ${max_variance}) {
                        map[get_map_idx(b, cell, 0)] = 0;
                        map[get_map_idx(b, cell, 1)] = ${initial_variance};
                        map[get_map_idx(b, cell, 2)] = 0;
                    }
                    else {
                        map[get_map_idx(b, cell, 0)] = new_h / new_cnt;
                        map[get_map_idx(b, cell, 1)] = new_v / new_cnt;
                        map[get_map_idx(b, cell, 2)] = 1;
                    }
                }
                if (valid < 0.5) {
                    map[get_map_idx(b, cell, 0)] = 0;
                    map[get_map_idx(b, cell, 1)] = ${initial_variance};
                    map[get_map_idx(b, cell, 2)] = 0;
                }
                """
            ).substitute(n=n, max_variance=p.max_variance, initial_variance=p.initial_variance),
            name="average_map_kernel_batched",
        )

        # --- dilation + normal (셀 단위, env 경계를 넘지 않는 이웃 인덱싱) ---
        nb_pre = string.Template(
            """
            __device__ bool cell_inside(int cx, int cy) {
                if (cx <= 0 || cx >= ${n} - 1) { return false; }
                if (cy <= 0 || cy >= ${n} - 1) { return false; }
                return true;
            }
            __device__ int get_map_idx(int b, int cell, int layer_n) {
                return (b * 7 + layer_n) * ${n} * ${n} + cell;
            }
            """
        ).substitute(n=n)
        self._dilation = cp.ElementwiseKernel(
            in_params="raw U map, raw U mask",
            out_params="raw U newmap, raw U newmask",
            preamble=nb_pre,
            operation=string.Template(
                """
                int cell = i % (${n} * ${n});
                int base = i - cell;
                int cx = cell / ${n};
                int cy = cell % ${n};
                U h = map[i];
                U valid = mask[i];
                newmap[i] = h;
                if (valid < 0.5) {
                    U distance = 100;
                    U near_value = 0;
                    for (int dy = -${d}; dy <= ${d}; dy++) {
                        for (int dx = -${d}; dx <= ${d}; dx++) {
                            if (!cell_inside(cx + dx, cy + dy)) {continue;}
                            int nidx = base + cell + ${n} * dy + dx;
                            U nvalid = mask[nidx];
                            if(nvalid > 0.5 && dx + dy < distance) {
                                distance = dx + dy;
                                near_value = map[nidx];
                            }
                        }
                    }
                    if(distance < 100) {
                        newmap[i] = near_value;
                        newmask[i] = 1.0;
                    }
                }
                """
            ).substitute(n=n, d=p.dilation_size),
            name="dilation_filter_kernel_batched",
        )
        self._normal = cp.ElementwiseKernel(
            in_params="raw U dil, raw U map",
            out_params="raw U newmap",
            preamble=nb_pre,
            operation=string.Template(
                """
                int cell = i % (${n} * ${n});
                int b = i / (${n} * ${n});
                int base = i - cell;
                int cx = cell / ${n};
                int cy = cell % ${n};
                U h = dil[i];
                U valid = map[get_map_idx(b, cell, 2)];
                if (valid > 0.5) {
                    if (!cell_inside(cx + 1, cy) || !cell_inside(cx, cy + 1)) { return; }
                    float dzdx = (dil[base + cell + ${n}] - h);
                    float dzdy = (dil[base + cell + 1] - h);
                    float nx = -dzdy / ${res};
                    float ny = -dzdx / ${res};
                    float norm = sqrt((nx * nx) + (ny * ny) + 1);
                    newmap[(b * 3 + 0) * ${n} * ${n} + cell] = nx / norm;
                    newmap[(b * 3 + 1) * ${n} * ${n} + cell] = ny / norm;
                    newmap[(b * 3 + 2) * ${n} * ${n} + cell] = 1.0 / norm;
                }
                """
            ).substitute(n=n, res=p.resolution),
            name="normal_filter_kernel_batched",
        )

    # -- 인터페이스 (ElevationMapBackend 와 동일) --------------------------------

    def clear(self, env_ids) -> None:
        # stock ElevationMap.clear 와 동일: 전체 0 + variance 만 초기값 (layer3 도 0)
        ids = torch.as_tensor(list(env_ids), dtype=torch.long)
        m = torch.as_tensor(self.maps, device=self.device)
        m[ids] = 0.0
        m[ids, 1] = self.initial_variance

    @torch.no_grad()
    def _move_to(self, position: torch.Tensor) -> None:
        """stock move_to 의 배치판: 셀 스냅 roll(-delta_pixel) + z 시프트."""
        n = self.cell_n
        delta = position - self.centers
        delta_pixel = torch.round(delta[:, :2] / self.resolution)
        self.centers[:, :2] += delta_pixel * self.resolution
        self.centers[:, 2] += delta[:, 2]
        shift = (-delta_pixel).long()  # cp.roll(map, shift, axis=(1,2)) 와 동일 규약

        m = torch.as_tensor(self.maps, device=self.device)  # (B,7,n,n) zero-copy
        sx, sy = shift[:, 0], shift[:, 1]
        ar = torch.arange(n, device=self.device)
        # roll: new[i] = old[(i - s) mod n]
        ix = (ar.view(1, n) - sx.view(-1, 1)) % n            # (B,n)
        iy = (ar.view(1, n) - sy.view(-1, 1)) % n
        rolled = m.gather(2, ix.view(-1, 1, n, 1).expand(-1, 7, n, n))
        rolled = rolled.gather(3, iy.view(-1, 1, 1, n).expand(-1, 7, n, n))
        # pad_value: wrap 된 밴드 무효화 (전 레이어 0, variance 는 initial)
        vx = (ar.view(1, n) >= sx.clamp_min(0).view(-1, 1)) & (
            ar.view(1, n) < (n + sy.new_zeros(1) + sx.clamp_max(0).view(-1, 1)))
        vy = (ar.view(1, n) >= sy.clamp_min(0).view(-1, 1)) & (
            ar.view(1, n) < (n + sx.new_zeros(1) + sy.clamp_max(0).view(-1, 1)))
        keep = (vx.view(-1, 1, n, 1) & vy.view(-1, 1, 1, n))  # (B,1,n,n)
        rolled = torch.where(keep, rolled, torch.zeros_like(rolled))
        pad_var = (~keep).squeeze(1)
        rolled[:, 1][pad_var] = self.initial_variance
        # z 시프트: shift_map_z(-delta_z) == elevation/ub 에 -delta_z 를 더한다
        rolled[:, 0] -= delta[:, 2].view(-1, 1, 1)
        rolled[:, 5] -= delta[:, 2].view(-1, 1, 1)
        m.copy_(rolled)

    @torch.no_grad()
    def update(
        self,
        points_sensor: list[torch.Tensor],
        R_sensor: torch.Tensor,
        t_sensor: torch.Tensor,
        base_pos: torch.Tensor,
        R_base: torch.Tensor,
    ) -> None:
        cp = self.cp
        n, B = self.cell_n, self.num_envs
        self._move_to(base_pos)

        counts = [int(p.shape[0]) for p in points_sensor]
        n_tot = sum(counts)
        self.newmap *= 0.0
        if n_tot > 0:
            # add_points 커널이 p 를 in-place 로 덮어쓰므로 스테이징 복사본을 만든다
            pts = torch.cat([p for p in points_sensor if p.shape[0] > 0], dim=0).contiguous().clone()
            env_id = torch.repeat_interleave(
                torch.arange(B, device=self.device, dtype=torch.int32),
                torch.tensor(counts, device=self.device),
            ).contiguous()
            # stock 과 동일: t 는 map center 를 사전 차감, center 인자는 0
            t_k = (t_sensor - self.centers).contiguous()
            p_cp = cp.asarray(pts)
            id_cp = cp.asarray(env_id)
            R_cp = cp.asarray(R_sensor.contiguous().view(-1))
            t_cp = cp.asarray(t_k.view(-1))
            self._err *= 0.0
            self._err_cnt *= 0.0
            self._error_counting(
                self.maps, p_cp, self._zeros_b, self._zeros_b, R_cp, t_cp, id_cp,
                self.newmap, self._err, self._err_cnt, size=n_tot,
            )
            self._add_points(
                self._zeros_b, self._zeros_b, R_cp, t_cp, self.norm, id_cp,
                p_cp, self.maps, self.newmap, size=n_tot,
            )
        self._average_map(self.newmap, self.maps, size=B * n * n)

        # stock traversability 스텁의 부작용 재현: layer3 내부 = 0
        m = torch.as_tensor(self.maps, device=self.device)
        m[:, 3, 3:-3, 3:-3] = 0.0

        # dilation(upper_bound 기반) → normal (visibility cleanup 의 cos 판정용)
        ub = cp.ascontiguousarray(self.maps[:, 5])
        mask = cp.ascontiguousarray(self.maps[:, 2] + self.maps[:, 6])
        self.dil *= 0.0
        self.dil_mask *= 0.0
        self._dilation(ub, mask, self.dil, self.dil_mask, size=B * n * n)
        self.norm *= 0.0
        self._normal(self.dil, self.maps, self.norm, size=B * n * n)

        # update_variance / update_time
        m[:, 1] += self.time_variance * m[:, 2]
        m[:, 4] += self.time_interval

    def layers(self) -> torch.Tensor:
        return torch.as_tensor(self.maps, device=self.device)

    def centers_t(self) -> torch.Tensor:
        return self.centers

    def sample(self, points_xy: torch.Tensor, base_z: torch.Tensor):
        return sample_scan_heights(
            self.layers(), self.centers, points_xy, base_z, self.resolution, self.cell_n
        )
