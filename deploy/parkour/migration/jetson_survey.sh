#!/usr/bin/env bash
# Go2 Jetson 환경 조사 (단계 B-1). 읽기 전용: 설치/서비스 변경/sudo/DDS 송신 없음.
# 사용: bash jetson_survey.sh   ->  ~/go2_jetson_survey_<host>_<timestamp>.txt 생성
set -u
OUT="${SURVEY_OUT_DIR:-$HOME}/go2_jetson_survey_$(hostname)_$(date +%Y%m%d_%H%M%S).txt"

sec() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$ %s\n' "$*"; timeout 15 "$@" 2>&1 | head -n "${MAXL:-60}"; }

{
sec "identity / time anchor"
run hostname; run uname -a; run date -Is
python3 -c 'import time; print("wall_ns", time.time_ns(), "monotonic_ns", time.monotonic_ns())'
run timedatectl
command -v chronyc >/dev/null && run chronyc tracking

sec "OS / L4T / JetPack"
run cat /etc/nv_tegra_release
run lsb_release -ds
MAXL=20 run bash -c "dpkg -l | grep -E 'nvidia-jetpack|nvidia-l4t-core|nvidia-l4t-cuda|cuda-toolkit|libcudnn8 |tensorrt ' | awk '{print \$2, \$3}'"

sec "CUDA toolkit"
run bash -c 'ls -d /usr/local/cuda* 2>/dev/null'
run bash -c 'command -v nvcc || ls /usr/local/cuda/bin/nvcc'
run bash -c '/usr/local/cuda/bin/nvcc --version 2>/dev/null | tail -2'
run bash -c 'cat /usr/local/cuda/version.json 2>/dev/null | head -8'

sec "power / thermal"
run bash -c 'nvpmodel -q 2>&1 | head -4'
run bash -c 'timeout 2 tegrastats 2>&1 | head -1'

sec "CPU / RAM / disk"
run nproc; run free -h; run df -h / /home

sec "python interpreters / env managers"
run bash -c 'ls /usr/bin/python3* 2>/dev/null'
run python3 --version
run bash -c 'command -v conda mamba micromamba pyenv virtualenv 2>/dev/null; ls -d ~/miniconda3 ~/miniforge3 ~/archiconda3 ~/.pyenv 2>/dev/null'
run bash -c 'python3 -m venv --help >/dev/null 2>&1 && echo "venv module: available" || echo "venv module: MISSING (python3-venv)"'
run python3 -m pip --version

sec "existing GPU / DDS python packages (system python3)"
MAXL=40 run bash -c "python3 -m pip list 2>/dev/null | grep -iE '^(torch|torchvision|cupy|numpy|scipy|cyclonedds|unitree|pyyaml|ruamel|simple.parsing|shapely|opencv|onnx|numba|pip|setuptools|wheel) '"
python3 - <<'EOF' 2>&1
for m in ("torch", "cupy", "cyclonedds", "unitree_sdk2py", "numpy"):
    try:
        mod = __import__(m)
        print(m, getattr(mod, "__version__", "?"), getattr(mod, "__file__", ""))
    except Exception as e:
        print(m, "NOT IMPORTABLE:", type(e).__name__, e)
try:
    import torch
    print("torch.cuda.is_available", torch.cuda.is_available(), "torch.version.cuda", torch.version.cuda)
except Exception:
    pass
EOF

sec "build tools"
run bash -c 'gcc --version | head -1; g++ --version | head -1; cmake --version | head -1; git --version'
run bash -c 'ls -d /usr/local/lib/libddsc* /usr/local/lib/libunitree* /opt/unitree* ~/cyclonedds* 2>/dev/null'
run bash -c 'echo CYCLONEDDS_HOME=${CYCLONEDDS_HOME:-}; echo CYCLONEDDS_URI=${CYCLONEDDS_URI:-}'

sec "network"
run ip -br addr
run ip route
run bash -c 'curl -sS -m 5 -o /dev/null -w "pypi https: http_code=%{http_code}\n" https://pypi.org; echo "curl exit=$? (0=ok 6=dns 7=connect 28=timeout 60=cert/clock)"'

sec "autostart services / containers / cron (기존 로봇 서비스 확인용)"
MAXL=80 run systemctl list-units --type=service --state=running --no-pager --no-legend
MAXL=40 run systemctl --user list-units --type=service --state=running --no-pager --no-legend
run bash -c 'docker ps 2>&1 | head -10'
run bash -c 'crontab -l 2>&1 | head -20'
run bash -c 'ls /etc/systemd/system/*.service 2>/dev/null | head -30'

sec "processes that may use DDS / lidar / GPU"
MAXL=40 run bash -c "ps -eo pid,user,pcpu,pmem,etime,args --sort=-pcpu | grep -iE 'unitree|dds|ros|lidar|python|sport|lio|slam' | grep -v grep | cut -c1-200"

sec "home directory layout"
run bash -c 'ls -la ~ | head -40'
} > "$OUT" 2>&1

echo "saved: $OUT"
echo "전달 전 프로세스 인자/홈 목록에 민감한 값이 없는지 한 번 훑어본 뒤 파일을 전달해 주세요."
