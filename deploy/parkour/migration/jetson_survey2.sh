#!/usr/bin/env bash
# Go2 Jetson 2차 조사: 1차에서 불명이던 항목만 확인한다.
# 읽기 전용: 설치/서비스 변경/sudo/DDS 송신 없음. 서비스 unit 과 스크립트는 "읽기"만 한다.
# 사용: cd ~/walking && bash jetson_survey2.sh  ->  ./go2_jetson_survey2_<timestamp>.txt
set -u
OUT="${SURVEY_OUT_DIR:-$PWD}/go2_jetson_survey2_$(date +%Y%m%d_%H%M%S).txt"

sec() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$ %s\n' "$*"; timeout "${TMO:-20}" "$@" 2>&1 | head -n "${MAXL:-40}"; }

{
sec "1. internet / clock (pip 가 HTTPS 를 쓸 수 있는가)"
run date -Is
run bash -c 'ping -c1 -W2 8.8.8.8 >/dev/null 2>&1; echo "ping 8.8.8.8 exit=$?"'
run bash -c 'getent hosts pypi.org || echo "DNS: pypi.org not resolvable"'
run bash -c 'curl -sS -m 6 -o /dev/null -w "https http_code=%{http_code}\n" https://pypi.org; echo "curl exit=$? (0=ok 6=dns 7=connect 28=timeout 60=cert/clock)"'
run bash -c 'curl -sSk -m 6 -o /dev/null -w "https(no cert check) http_code=%{http_code}\n" https://pypi.org; echo "curl -k exit=$?"'
run bash -c 'timedatectl show-timesync --all 2>/dev/null | grep -E "ServerName|ServerAddress|NTPMessage|SystemNTPServers|FallbackNTPServers" | cut -c1-160'
run bash -c 'nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>/dev/null'

sec "2. CycloneDDS C library (unitree_sdk2_python 은 cyclonedds==0.10.2 필요)"
run bash -c 'ls -la /usr/local/lib/libddsc* /usr/local/lib/cmake/CycloneDDS/ 2>/dev/null'
run bash -c 'grep -hE "PACKAGE_VERSION |set\(PACKAGE_VERSION" /usr/local/lib/cmake/CycloneDDS/CycloneDDSConfigVersion.cmake 2>/dev/null | head -3'
run bash -c 'cd ~/cyclonedds 2>/dev/null && git describe --tags --always && git branch --show-current && git log -1 --format="%h %ad %s" --date=short'
run bash -c 'ls ~/cyclonedds/install ~/cyclonedds/build 2>/dev/null | head -12'
run bash -c 'ls ~/cyclonedds_ws ~/cyclonedds_ws/src 2>/dev/null'
run bash -c 'find /usr /opt ~/cyclonedds ~/cyclonedds_ws -maxdepth 6 -name "libddsc.so*" 2>/dev/null | head -20'
run bash -c 'grep -n "CYCLONEDDS\|RMW_IMPLEMENTATION\|ROS_DOMAIN_ID\|source .*setup" ~/.bashrc'

sec "3. 기존 서비스가 로봇에 무엇을 보내는가 (LowCmd 경쟁 여부 확인용, unit 파일 읽기만)"
# 사용자 추가 서비스 = ExecStart 가 /home 아래를 가리키는 unit. 이름을 스크립트에 고정하지 않는다.
USER_UNITS=$(grep -lE '^ExecStart=.*/home/' /etc/systemd/system/*.service 2>/dev/null | xargs -r -n1 basename)
for u in $USER_UNITS; do
  MAXL=25 run systemctl cat "$u" --no-pager
  run systemctl show "$u" -p UnitFileState -p ActiveState -p SubState -p NRestarts
done
# 홈 아래 소스 중 LowCmd 를 다루는 파일 (파일 이름만, 내용은 출력하지 않음)
TMO=150 MAXL=40 run bash -c 'grep -rlE "rt/lowcmd|LowCmd_|lowcmd" --include=*.py --include=*.cpp --include=*.hpp --include=*.h --include=*.yaml ~ 2>/dev/null | grep -vE "/(\.cache|\.venv|site-packages|build|install|log|node_modules|walking|example|examples|unitree_sdk2|unitree_sdk2_python|\.claude)/" | head -40'
# 재시작을 반복하는 서비스
for u in $USER_UNITS; do
  n=$(systemctl show "$u" -p NRestarts --value 2>/dev/null)
  [ "${n:-0}" -gt 3 ] 2>/dev/null && MAXL=12 run bash -c "journalctl -u $u -n 10 --no-pager 2>&1 | cut -c1-200"
done

sec "4. python env tooling"
run bash -c '~/.local/bin/uv --version; ~/.local/bin/uv python list 2>/dev/null | head -12'
run bash -c 'python3.9 --version; python3.9 -c "import venv, ensurepip; print(\"py3.9 venv ok\")"'
run bash -c 'dpkg -l | grep -E "libopenblas|libopenmpi|libomp|python3.8-dev|python3-dev |libpython3.8-dev" | awk "{print \$2, \$3}"'
run bash -c 'ls ~/walking 2>/dev/null'

sec "5. GPU load baseline (5 samples, 1s)"
MAXL=6 run bash -c 'timeout 6 tegrastats --interval 1000 | cut -c1-220'

sec "6. docker (대안 경로 참고용)"
run bash -c 'docker info 2>/dev/null | grep -iE "runtimes|default runtime"'
run bash -c 'docker images --format "{{.Repository}}:{{.Tag}} {{.Size}}" 2>/dev/null | head -12'
} > "$OUT" 2>&1

echo "saved: $OUT"
echo "전달 전 민감한 값(토큰 등)이 없는지 한 번 훑어본 뒤 파일을 전달해 주세요."
