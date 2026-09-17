# Go2 무선 연결 조사 — 내장 AP 로는 왜 안 되는가, 그러면 무엇을 쓰는가

2026-09-17. 단계 D 1차 시도가 실패한 뒤, "Go2 AP 의 목적 자체가 노트북 제어 아니냐" 는
물음에 답하기 위해 공식 문서·커뮤니티 사례·보안 연구를 조사한 결과다.
우리 실측은 `README.md` 의 "단계 D 1차 시도" 절에 있고, 여기서는 **왜 그런지**와
**그래서 무엇을 쓸지**를 다룬다.

표기: **[실측]** 우리가 직접 잰 것 · **[문헌]** 출처가 있는 것 · **[추정]** 근거는 있으나
확인 못 한 것.

---

## 0. 한 줄 답

**맞다. 다만 그 "제어" 는 우리가 하려는 제어가 아니다.**

Go2 내장 AP 의 목적은 **앱을 통한 고수준 제어(WebRTC)** 다. 그 경로는 `rt/lowcmd` 를
**설계상 지원하지 않는다** — 저수준 관절 제어는 아예 통로가 없다 **[문헌]**. 우리 구조는
`rt/lowcmd` 를 500 Hz 로 쏘는 DDS 저수준 제어이고, Unitree 공식 SDK 문서는 그 경로로
**Ethernet, PC 를 192.168.123.99/24** 만 기술한다 — Wi-Fi 언급이 없다 **[문헌]**.

무선으로 저수준 제어를 하는 사람들은 내장 AP 를 쓰지 않는다. **192.168.123.0/24 에
AP 를 붙여 노트북을 로봇과 같은 L2 브로드캐스트 도메인에 넣는다.**

---

## 1. Go2 의 네트워크는 두 겹이다

| 망 | 주소 | 사는 것 | 용도 |
|---|---|---|---|
| 내부 유선 | `192.168.123.0/24` | 제어보드(MCU) `.161`, Jetson/확장독 `.18`, 애드온 라우터 `.100` | **DDS**. `rt/lowstate`, `rt/lowcmd`, lidar |
| 내장 AP | `192.168.12.0/24` (게이트웨이 `.1`) | AP 클라이언트 | **앱/WebRTC** |
| 로봇 wlan0 (STA) | 외부 공유기가 주는 주소 | — | 앱/WebRTC, 인터넷 |

**[문헌]** 제어보드의 `actuator_manager` 가 `192.168.123.161` 에서 DDS domain 0 을 듣는다.
내부망은 외부에서 기본적으로 **격리**되어 있다 (boschko.ca).
**[문헌]** Unitree 공식 `unitree_ros2` 의 `setup.sh` 는 CycloneDDS 를 이렇게 잡는다:

```xml
<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>
```

그리고 PC 를 `192.168.123.99 / 255.255.255.0` 으로 두라고 한다. **우리가 유선에서 쓰던
바로 그 주소이고, 공식 경로를 그대로 따르고 있었다.**

---

## 2. 왜 우리 시도가 실패했나 — 벽이 세 개다

### 벽 1. 전송 계층 — 내장 AP 는 WebRTC 용이고, WebRTC 는 `rt/lowcmd` 를 안 나른다

**[문헌]** 로봇 안에 `webrtc_bridge` 가 있어 WebRTC ↔ DDS 를 변환한다. 앱은 WebRTC 를
쓴다. 그런데 그 변환은 부분집합이다:

> "low-level commands would not work fully as **`rt/lowcmd` is not supported**.
> Reading is only supported through **`rt/lf/lowstate`** (lf for low frequency)."

즉 내장 AP 로 붙으면 **쓰기는 저수준 명령이 없고, 읽기는 저주파 lowstate 뿐**이다.
설령 네트워크를 다 뚫어도 이 경로로는 우리 컨트롤러가 돌 수 없다.

**[문헌]** 덤으로: CycloneDDS 자체가 **EDU 에서만 기본 동작**하고 AIR/PRO 는 커스텀
펌웨어가 필요하다. 우리가 유선 DDS 로 저수준 제어를 하고 있다는 사실 자체가 이 로봇이
EDU 계열임을 뜻한다.

### 벽 2. 라우팅 — 게이트웨이 자신까지만 간다

**[실측]** `ip route add 192.168.123.0/24 via 192.168.12.1` 을 넣으면:

| 대상 | 결과 | 해석 |
|---|---|---|
| `192.168.123.161` 제어보드 | ping 3/3, 2.97 ms · TCP 22 `connection refused` | **도달**. 게이트웨이 자신일 가능성이 높다 (한 장비의 두 주소) |
| `192.168.123.18` Jetson | ICMP 100 % loss · TCP 22 무응답 | **도달 불가** |

Jetson 까지 가려면 (a) 게이트웨이가 내부망으로 **전달**하고 (b) Jetson 에
`192.168.12.0/24` **반환 경로**가 있어야 한다. 둘 중 무엇이 없는지는 **[추정]** 이다 —
가르려면 Jetson 에 들어가야 하는데 **AP 만으로는 Jetson 에 ssh 가 안 되므로 그 확인
자체가 불가능하다.** (§5 실험 1 이 이걸 푼다.)

**[문헌]** 보안 연구자들은 로봇을 **STA 모드**(외부 공유기에 붙어 `192.168.1.7`)로 두고
`sudo route add -net 192.168.123.0/24 192.168.1.7` 로 내부망에 들어갔다. **로봇이
라우터 역할을 하긴 한다는 증거**다. 다만 그들이 필요했던 건 `.161` 의 `actuator_manager`
하나뿐이라 `.18` 까지 가는지는 그 글로 알 수 없다.

### 벽 3. DDS discovery — 멀티캐스트는 라우터를 못 넘는다

**[실측]** `wlo1` 에서 `239.255.0.1:7400` 을 10 초 들어도 **discovery 패킷이 0 개**다.
`go2_state_probe wlo1` → LowState 0. `AllowMulticast false` + `Peers` 로 unicast 를
명시해도 0.

**[문헌]** 계획 D-3 의 경고 그대로이고, 같은 증상을 커뮤니티가 겪었다. MYBOTSHOP 포럼의
Go2 EDU 무선 스레드에서 사용자는 **"ROS2 토픽이 WLAN 으로 안 나가고 내부 서브넷
`192.168.123.X` 에 머문다"** 고 보고했고, 해법은 CycloneDDS XML 에 `wlan0` 인터페이스와
**명시적 Peer IP** 를 넣는 것이었다.

여기에 우리만의 추가 제약이 있다. peers 는 **양쪽 다** 설정해야 하는데,
**제어보드의 DDS 설정은 우리 것이 아니다.** 우리 브리지는 `eth0` 에 바인딩하고
(`ChannelFactoryInitialize(0, "eth0")`) locator 를 `192.168.123.18` 로 광고한다.
제어보드는 `192.168.123.161` 로 광고한다. 노트북이 다른 서브넷에 있으면 이 locator 들이
전부 노트북에서 **되돌아갈 수 없는 주소**가 된다.

---

## 3. 그래서 다른 사람들은 어떻게 하나 — 네 가지

### A. 192.168.123.0/24 에 AP/라우터를 붙인다 ← 커뮤니티 표준

**[문헌]** 로봇 등에 소형 트래블 라우터를 얹고, **Go2 payload 이더넷 포트 → 라우터 LAN
포트**로 물린다. 라우터가 AP 가 되어 `go2`(2.4 GHz) / `go2_5g`(5 GHz) 를 쏘고
게이트웨이는 **`192.168.123.1`**, Jetson 은 `192.168.123.18` 고정, 노트북은 Wi-Fi 로
붙어 **`192.168.123.222` 고정**을 받는다 (GO2_THDTCC / QRE 문서).
MYBOTSHOP 의 Go2 구성도 **라우터를 `192.168.123.100`** 에 둔다.

**[문헌]** Go2 는 payload 인터페이스에 RJ45 로 "Gigabit Ethernet: 1 channel to GO2,
**1 channel for external user**" 를 낸다 — 라우터를 물릴 포트가 있다.

**우리 구조 적합성: 최적.** 노트북이 로봇과 **같은 L2** 에 들어가므로
- 멀티캐스트 discovery 가 그냥 된다 → CycloneDDS peers 설정이 **필요 없다**
- `.161`(lowstate/lowcmd) 과 `.18`(scandots/gyro_bias) 둘 다 자연스럽게 닿는다
- **우리 코드는 한 줄도 안 바뀐다.** 노트북은 `--network wlo1`, Jetson 브리지는
  `--interface eth0` 그대로. 노트북 주소도 `192.168.123.99` 를 그대로 쓰면 된다.

### B. Jetson 에 USB Wi-Fi 동글 + 외부 WLAN(STA) + CycloneDDS peers

**[문헌]** MYBOTSHOP 포럼의 권장안. TP-Link TL-WN722N 같은 동글을 **Go2 USB 포트에 직접**
꽂고(허브 말고) `sudo nmtui` 로 랩 WLAN 에 붙인 뒤, CycloneDDS XML 에 `wlan0` +
명시적 Peer 를 넣는다.

**우리 구조 적합성: 부분적.** Jetson↔노트북은 풀리지만 **제어보드(`.161`)가 남는다.**
제어보드 DDS 설정을 우리가 못 바꾸므로 `lowcmd` 경로가 그대로 막힌다.
경고도 있다 — **"동글을 빼면 그 인터페이스에 바인딩된 노드가 전부 죽는다."**

### C. Jetson 자체를 핫스팟으로 (NetworkManager `shared`)

**[문헌]** DroneBlocks 방식. BrosTrend AC1L 을 Jetson 에 꽂고
`ipv4.method shared ipv4.addresses "10.42.0.1/24"` 로 핫스팟을 띄운다. 노트북은
`ssh unitree@10.42.0.1` 로 붙는다.

**우리 구조 적합성: 불가.** `shared` 는 **NAT** 다. NAT 는 DDS locator 를 깨뜨리고,
제어보드는 여전히 다른 서브넷이다. SSH·대시보드용으로는 훌륭하지만 저수준 제어용이 아니다.
(동글을 `eth0` 에 **브리지**하면 A 와 같아지지만, USB 동글 hostapd 브리지는 까다롭고
로봇 내부망에 DHCP 를 내보내는 위험이 있다.)

### D. Zenoh 브리지

**[문헌]** GO2_THDTCC 는 Jetson `ROS_DOMAIN_ID=0` / 노트북 `=1` 로 분리하고
`zenoh_bridge_ros2dds` 로 토픽을 넘긴다 — discovery 충돌을 피하려는 것이지 무선 자체의
해법은 아니다.

**우리 구조 적합성: 과하다.** 우리 스택은 ROS2 가 아니라 raw `unitree_sdk2` DDS 라
`zenoh-bridge-ros2dds` 가 바로 붙지 않고, 홉이 하나 늘어 지연·실패 지점이 는다.
그리고 **제어보드는 여전히 우리 설정 밖**이다.

---

## 4. 결론: A 안

내장 AP 는 우리 용도에 **구조적으로** 맞지 않는다 (§2 벽 1). 네트워크를 아무리 만져도
`rt/lowcmd` 가 그 경로로는 안 간다. 반면 A 안은

- 커뮤니티에서 검증된 표준 구성이고
- **우리 소프트웨어를 하나도 안 바꾸며**
- 유선에서 이미 전부 검증한 경로(같은 서브넷, 멀티캐스트 discovery)를 그대로 쓴다

필요한 것: **소형 AP/트래블 라우터 1 대**. 요구사항은

1. **AP(브리지) 모드** 지원 — NAT/라우터 모드가 아니라 L2 로 통과시켜야 한다
2. 자체 DHCP **끔** (로봇 내부망에 주소를 뿌리면 안 된다). 노트북은 `192.168.123.99` 고정
3. 5 GHz 지원 — 우리 실측에서 5.18 GHz 채널이 −37 dBm 로 깨끗했다
4. 로봇 payload 포트에서 급전 가능한 소형/경량 (GL.iNet 계열이 커뮤니티에서 흔하다)

---

## 5. 로봇이 있을 때 바로 할 검증 두 개

### 실험 1 — 내장 AP 가능성 최종 판정 (라우터 없이, 30 분)

§2 벽 2 의 **[추정]** 을 없앤다. 유선을 잠깐 되살려 Jetson 에 들어가서:

```bash
# Jetson 에서 — 노트북(AP 클라이언트)으로 가는 반환 경로
sudo ip route add 192.168.12.0/24 via 192.168.123.161 dev eth0
```

그 다음 노트북을 AP 로 돌리고 `ping 192.168.123.18`.

- **응답하면**: 게이트웨이는 전달하고 있었고 없던 건 반환 경로다. 그러면 CycloneDDS
  peers 로 Jetson↔노트북은 뚫린다. **다만 벽 1 때문에 `.161` 의 lowcmd 는 여전히
  막혀 있다** — 즉 실험이 성공해도 A 안은 필요하다. 진단 가치만 있다.
- **무응답이면**: 게이트웨이가 전달하지 않는다. 내장 AP 경로는 완전히 닫힌다.

어느 쪽이든 "내장 AP 로 될지도 모른다" 는 미련을 끝낸다.

### 실험 2 — A 안 구성 후, 유선과 같은 측정

라우터를 붙인 뒤 `README.md` 의 유선 기준선과 **같은 항목**을 잰다:

| 항목 | 유선 기준선 |
|---|---|
| scandots | 9.98 Hz, `bad=0`, age 85.7 ms, max_gap 150.5 ms |
| LowState (노트북) | 499.3 Hz |
| Jetson 브리지 | scan 초당 10 회 고정, fatal 0 |

도구는 그대로다: 노트북 `go2_parkour_rx_probe <nic> <초> 0`, Jetson
`tools/dds_rx_probe.py`. 계획 D-4 대로 **최소 수 분간** 돌리고, D-5 대로 각 로그에
wall+monotonic anchor 를 남긴다 (호스트 시계가 다르므로 LowState tick 을 공통 기준으로).

**그리고 Wi-Fi 절전은 반드시 먼저 끈다.** 우리 실측에서 절전이 켜져 있으면 RTT 가
3.6 → 117.5 ms 로 요동쳤고, 끄면 2.0 / 3.2 / 15.2 ms 가 됐다. 유선 scandots age 가
85 ms 였으니 절전 상태로는 200 ms 마감을 수시로 넘긴다.

```bash
sudo iw dev wlo1 set power_save off          # 재부팅하면 원복된다
```

---

## 6. 무선에서 달라지는 안전 조건

**우리 계획이 이미 적어 둔 것** (`migration_jetson_galaxybook_plan.md`):

> 36 행 — Ctrl+C=Stand → StandDown → 실제 down 안정 확인 → 종료.
> 프로세스 crash / 전원단절 / **무선단절**까지 보장하지 않는다.
>
> 95 행 (D-6) — 통신 단절 시 로봇 측 watchdog/동작을 문서와 코드로 확인한다. laptop 의
> Ctrl+C graceful shutdown 은 Wi-Fi 연결이 끊어지면 전달이 보장되지 않는다.
> **구동 중 의도적인 단절 실험은 이 단계에서 하지 않는다.**

**[문헌]** 로봇 측에 저수준 명령 watchdog 이 있어 명령 간격이 벌어지거나 관절이 안전
범위를 벗어나면 로봇을 정지시킨다(inria `unitree_control_interface`). **약 0.5 초**라는
언급이 있으나 1차 출처로 확증하지 못했다 — **[추정]**. 정확한 값과 정지 방식(댐핑인지
급정지인지)은 E 단계 전에 확인이 필요하다. 유선에서는 이 경로를 밟을 일이 거의 없지만
무선에서는 **일상적인 실패 모드**가 된다.

**[문헌]** 참고 수치 하나: Go1 Air 연구에서 이더넷 평균 지연 78.3 ms 대 Wi-Fi 평균
206.7 ms(표준편차 53 ms 미만)로 보고됐다. 미션 수준 경로라 우리 DDS 경로에 그대로
대입할 수는 없지만, 방향은 분명하다 — **무선은 느리고 흔들린다.** 우리 자신의 유선
기준선(age 85.7 ms, max_gap 150.5 ms)이 더 정확한 비교 대상이다.

---

## 7. 한 줄 정리

| 물음 | 답 |
|---|---|
| 내장 AP 의 목적은? | **앱을 통한 고수준(WebRTC) 제어.** 노트북 저수준 제어용이 아니다 |
| 왜 안 되나? | WebRTC 경로가 `rt/lowcmd` 를 안 나르고, AP 서브넷이 DDS 망과 갈려 있으며, 멀티캐스트가 라우터를 못 넘는다 |
| 네트워크만 잘 만지면 되나? | **아니다.** 벽 1 은 네트워크 문제가 아니라 설계다 |
| 그럼 무엇을 쓰나? | **192.168.123.0/24 에 AP(브리지)를 붙인다.** 커뮤니티 표준, 우리 코드 무변경 |
| 지금 당장은? | **유선 유지.** 이미 전부 검증됐고 정책도 유선에서 정상 동작했다 |

---

## 출처

- [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) — 공식 네트워크/CycloneDDS 설정 (Ethernet, 192.168.123.99)
- [legion1581/go2_python_sdk](https://github.com/legion1581/go2_python_sdk) — DDS vs WebRTC 기능 차이, `rt/lowcmd` 미지원, EDU 전용
- [legion1581/unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect) — AP / STA-L / STA-T 연결 방식
- [boschko.ca — Two RCEs in Unitree Robots](https://boschko.ca/unitree-go2-rce/) — 내부망 격리, `actuator_manager` @ 192.168.123.161, 라우팅·peers 우회
- [adhamaziz/GO2_THDTCC](https://github.com/adhamaziz/GO2_THDTCC) — 트래블 라우터 AP(게이트웨이 192.168.123.1), Zenoh 브리지
- [QRE DOCS — Unitree GO2](https://www.docs.quadruped.de/projects/go2/html/controller.html) — 라우터 192.168.123.100, AP 모드 vs Wi-Fi 모드, payload 이더넷
- [MYBOTSHOP — Go2 Manual](https://www.docs.mybotshop.de/go2.html) — 내부 IP 배치, 유선 개발 권장
- [MYBOTSHOP 포럼 — Go2 EDU Wireless Connection](https://forum.mybotshop.de/t/unitree-go2-edu-wireless-connection/1292) — "토픽이 내부 서브넷에 머문다" 와 CycloneDDS peers 해법
- [DroneBlocks/go2-wifi-adapter](https://github.com/DroneBlocks/go2-wifi-adapter) — Jetson USB 동글 핫스팟(10.42.0.1, NAT)
- [abizovnuralem/go2_ros2_sdk#203](https://github.com/abizovnuralem/go2_ros2_sdk/issues/203) — AP 모드는 WebRTC 문맥에서만 논의됨
- [inria-paris-robotics-lab/unitree_control_interface](https://github.com/inria-paris-robotics-lab/unitree_control_interface) — 저수준 명령 watchdog
- [Teddy-Liao/walk-these-ways-go2](https://github.com/Teddy-Liao/walk-these-ways-go2) — Jetson 192.168.123.18 / 제어보드 192.168.123.161, 유선 전제
