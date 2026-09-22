"""Go2 실기 센서 브리지 — 런타임 패키지.

진입점: ``python -m go2_bridge`` (deploy/parkour 에서). 점군·LowState·odometry 를 받아
정책 입력 scandots 를 만든다. 이 패키지와 em_sidecar 는 eval/ 을 import 하지 않는다
(tests/test_layout.py 가 강제).
"""
