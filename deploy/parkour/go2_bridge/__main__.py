"""``python -m go2_bridge`` 진입점. 경로로 직접 실행해도 되도록 parkour 루트를 sys.path 에 넣는다."""

import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_bridge.sensor_bridge import main  # noqa: E402

if __name__ == "__main__":
    main()
