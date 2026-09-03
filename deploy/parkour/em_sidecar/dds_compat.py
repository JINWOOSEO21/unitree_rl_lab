"""unitree_sdk2_python 을 이 환경에서 쓰기 위한 호환 처리.

문제
----
`ChannelFactoryInitialize()` 가 CycloneDDS 에 넘기는 설정 XML 에 다음 블록이 있다:

    <Tracing>
        <Verbosity>config</Verbosity>
        <OutputFile>/tmp/cdds.LOG</OutputFile>
    </Tracing>

이 블록이 있으면 CycloneDDS 0.10.2 가 로그를 몇 KB 쓰다가
`*** buffer overflow detected *** : terminated` 로 **프로세스를 죽인다.**
(파이썬 예외가 아니라 C 레벨 abort 라 try/except 로도 못 잡는다.)

Tracing 블록만 빼면 정상 동작한다. 로그는 우리에게 필요 없으므로 그대로 뺀다.

버전 메모
--------
`/usr/local/lib/libddsc.so`(unitree_sdk2 동봉)가 **0.10.2** 라서 파이썬 바인딩도
0.10.2 여야 한다. pip 기본 wheel 은 11.0.1 이고, 그걸 쓰면 C++ 쪽과 **토픽이 아예
매칭되지 않는다**(메시지가 하나도 안 온다). 설치 방법:

    # CycloneDDS 0.10.2 를 사용자 prefix 에 빌드 (sudo 불필요)
    git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds
    cd cyclonedds && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=~/.local/cyclonedds -DBUILD_IDLC=ON \
             -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
    make -j8 && make install
    CYCLONEDDS_HOME=~/.local/cyclonedds pip install cyclonedds==0.10.2
    pip install --no-deps -e <unitree_sdk2_python>   # 구 cyclonedds 재빌드 방지
"""

_CONFIG_NO_TRACING = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""


def init_dds(domain_id: int = 0, interface: str = "lo") -> None:
    """Tracing 블록을 제거한 설정으로 unitree DDS 채널을 초기화한다."""
    import unitree_sdk2py.core.channel as ch
    import unitree_sdk2py.core.channel_config as cc

    cc.ChannelConfigHasInterface = _CONFIG_NO_TRACING
    ch.ChannelConfigHasInterface = _CONFIG_NO_TRACING
    ch.ChannelFactoryInitialize(domain_id, interface)
