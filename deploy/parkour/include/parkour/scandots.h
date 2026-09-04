// scandots 132 수신 — EM 사이드카가 발행하는 rt/parkour/scandots 를 구독한다.
//
// 왜 별도 프로세스인가: elevation map 은 cupy 커널(학습과 **같은 코드**)로 돌기 때문에
// 배포 바이너리에 넣을 수 없다. 사이드카가 DDS 로 132 float 를 내고 여기서 받는다.
// 나중에 C++ 로 포팅해도(3-3 (b)) 이 소비자는 한 줄도 바뀌지 않는다.
//
// 메시지는 unitree_go::HeightMap_ 를 132 float 전송용으로 쓴다. 격자가 로봇 yaw 를
// 따라 도는 지역 격자라 origin+resolution 으로는 표현되지 않으므로, data 를 정책
// obs[53:185] 에 그대로 들어갈 132 벡터로 약속했다 (사이드카 sidecar.py 참조).
//
// **신선도가 안전 문제다.** 사이드카가 죽거나 밀리면 정책은 낡은 지형을 보고
// 계속 달린다. last_age() 로 나이를 재고, FSM 이 임계를 넘으면 Passive 로 떨어뜨린다.
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace parkour
{

class ScandotsSubscriber
{
public:
    static constexpr int kNumScan = 132;

    explicit ScandotsSubscriber(const std::string& topic = "rt/parkour/scandots")
        : values_(kNumScan, 0.0f)
    {
        sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>(topic));
        sub_->InitChannel([this](const void* msg) { this->on_msg(msg); }, 1);
    }

    /// 최신 scandots 사본. 아직 하나도 못 받았으면 전부 0 (= 평지 가정).
    std::vector<float> get() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return values_;
    }

    /// 마지막 수신으로부터 흐른 시간 [s]. 하나도 못 받았으면 매우 큰 값.
    double last_age() const
    {
        if (n_recv_.load() == 0) return 1e9;
        std::lock_guard<std::mutex> lk(mtx_);
        return std::chrono::duration<double>(Clock::now() - last_).count();
    }

    uint64_t count() const { return n_recv_.load(); }

    /// 길이가 132 가 아닌 메시지를 받은 횟수 (계약 위반 진단용).
    uint64_t bad_size_count() const { return n_bad_.load(); }

private:
    using Clock = std::chrono::steady_clock;

    void on_msg(const void* message)
    {
        const auto* m = static_cast<const unitree_go::msg::dds_::HeightMap_*>(message);
        const auto& d = m->data();
        if (static_cast<int>(d.size()) != kNumScan) {
            n_bad_.fetch_add(1);
            return;  // 조용히 무시하면 안 된다 — 카운터로 드러낸다
        }
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < kNumScan; ++i) values_[i] = d[i];
        last_ = Clock::now();
        n_recv_.fetch_add(1);
    }

    mutable std::mutex mtx_;
    std::vector<float> values_;
    Clock::time_point last_{};
    std::atomic<uint64_t> n_recv_{0};
    std::atomic<uint64_t> n_bad_{0};
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::HeightMap_> sub_;
};

}  // namespace parkour
