#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "shared/msg/telemetry_frame_v2.hpp"
#include "shared/shm/telemetry_frame_v2_shm.hpp"

namespace comm_gcs::ipc::state {

class TelemetryFrameV2SubscriberShm final {
public:
    struct Config {
        bool        enable    = true;
        std::string shm_name  = "/rovctrl_telemetry_v2";
        std::size_t shm_size  = 0;
        bool        lazy_init = true;
    };

    TelemetryFrameV2SubscriberShm() = default;
    ~TelemetryFrameV2SubscriberShm() noexcept;

    TelemetryFrameV2SubscriberShm(const TelemetryFrameV2SubscriberShm&) = delete;
    TelemetryFrameV2SubscriberShm& operator=(const TelemetryFrameV2SubscriberShm&) = delete;

    TelemetryFrameV2SubscriberShm(TelemetryFrameV2SubscriberShm&& other) noexcept;
    TelemetryFrameV2SubscriberShm& operator=(TelemetryFrameV2SubscriberShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    std::optional<shared::msg::TelemetryFrameV2> poll(std::uint64_t* out_mono_ns = nullptr,
                                                      std::uint64_t* out_wall_ns = nullptr);

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;
    bool read_once(shared::msg::TelemetryFrameV2& out,
                   std::uint64_t* out_mono_ns,
                   std::uint64_t* out_wall_ns) noexcept;

private:
    bool enabled_     = false;
    bool initialized_ = false;
    bool error_flag_  = false;
    Config cfg_{};

    std::string shm_name_;
    std::size_t shm_size_ = 0;

#ifndef _WIN32
    using ShmLayout = shared::shm::TelemetryFrameV2ShmLayout;
    int shm_fd_ = -1;
    ShmLayout* shm_ptr_ = nullptr;
#else
    void* shm_handle_ = nullptr;
    void* shm_ptr_ = nullptr;
#endif
};

} // namespace comm_gcs::ipc::state
