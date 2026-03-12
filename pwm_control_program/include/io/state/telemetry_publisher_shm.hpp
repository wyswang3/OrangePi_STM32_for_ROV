#pragma once
#ifndef ROVCTRL_IO_STATE_TELEMETRY_PUBLISHER_SHM_HPP
#define ROVCTRL_IO_STATE_TELEMETRY_PUBLISHER_SHM_HPP

#include <cstddef>
#include <string>

#include "shared/msg/telemetry_frame_v2.hpp"
#include "shared/shm/telemetry_frame_v2_shm.hpp"

namespace rovctrl::io::state {

class TelemetryPublisherShm final {
public:
    struct Config {
        bool        enable   = true;
        std::string shm_name = "/rovctrl_telemetry_v2";
        std::size_t shm_size = 0;
    };

    TelemetryPublisherShm() = default;
    ~TelemetryPublisherShm() noexcept;

    TelemetryPublisherShm(const TelemetryPublisherShm&) = delete;
    TelemetryPublisherShm& operator=(const TelemetryPublisherShm&) = delete;

    TelemetryPublisherShm(TelemetryPublisherShm&& other) noexcept;
    TelemetryPublisherShm& operator=(TelemetryPublisherShm&& other) noexcept;

    bool init(const Config& cfg);
    void shutdown() noexcept;

    bool enabled() const noexcept { return enabled_; }
    bool initialized() const noexcept { return initialized_; }

    bool publish(const shared::msg::TelemetryFrameV2& frame);

private:
    bool init_shm(const Config& cfg);
    void close_shm() noexcept;

private:
    bool        enabled_     = false;
    bool        initialized_ = false;
    bool        error_flag_  = false;
    std::string shm_name_;

#ifndef _WIN32
    using ShmLayout = shared::shm::TelemetryFrameV2ShmLayout;
    ShmLayout*   shm_ptr_  = nullptr;
    std::size_t  shm_size_ = 0;
    int          shm_fd_   = -1;
#else
    void*        shm_handle_ = nullptr;
    void*        shm_ptr_    = nullptr;
    std::size_t  shm_size_   = 0;
#endif
};

} // namespace rovctrl::io::state

#endif // ROVCTRL_IO_STATE_TELEMETRY_PUBLISHER_SHM_HPP
