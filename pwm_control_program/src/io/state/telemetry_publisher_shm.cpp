#include "io/state/telemetry_publisher_shm.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rovctrl::io::state {

namespace {

inline std::uint64_t now_mono_ns() noexcept
{
    using clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            clock::now().time_since_epoch()).count());
}

inline std::uint64_t now_wall_ns() noexcept
{
    using clock = std::chrono::system_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            clock::now().time_since_epoch()).count());
}

} // namespace

TelemetryPublisherShm::~TelemetryPublisherShm() noexcept
{
    shutdown();
}

TelemetryPublisherShm::TelemetryPublisherShm(TelemetryPublisherShm&& other) noexcept
{
    *this = std::move(other);
}

TelemetryPublisherShm& TelemetryPublisherShm::operator=(TelemetryPublisherShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    enabled_ = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_ = other.error_flag_;
    shm_name_ = std::move(other.shm_name_);

#ifndef _WIN32
    shm_ptr_ = other.shm_ptr_;
    shm_size_ = other.shm_size_;
    shm_fd_ = other.shm_fd_;
    other.shm_ptr_ = nullptr;
    other.shm_size_ = 0;
    other.shm_fd_ = -1;
#else
    shm_handle_ = other.shm_handle_;
    shm_ptr_ = other.shm_ptr_;
    shm_size_ = other.shm_size_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_ = nullptr;
    other.shm_size_ = 0;
#endif

    other.enabled_ = false;
    other.initialized_ = false;
    other.error_flag_ = false;
    return *this;
}

bool TelemetryPublisherShm::init(const Config& cfg)
{
    shutdown();

    enabled_ = cfg.enable;
    if (!enabled_) {
        return true;
    }
    return init_shm(cfg);
}

void TelemetryPublisherShm::shutdown() noexcept
{
    close_shm();
    enabled_ = false;
    initialized_ = false;
    error_flag_ = false;
    shm_name_.clear();
}

bool TelemetryPublisherShm::publish(const shared::msg::TelemetryFrameV2& frame)
{
    if (!enabled_) return true;
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    auto& hdr = layout->hdr;

    const std::uint64_t seq0 = hdr.seqlock.load(std::memory_order_relaxed);
    hdr.seqlock.store(seq0 + 1, std::memory_order_release);
    hdr.mono_ns = now_mono_ns();
    hdr.wall_ns = now_wall_ns();
    layout->payload = frame;
    hdr.seqlock.store(seq0 + 2, std::memory_order_release);
    return true;
}

bool TelemetryPublisherShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[TelemetryPublisherShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_ = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_telemetry_v2" : cfg.shm_name;
    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[TelemetryPublisherShm] Invalid shm_name: " << shm_name_ << "\n";
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size) ? min_size : cfg.shm_size;

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[TelemetryPublisherShm] shm_open failed");
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[TelemetryPublisherShm] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    void* addr = ::mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[TelemetryPublisherShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);
    auto* layout = static_cast<ShmLayout*>(shm_ptr_);
    const bool mismatch =
        (layout->hdr.magic != shared::shm::kTelemetryFrameV2Magic) ||
        (layout->hdr.layout_ver != shared::shm::kTelemetryFrameV2LayoutVersion) ||
        (layout->hdr.payload_ver != shared::msg::kTelemetryFrameV2WireVersion) ||
        (layout->hdr.payload_size != sizeof(shared::msg::TelemetryFrameV2)) ||
        (layout->hdr.payload_align != alignof(shared::msg::TelemetryFrameV2));

    if (mismatch) {
        layout->hdr.seqlock.store(1, std::memory_order_relaxed);
        layout->payload = shared::msg::TelemetryFrameV2{};
        layout->hdr.magic = shared::shm::kTelemetryFrameV2Magic;
        layout->hdr.layout_ver = shared::shm::kTelemetryFrameV2LayoutVersion;
        layout->hdr.payload_ver = shared::msg::kTelemetryFrameV2WireVersion;
        layout->hdr.payload_size = sizeof(shared::msg::TelemetryFrameV2);
        layout->hdr.payload_align = alignof(shared::msg::TelemetryFrameV2);
        layout->hdr.mono_ns = now_mono_ns();
        layout->hdr.wall_ns = now_wall_ns();
        std::atomic_thread_fence(std::memory_order_release);
        layout->hdr.seqlock.store(2, std::memory_order_release);
    }

    initialized_ = true;
    error_flag_ = false;
    return true;
#endif
}

void TelemetryPublisherShm::close_shm() noexcept
{
#ifndef _WIN32
    if (shm_ptr_) {
        ::munmap(shm_ptr_, shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    shm_size_ = 0;
#else
    shm_ptr_ = nullptr;
    shm_handle_ = nullptr;
    shm_size_ = 0;
#endif
}

} // namespace rovctrl::io::state
