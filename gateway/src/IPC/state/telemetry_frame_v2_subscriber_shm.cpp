#include "gateway/IPC/state/telemetry_frame_v2_subscriber_shm.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace comm_gcs::ipc::state {

namespace {

constexpr int kMaxSeqLockRetries = 8;

inline bool check_header_contract(const shared::shm::TelemetryFrameV2ShmHeader& h) noexcept
{
    if (h.magic != shared::shm::kTelemetryFrameV2Magic) return false;
    if (h.layout_ver != shared::shm::kTelemetryFrameV2LayoutVersion) return false;
    if (h.payload_ver != shared::msg::kTelemetryFrameV2WireVersion) return false;
    if (h.payload_size != static_cast<std::uint32_t>(sizeof(shared::msg::TelemetryFrameV2))) return false;
    if (h.payload_align != static_cast<std::uint32_t>(alignof(shared::msg::TelemetryFrameV2))) return false;
    return true;
}

} // namespace

TelemetryFrameV2SubscriberShm::~TelemetryFrameV2SubscriberShm() noexcept
{
    shutdown();
}

TelemetryFrameV2SubscriberShm::TelemetryFrameV2SubscriberShm(TelemetryFrameV2SubscriberShm&& other) noexcept
{
    *this = std::move(other);
}

TelemetryFrameV2SubscriberShm&
TelemetryFrameV2SubscriberShm::operator=(TelemetryFrameV2SubscriberShm&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    enabled_ = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_ = other.error_flag_;
    cfg_ = other.cfg_;
    shm_name_ = std::move(other.shm_name_);
    shm_size_ = other.shm_size_;

#ifndef _WIN32
    shm_fd_ = other.shm_fd_;
    shm_ptr_ = other.shm_ptr_;
    other.shm_fd_ = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_ = other.shm_handle_;
    shm_ptr_ = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_ = nullptr;
#endif

    other.enabled_ = false;
    other.initialized_ = false;
    other.error_flag_ = false;
    other.shm_size_ = 0;
    return *this;
}

bool TelemetryFrameV2SubscriberShm::init(const Config& cfg)
{
    cfg_ = cfg;
    enabled_ = cfg.enable;

    if (!enabled_) {
        initialized_ = false;
        error_flag_ = false;
        return true;
    }

    if (cfg_.lazy_init) {
        initialized_ = false;
        error_flag_ = false;
        shm_name_ = cfg_.shm_name;
        return true;
    }

    return init_shm(cfg_);
}

void TelemetryFrameV2SubscriberShm::shutdown() noexcept
{
    close_shm();
    enabled_ = false;
    initialized_ = false;
    error_flag_ = false;
}

std::optional<shared::msg::TelemetryFrameV2>
TelemetryFrameV2SubscriberShm::poll(std::uint64_t* out_mono_ns,
                                    std::uint64_t* out_wall_ns)
{
    if (!enabled_) return std::nullopt;

    if (!initialized_) {
        if (!cfg_.lazy_init) return std::nullopt;
        if (!init_shm(cfg_)) {
            return std::nullopt;
        }
    }

    if (error_flag_ || !shm_ptr_) return std::nullopt;

    shared::msg::TelemetryFrameV2 frame{};
    std::uint64_t mono_ns = 0;
    std::uint64_t wall_ns = 0;
    if (!read_once(frame, &mono_ns, &wall_ns)) {
        return std::nullopt;
    }

    if (out_mono_ns) *out_mono_ns = mono_ns;
    if (out_wall_ns) *out_wall_ns = wall_ns;
    return frame;
}

bool TelemetryFrameV2SubscriberShm::read_once(shared::msg::TelemetryFrameV2& out,
                                              std::uint64_t* out_mono_ns,
                                              std::uint64_t* out_wall_ns) noexcept
{
    if (!shm_ptr_) return false;

    const auto* layout = static_cast<const ShmLayout*>(shm_ptr_);
    const auto& hdr = layout->hdr;

    for (int i = 0; i < kMaxSeqLockRetries; ++i) {
        const std::uint64_t s1 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 & 1u) continue;

        if (!check_header_contract(hdr)) {
            return false;
        }

        const std::uint64_t mono_ns = hdr.mono_ns;
        const std::uint64_t wall_ns = hdr.wall_ns;
        std::memcpy(&out, &layout->payload, sizeof(out));

        const std::uint64_t s2 = hdr.seqlock.load(std::memory_order_acquire);
        if (s1 == s2 && ((s2 & 1u) == 0)) {
            if (out_mono_ns) *out_mono_ns = mono_ns;
            if (out_wall_ns) *out_wall_ns = wall_ns;
            return true;
        }
    }

    return false;
}

bool TelemetryFrameV2SubscriberShm::init_shm(const Config& cfg)
{
#ifdef _WIN32
    std::cerr << "[TelemetryFrameV2SubscriberShm] Shared memory not supported on Windows.\n";
    initialized_ = false;
    error_flag_ = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rovctrl_telemetry_v2" : cfg.shm_name;
    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[TelemetryFrameV2SubscriberShm] Invalid shm_name: " << shm_name_ << "\n";
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ < 0) {
        if (cfg.lazy_init && errno == ENOENT) {
            initialized_ = false;
            error_flag_ = false;
            return false;
        }
        std::perror("[TelemetryFrameV2SubscriberShm] shm_open failed");
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    struct ::stat st {};
    if (::fstat(shm_fd_, &st) != 0) {
        std::perror("[TelemetryFrameV2SubscriberShm] fstat failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    const std::size_t real_size = static_cast<std::size_t>(st.st_size);
    if (real_size < min_size) {
        std::cerr << "[TelemetryFrameV2SubscriberShm] shm size too small: "
                  << real_size << " < " << min_size << "\n";
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    shm_size_ = real_size;
    void* addr = ::mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[TelemetryFrameV2SubscriberShm] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_ = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);
    initialized_ = true;
    error_flag_ = false;
    return true;
#endif
}

void TelemetryFrameV2SubscriberShm::close_shm() noexcept
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

} // namespace comm_gcs::ipc::state
