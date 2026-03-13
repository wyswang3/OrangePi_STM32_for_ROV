#include "io/log/telemetry_timeline_logger.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string_view>
#include <ctime>

namespace rovctrl::io {

namespace {

std::string safe_cstr(const char* s)
{
    return (s != nullptr) ? std::string(s) : std::string{};
}

} // namespace

struct TelemetryTimelineLogger::Impl {
    std::ofstream timeline_ofs;
    std::ofstream event_ofs;
    std::uint64_t last_event_seq = 0;
    std::uint64_t last_command_stamp_ns = 0;

    static bool ensure_dir_(const std::filesystem::path& p)
    {
        std::error_code ec;
        if (std::filesystem::exists(p, ec)) return true;
        return std::filesystem::create_directories(p, ec);
    }

    static std::string make_timestamp_()
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    void write_headers_()
    {
        if (timeline_ofs.is_open()) {
            timeline_ofs
                << "telemetry_stamp_ns,telemetry_seq,active_mode,armed,estop_latched"
                << ",failsafe_active,controller_name,desired_controller"
                << ",nav_valid,nav_state,nav_health,nav_stale,nav_degraded,nav_age_ms"
                << ",health_state,degraded,fault_state,last_fault_code"
                << ",stm32_link_state,pwm_link_state,heartbeat_age_ms"
                << ",cmd_status,cmd_fault_code,event_seq,event_code,event_fault_code"
                << "\n";
            timeline_ofs.flush();
        }

        if (event_ofs.is_open()) {
            event_ofs
                << "kind,stamp_ns,seq_or_cmd,code,fault_code,arg0,arg1,intent_id,source,status"
                << "\n";
            event_ofs.flush();
        }
    }

    void log_timeline_row_(const shared::msg::TelemetryFrameV2& frame)
    {
        if (!timeline_ofs.is_open()) {
            return;
        }

        timeline_ofs
            << frame.stamp_ns
            << "," << frame.seq
            << "," << static_cast<unsigned>(frame.control.active_mode)
            << "," << static_cast<unsigned>(frame.control.armed)
            << "," << static_cast<unsigned>(frame.control.estop_latched)
            << "," << static_cast<unsigned>(frame.control.failsafe_active)
            << "," << safe_cstr(frame.control.controller_name)
            << "," << safe_cstr(frame.control.desired_controller)
            << "," << static_cast<unsigned>(frame.system.nav_valid)
            << "," << static_cast<unsigned>(frame.system.nav_state)
            << "," << static_cast<unsigned>(frame.system.nav_health)
            << "," << static_cast<unsigned>(frame.system.nav_stale)
            << "," << static_cast<unsigned>(frame.system.nav_degraded)
            << "," << frame.system.nav_age_ms
            << "," << static_cast<unsigned>(frame.system.health_state)
            << "," << static_cast<unsigned>(frame.system.degraded)
            << "," << static_cast<unsigned>(frame.system.fault_state)
            << "," << frame.system.last_fault_code
            << "," << static_cast<unsigned>(frame.system.stm32_link_state)
            << "," << static_cast<unsigned>(frame.system.pwm_link_state)
            << "," << frame.system.heartbeat_age_ms
            << "," << static_cast<unsigned>(frame.last_command_result.status)
            << "," << frame.last_command_result.fault_code
            << "," << frame.last_event.seq
            << "," << frame.last_event.event_code
            << "," << frame.last_event.fault_code
            << "\n";
    }

    void log_event_rows_(const shared::msg::TelemetryFrameV2& frame)
    {
        if (!event_ofs.is_open()) {
            return;
        }

        if (frame.last_event.seq != 0 && frame.last_event.seq != last_event_seq) {
            last_event_seq = frame.last_event.seq;
            event_ofs
                << "event"
                << "," << frame.last_event.stamp_ns
                << "," << frame.last_event.seq
                << "," << frame.last_event.event_code
                << "," << frame.last_event.fault_code
                << "," << frame.last_event.arg0
                << "," << frame.last_event.arg1
                << "," << 0
                << "," << 0
                << "," << 0
                << "\n";
        }

        if (frame.last_command_result.stamp_ns != 0 &&
            frame.last_command_result.stamp_ns != last_command_stamp_ns) {
            last_command_stamp_ns = frame.last_command_result.stamp_ns;
            event_ofs
                << "command_result"
                << "," << frame.last_command_result.stamp_ns
                << "," << frame.last_command_result.cmd_seq
                << "," << frame.last_command_result.event_code
                << "," << frame.last_command_result.fault_code
                << "," << 0
                << "," << 0
                << "," << frame.last_command_result.intent_id
                << "," << static_cast<unsigned>(frame.last_command_result.source)
                << "," << static_cast<unsigned>(frame.last_command_result.status)
                << "\n";
        }
    }
};

TelemetryTimelineLogger::TelemetryTimelineLogger()
    : impl_(std::make_unique<Impl>())
{
}

TelemetryTimelineLogger::~TelemetryTimelineLogger()
{
    close();
}

bool TelemetryTimelineLogger::init(const std::string& root_dir, const std::string& prefix)
{
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    close();

    const std::filesystem::path root(root_dir);
    if (!Impl::ensure_dir_(root)) {
        std::cerr << "[TelemetryTimelineLogger] ensure_dir failed: " << root << "\n";
        return false;
    }

    const std::string ts = Impl::make_timestamp_();
    const std::filesystem::path timeline_file = root / (prefix + "_timeline_" + ts + ".csv");
    const std::filesystem::path events_file = root / (prefix + "_events_" + ts + ".csv");

    impl_->timeline_ofs.open(timeline_file, std::ios::out | std::ios::trunc);
    impl_->event_ofs.open(events_file, std::ios::out | std::ios::trunc);
    if (!impl_->timeline_ofs.is_open() || !impl_->event_ofs.is_open()) {
        std::cerr << "[TelemetryTimelineLogger] open failed under: " << root << "\n";
        close();
        return false;
    }

    impl_->timeline_ofs.setf(std::ios::fixed);
    impl_->event_ofs.setf(std::ios::fixed);
    impl_->write_headers_();
    return true;
}

void TelemetryTimelineLogger::close() noexcept
{
    if (!impl_) {
        return;
    }
    if (impl_->timeline_ofs.is_open()) {
        impl_->timeline_ofs.flush();
        impl_->timeline_ofs.close();
    }
    if (impl_->event_ofs.is_open()) {
        impl_->event_ofs.flush();
        impl_->event_ofs.close();
    }
}

bool TelemetryTimelineLogger::is_open() const noexcept
{
    return impl_ && impl_->timeline_ofs.is_open() && impl_->event_ofs.is_open();
}

void TelemetryTimelineLogger::log_frame(const shared::msg::TelemetryFrameV2& frame)
{
    if (!is_open()) {
        return;
    }
    impl_->log_timeline_row_(frame);
    impl_->log_event_rows_(frame);
}

} // namespace rovctrl::io
