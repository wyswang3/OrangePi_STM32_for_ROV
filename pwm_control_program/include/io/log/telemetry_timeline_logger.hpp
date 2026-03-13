#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "shared/msg/telemetry_frame_v2.hpp"

namespace rovctrl::io {

/**
 * @brief Persist authoritative telemetry frames into timeline/event CSV files.
 *
 * Usage:
 *  - Called from the control main thread after TelemetryFrameV2 has been fully built.
 *  - `timeline` records every published control/telemetry snapshot.
 *  - `events` records only new `last_event` and `last_command_result` entries.
 *
 * Thread boundary:
 *  - No internal locking; intended for single-threaded control loop usage.
 */
class TelemetryTimelineLogger {
public:
    TelemetryTimelineLogger();
    ~TelemetryTimelineLogger();

    bool init(const std::string& root_dir, const std::string& prefix);
    void close() noexcept;
    bool is_open() const noexcept;

    /**
     * @brief Append one telemetry frame to the timeline and emit deduplicated event rows.
     *
     * Failure semantics:
     *  - Logging failure is non-fatal to control execution; callers should keep running.
     */
    void log_frame(const shared::msg::TelemetryFrameV2& frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rovctrl::io
