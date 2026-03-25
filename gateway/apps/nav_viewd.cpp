// gateway/apps/nav_viewd.cpp
//
// nav_viewd: NavState(SHM) -> NavStateView(SHM) bridge daemon
//
// Pipeline:
//   NavStateSubscriberShm.poll() -> NavViewBuilder.build() -> NavViewPublisherShm.publish()
//
// Responsibilities:
//   - Builder: preserve explicit valid/stale/degraded/fault semantics from NavState
//   - Daemon : define the current hop's age/stale policy and publish a control-facing snapshot
//              that never silently reuses old kinematics as "current valid nav".
//
// Defaults:
//   NavState shm: /rov_nav_state_v1
//   NavView  shm: /rovctrl_nav_view_v1

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <algorithm>
#include <unistd.h>

#include "gateway/IPC/nav/nav_state_subscriber_shm.hpp"
#include "gateway/IPC/nav/nav_view_builder.hpp"
#include "gateway/IPC/nav/nav_view_policy.hpp"
#include "gateway/IPC/nav/nav_view_publisher_shm.hpp"

#include "shared/msg/nav_state.hpp"
#include "shared/msg/nav_state_view.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;

std::atomic_bool g_stop{false};
void on_sigint(int) { g_stop.store(true); }

inline std::uint64_t now_mono_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch())
            .count());
}

std::string csv_escape(const std::string& value)
{
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (const char ch : value) {
        if (ch == '"') {
            out.push_back('"');
        }
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

std::string wall_time_now_string()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string resolve_run_id(const char* process_name)
{
    if (const char* env = std::getenv("ROV_RUN_ID"); env != nullptr && env[0] != '\0') {
        return env;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << process_name << "-" << ::getpid() << "-" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

struct NavViewEventCsvLogger {
    bool init(const std::string& path)
    {
        if (path.empty()) {
            return false;
        }

        std::error_code ec;
        const std::filesystem::path log_path(path);
        const auto dir = log_path.parent_path();
        if (!dir.empty() && !std::filesystem::exists(dir, ec) &&
            !std::filesystem::create_directories(dir, ec)) {
            return false;
        }

        const bool need_header = !std::filesystem::exists(log_path, ec) ||
                                 std::filesystem::file_size(log_path, ec) == 0;
        ofs_.open(log_path, std::ios::out | std::ios::app);
        if (!ofs_.is_open()) {
            return false;
        }

        run_id_ = resolve_run_id("nav_viewd");
        if (need_header) {
            ofs_
                << "mono_ns,wall_time,component,event,level,run_id,process_name,pid"
                << ",fault_code,nav_valid,nav_stale,nav_degraded,message,age_ms_from_nav_pub"
                << ",publish,diagnostic_only,degraded_publish,no_nav_yet,stale_triggered"
                << ",source_valid,source_stale,source_degraded,source_fault_code\n";
            ofs_.flush();
        }
        return true;
    }

    void log_event(std::uint64_t mono_ns,
                   const char*   event,
                   const char*   level,
                   std::uint16_t fault_code,
                   std::uint8_t  nav_valid,
                   std::uint8_t  nav_stale,
                   std::uint8_t  nav_degraded,
                   std::uint64_t age_ms_from_nav_pub,
                   bool          publish,
                   bool          diagnostic_only,
                   bool          degraded_publish,
                   bool          no_nav_yet,
                   bool          stale_triggered,
                   std::uint8_t  source_valid,
                   std::uint8_t  source_stale,
                   std::uint8_t  source_degraded,
                   std::uint16_t source_fault_code,
                   const std::string& message)
    {
        if (!ofs_.is_open()) {
            return;
        }

        ofs_
            << mono_ns
            << "," << csv_escape(wall_time_now_string())
            << "," << csv_escape("nav_viewd")
            << "," << csv_escape(event != nullptr ? event : "")
            << "," << csv_escape(level != nullptr ? level : "")
            << "," << csv_escape(run_id_)
            << "," << csv_escape("nav_viewd")
            << "," << ::getpid()
            << "," << fault_code
            << "," << static_cast<unsigned>(nav_valid)
            << "," << static_cast<unsigned>(nav_stale)
            << "," << static_cast<unsigned>(nav_degraded)
            << "," << csv_escape(message)
            << "," << age_ms_from_nav_pub
            << "," << (publish ? 1 : 0)
            << "," << (diagnostic_only ? 1 : 0)
            << "," << (degraded_publish ? 1 : 0)
            << "," << (no_nav_yet ? 1 : 0)
            << "," << (stale_triggered ? 1 : 0)
            << "," << static_cast<unsigned>(source_valid)
            << "," << static_cast<unsigned>(source_stale)
            << "," << static_cast<unsigned>(source_degraded)
            << "," << source_fault_code
            << "\n";
        ofs_.flush();
    }

private:
    std::ofstream ofs_;
    std::string run_id_;
};

struct DecisionSnapshot {
    bool publish = false;
    bool diagnostic_only = false;
    bool degraded_publish = false;
    bool no_nav_yet = false;
    bool stale_triggered = false;
    std::uint8_t nav_valid = 0;
    std::uint8_t nav_stale = 0;
    std::uint8_t nav_degraded = 0;
    std::uint16_t fault_code = 0;
    std::uint8_t source_valid = 0;
    std::uint8_t source_stale = 0;
    std::uint8_t source_degraded = 0;
    std::uint16_t source_fault_code = 0;
    std::uint64_t age_ms_from_nav_pub = 0;

    bool operator==(const DecisionSnapshot& rhs) const noexcept
    {
        return publish == rhs.publish &&
               diagnostic_only == rhs.diagnostic_only &&
               degraded_publish == rhs.degraded_publish &&
               no_nav_yet == rhs.no_nav_yet &&
               stale_triggered == rhs.stale_triggered &&
               nav_valid == rhs.nav_valid &&
               nav_stale == rhs.nav_stale &&
               nav_degraded == rhs.nav_degraded &&
               fault_code == rhs.fault_code &&
               source_valid == rhs.source_valid &&
               source_stale == rhs.source_stale &&
               source_degraded == rhs.source_degraded &&
               source_fault_code == rhs.source_fault_code &&
               age_ms_from_nav_pub == rhs.age_ms_from_nav_pub;
    }
};

std::string decision_reason(const DecisionSnapshot& snap)
{
    if (snap.no_nav_yet) {
        return "no_nav_yet";
    }
    if (snap.stale_triggered && !snap.publish) {
        return "stale_diagnostic_publish_suppressed";
    }
    if (snap.stale_triggered && snap.diagnostic_only) {
        return "stale_diagnostic_only_publish";
    }
    if (snap.nav_valid != 0 && snap.nav_stale == 0) {
        return "pass_through_valid";
    }
    return "pass_through_invalid";
}

struct Args {
    // SHM names
    std::string nav_state_shm = "/rov_nav_state_v1";
    std::string nav_view_shm  = "/rovctrl_nav_view_v1";

    // Rates
    double poll_hz = 50.0;   // poll at 50Hz
    double pub_hz  = 20.0;   // publish at 20Hz (0 disables publish)

    // Health / staleness
    std::uint32_t max_age_ms = 250;   // stale if (now - last_nav_pub_mono_ns) > max_age_ms (after warmup)
    std::uint32_t warmup_ms  = 1500;  // startup grace period

    // Degrade policy
    bool publish_when_stale = true;   // still publish explicit invalid diagnostics when stale/no-data
    bool hold_last_good     = false;  // deprecated compatibility knob; control-facing payload never reuses kinematics
    double degrade_pub_hz   = 5.0;    // degrade publish throttle (0 => publish every pub slot)

    // Logging
    double print_hz = 2.0;
    std::string event_log_path = "./logs/nav/nav_events.csv";
};

static void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --nav-state-shm <name>     default: /rov_nav_state_v1\n"
        << "  --nav-view-shm  <name>     default: /rovctrl_nav_view_v1\n"
        << "  --poll-hz <hz>             default: 50\n"
        << "  --pub-hz  <hz>             default: 20 (0 disables publish)\n"
        << "  --max-age-ms <ms>          default: 250\n"
        << "  --warmup-ms <ms>           default: 1500\n"
        << "  --publish-when-stale 0|1   default: 1\n"
        << "  --hold-last-good     0|1   default: 0 (deprecated)\n"
        << "  --degrade-pub-hz <hz>      default: 5 (0 disables degrade throttle)\n"
        << "  --print-hz <hz>            default: 2\n"
        << "  --event-log <path>         default: ./logs/nav/nav_events.csv\n";
}

static bool parse_bool(const std::string& s, bool& out)
{
    if (s == "1" || s == "true" || s == "TRUE")  { out = true;  return true; }
    if (s == "0" || s == "false" || s == "FALSE"){ out = false; return true; }
    return false;
}

static bool parse_args(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];

        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[ERR] missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (k == "--help" || k == "-h") {
            usage(argv[0]);
            return false;
        } else if (k == "--nav-state-shm") {
            const char* v = need("--nav-state-shm"); if (!v) return false;
            a.nav_state_shm = v;
        } else if (k == "--nav-view-shm") {
            const char* v = need("--nav-view-shm"); if (!v) return false;
            a.nav_view_shm = v;
        } else if (k == "--poll-hz") {
            const char* v = need("--poll-hz"); if (!v) return false;
            a.poll_hz = std::atof(v);
        } else if (k == "--pub-hz") {
            const char* v = need("--pub-hz"); if (!v) return false;
            a.pub_hz = std::atof(v);
        } else if (k == "--max-age-ms") {
            const char* v = need("--max-age-ms"); if (!v) return false;
            a.max_age_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--warmup-ms") {
            const char* v = need("--warmup-ms"); if (!v) return false;
            a.warmup_ms = static_cast<std::uint32_t>(std::strtoul(v, nullptr, 10));
        } else if (k == "--publish-when-stale") {
            const char* v = need("--publish-when-stale"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.publish_when_stale = b;
        } else if (k == "--hold-last-good") {
            const char* v = need("--hold-last-good"); if (!v) return false;
            bool b=false; if (!parse_bool(v, b)) { std::cerr << "[ERR] invalid bool: " << v << "\n"; return false; }
            a.hold_last_good = b;
        } else if (k == "--degrade-pub-hz") {
            const char* v = need("--degrade-pub-hz"); if (!v) return false;
            a.degrade_pub_hz = std::atof(v);
        } else if (k == "--print-hz") {
            const char* v = need("--print-hz"); if (!v) return false;
            a.print_hz = std::atof(v);
        } else if (k == "--event-log") {
            const char* v = need("--event-log"); if (!v) return false;
            a.event_log_path = v;
        } else {
            std::cerr << "[ERR] unknown arg: " << k << "\n";
            usage(argv[0]);
            return false;
        }
    }

    auto check_shm = [](const std::string& s, const char* name) -> bool {
        if (s.empty() || s.front() != '/') {
            std::cerr << "[ERR] " << name << " must start with '/': " << s << "\n";
            return false;
        }
        return true;
    };
    if (!check_shm(a.nav_state_shm, "nav_state_shm")) return false;
    if (!check_shm(a.nav_view_shm,  "nav_view_shm"))  return false;

    if (a.poll_hz <= 0.0)  { std::cerr << "[ERR] poll_hz must be > 0\n"; return false; }
    if (a.pub_hz < 0.0)    { std::cerr << "[ERR] pub_hz must be >= 0\n"; return false; }
    if (a.degrade_pub_hz < 0.0) { std::cerr << "[ERR] degrade_pub_hz must be >= 0\n"; return false; }
    if (a.print_hz <= 0.0) { std::cerr << "[ERR] print_hz must be > 0\n"; return false; }

    return true;
}

// period helpers
inline std::chrono::nanoseconds hz_to_period_ns(double hz)
{
    if (hz <= 0.0) return std::chrono::nanoseconds{0};
    const double sec = 1.0 / hz;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sec));
}

// bounded catch-up to avoid drift and huge catch-up loops after stalls
inline void advance_next(SteadyClock::time_point& next,
                         SteadyClock::time_point now,
                         std::chrono::nanoseconds period,
                         int max_catchup = 3)
{
    if (period.count() <= 0) { next = now; return; }
    if (now < next) return;

    int n = 0;
    while (now >= next && n < max_catchup) {
        next += period;
        ++n;
    }
    if (now >= next) next = now + period;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    if (!parse_args(argc, argv, args)) return 2;

    std::cerr
        << "[nav_viewd] starting...\n"
        << "  nav_state_shm=" << args.nav_state_shm << "\n"
        << "  nav_view_shm =" << args.nav_view_shm  << "\n"
        << "  poll_hz=" << args.poll_hz
        << " pub_hz=" << args.pub_hz
        << " degrade_pub_hz=" << args.degrade_pub_hz << "\n"
        << "  max_age_ms=" << args.max_age_ms
        << " warmup_ms=" << args.warmup_ms << "\n"
        << "  publish_when_stale=" << (args.publish_when_stale ? 1 : 0)
        << " hold_last_good=" << (args.hold_last_good ? 1 : 0)
        << " print_hz=" << args.print_hz << "\n"
        << "  event_log=" << args.event_log_path << "\n";
    if (args.hold_last_good) {
        std::cerr << "[nav_viewd][WARN] --hold-last-good is deprecated; "
                     "control-facing stale frames will still clear kinematics.\n";
    }

    NavViewEventCsvLogger event_logger;
    if (!event_logger.init(args.event_log_path)) {
        std::cerr << "[nav_viewd][WARN] event logger init failed, continue without nav events CSV\n";
    }

    // -------------------------------------------------------------------------
    // Init subscriber (NavState shm)
    // -------------------------------------------------------------------------
    comm_gcs::ipc::nav::NavStateSubscriberShm nav_sub;
    {
        comm_gcs::ipc::nav::NavStateSubscriberShm::Config cfg;
        cfg.enable    = true;
        cfg.shm_name  = args.nav_state_shm;
        cfg.shm_size  = 0;
        cfg.lazy_init = true;
        if (!nav_sub.init(cfg)) {
            std::cerr << "[nav_viewd][ERR] NavStateSubscriberShm.init failed\n";
            return 10;
        }
    }

    // -------------------------------------------------------------------------
    // Init publisher (NavView shm)
    // -------------------------------------------------------------------------
    comm_gcs::ipc::nav::NavViewPublisherShm nav_pub;
    {
        comm_gcs::ipc::nav::NavViewPublisherShm::Config cfg;
        cfg.enable   = true;
        cfg.shm_name = args.nav_view_shm;
        cfg.shm_size = 0;
        if (!nav_pub.init(cfg)) {
            std::cerr << "[nav_viewd][ERR] NavViewPublisherShm.init failed\n";
            return 11;
        }
    }

    comm_gcs::ipc::nav::NavViewBuilder builder;

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------
    const auto poll_period    = hz_to_period_ns(args.poll_hz);
    const auto pub_period     = hz_to_period_ns(args.pub_hz);
    const auto print_period   = hz_to_period_ns(args.print_hz);
    const auto degrade_period = hz_to_period_ns(args.degrade_pub_hz);

    const auto start_tp = SteadyClock::now();
    const std::uint64_t start_mono_ns = now_mono_ns();
    auto next_poll      = start_tp;
    auto next_pub       = start_tp;
    auto next_print     = start_tp;
    auto next_degrade   = start_tp;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    shared::msg::NavStateView last_view{};
    bool has_last_view = false;

    std::uint64_t last_nav_pub_mono_ns = 0;
    bool last_source_healthy = false;
    bool have_last_decision = false;
    DecisionSnapshot last_decision{};

    // stats
    std::uint64_t cnt_poll = 0, cnt_poll_hit = 0, cnt_no_nav = 0;
    std::uint64_t cnt_pub  = 0, cnt_pub_degrade = 0, cnt_stale = 0;

    while (!g_stop.load()) {
        const auto now_tp = SteadyClock::now();

        // ---------------- Poll phase ----------------
        if (poll_period.count() > 0 && now_tp >= next_poll) {
            advance_next(next_poll, now_tp, poll_period);
            ++cnt_poll;

            std::uint64_t mono_ns = 0, wall_ns = 0;
            auto nav_opt = nav_sub.poll(&mono_ns, &wall_ns);

            if (nav_opt.has_value()) {
                ++cnt_poll_hit;
                last_nav_pub_mono_ns = mono_ns;
                (void)wall_ns;

                // 保留最新一帧候选，不区分 valid/invalid，让上游状态机显式透传到 control-facing SHM。
                shared::msg::NavStateView cand = builder.build(*nav_opt);
                last_view = cand;
                has_last_view = true;
            } else {
                ++cnt_no_nav;
            }
        }

        // ---------------- Publish phase ----------------
        const bool pub_enabled = (pub_period.count() > 0);
        if (pub_enabled && now_tp >= next_pub) {
            advance_next(next_pub, now_tp, pub_period);
            const std::uint64_t now_ns = now_mono_ns();
            bool diagnostic_slot_ready = true;
            if (degrade_period.count() > 0 && now_tp < next_degrade) {
                diagnostic_slot_ready = false;
            }

            comm_gcs::ipc::nav::NavViewDaemonPolicyConfig policy_cfg{};
            policy_cfg.max_age_ms = args.max_age_ms;
            policy_cfg.warmup_ms = args.warmup_ms;
            policy_cfg.publish_when_stale = args.publish_when_stale;

            const auto decision = comm_gcs::ipc::nav::evaluate_nav_view_publish(
                has_last_view ? &last_view : nullptr,
                last_nav_pub_mono_ns,
                start_mono_ns,
                now_ns,
                policy_cfg,
                diagnostic_slot_ready);

            if (decision.stale_triggered) {
                ++cnt_stale;
            }

            if (decision.publish && decision.diagnostic_only && degrade_period.count() > 0) {
                advance_next(next_degrade, now_tp, degrade_period);
            }

            const std::uint64_t age_ms_from_nav_pub = (last_nav_pub_mono_ns == 0 || now_ns < last_nav_pub_mono_ns)
                ? UINT64_MAX
                : (now_ns - last_nav_pub_mono_ns) / 1000000ull;
            const DecisionSnapshot snap{
                decision.publish,
                decision.diagnostic_only,
                decision.degraded_publish,
                decision.no_nav_yet,
                decision.stale_triggered,
                decision.out.valid,
                decision.out.stale,
                decision.out.degraded,
                static_cast<std::uint16_t>(decision.out.fault_code),
                static_cast<std::uint8_t>(has_last_view ? last_view.valid : 0),
                static_cast<std::uint8_t>(has_last_view ? last_view.stale : 0),
                static_cast<std::uint8_t>(has_last_view ? last_view.degraded : 0),
                static_cast<std::uint16_t>(has_last_view ? last_view.fault_code : shared::msg::NavFaultCode::kNoData),
                age_ms_from_nav_pub,
            };

            // 这里只在决策组合变化时写事件，避免每个 publish slot 都重复刷同一条 stale/no-data 文本。
            if (!have_last_decision || !(snap == last_decision)) {
                const bool degraded_path = snap.diagnostic_only || snap.no_nav_yet || snap.stale_triggered;
                event_logger.log_event(
                    now_ns,
                    "nav_view_decision_changed",
                    degraded_path ? "warn" : "info",
                    snap.fault_code,
                    snap.nav_valid,
                    snap.nav_stale,
                    snap.nav_degraded,
                    snap.age_ms_from_nav_pub,
                    snap.publish,
                    snap.diagnostic_only,
                    snap.degraded_publish,
                    snap.no_nav_yet,
                    snap.stale_triggered,
                    snap.source_valid,
                    snap.source_stale,
                    snap.source_degraded,
                    snap.source_fault_code,
                    decision_reason(snap));
                last_decision = snap;
                have_last_decision = true;
            }

            const bool source_healthy = snap.publish && !snap.diagnostic_only && snap.nav_valid != 0 && snap.nav_stale == 0;
            if (!last_source_healthy && source_healthy) {
                event_logger.log_event(
                    now_ns,
                    "nav_view_source_recovered",
                    "info",
                    snap.fault_code,
                    snap.nav_valid,
                    snap.nav_stale,
                    snap.nav_degraded,
                    snap.age_ms_from_nav_pub,
                    snap.publish,
                    snap.diagnostic_only,
                    snap.degraded_publish,
                    snap.no_nav_yet,
                    snap.stale_triggered,
                    snap.source_valid,
                    snap.source_stale,
                    snap.source_degraded,
                    snap.source_fault_code,
                    "nav view returned to normal pass-through");
            }
            last_source_healthy = source_healthy;

            if (decision.publish) {
                if (nav_pub.publish(decision.out)) {
                    ++cnt_pub;
                    if (decision.degraded_publish) ++cnt_pub_degrade;
                } else {
                    std::cerr << "[nav_viewd][WARN] publish failed\n";
                    event_logger.log_event(
                        now_ns,
                        "nav_view_publish_failed",
                        "error",
                        static_cast<std::uint16_t>(decision.out.fault_code),
                        decision.out.valid,
                        decision.out.stale,
                        decision.out.degraded,
                        age_ms_from_nav_pub,
                        decision.publish,
                        decision.diagnostic_only,
                        decision.degraded_publish,
                        decision.no_nav_yet,
                        decision.stale_triggered,
                        has_last_view ? last_view.valid : 0,
                        has_last_view ? last_view.stale : 0,
                        has_last_view ? last_view.degraded : 0,
                        static_cast<std::uint16_t>(has_last_view ? last_view.fault_code : shared::msg::NavFaultCode::kNoData),
                        "NavViewPublisherShm.publish returned false");
                }
            }
        }

        // ---------------- Diagnostics ----------------
        if (print_period.count() > 0 && now_tp >= next_print) {
            advance_next(next_print, now_tp, print_period);

            const std::uint64_t now_ns = now_mono_ns();
            const std::uint64_t age_ms_from_nav_pub = (last_nav_pub_mono_ns == 0)
                ? UINT64_MAX
                : (now_ns - last_nav_pub_mono_ns) / 1000000ull;

            std::cerr
                << "[nav_viewd] poll=" << cnt_poll
                << " hit=" << cnt_poll_hit
                << " no_nav=" << cnt_no_nav
                << " pub=" << cnt_pub
                << " degrade_pub=" << cnt_pub_degrade
                << " stale_cnt=" << cnt_stale
                << " last_age_ms=" << (age_ms_from_nav_pub == UINT64_MAX ? -1LL
                                  : static_cast<long long>(age_ms_from_nav_pub))
                << " sub_init=" << (nav_sub.initialized() ? 1 : 0)
                << "\n";
        }

        // ---------------- Sleep until next event ----------------
        SteadyClock::time_point next_wake = now_tp + std::chrono::milliseconds(50);

        if (poll_period.count() > 0)  next_wake = std::min(next_wake, next_poll);
        if (pub_period.count() > 0)   next_wake = std::min(next_wake, next_pub);
        if (print_period.count() > 0) next_wake = std::min(next_wake, next_print);
        if (degrade_period.count() > 0) next_wake = std::min(next_wake, next_degrade);

        auto sleep_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(next_wake - SteadyClock::now());
        if (sleep_ns.count() < 0) sleep_ns = std::chrono::nanoseconds(0);

        auto sleep_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sleep_ns);
        if (sleep_ms < std::chrono::milliseconds(1))  sleep_ms = std::chrono::milliseconds(1);
        if (sleep_ms > std::chrono::milliseconds(50)) sleep_ms = std::chrono::milliseconds(50);

        std::this_thread::sleep_for(sleep_ms);
    }

    std::cerr << "[nav_viewd] stopping...\n";
    nav_pub.shutdown();
    nav_sub.shutdown();
    return 0;
}
