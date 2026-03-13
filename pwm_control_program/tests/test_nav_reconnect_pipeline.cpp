#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "control_core/control_guard.hpp"
#include "control_core/telemetry_frame_builder.hpp"
#include "gateway/telemetry/status_telemetry_adapter.hpp"
#include "gateway/IPC/nav/nav_state_subscriber_shm.hpp"
#include "gateway/IPC/nav/nav_view_builder.hpp"
#include "gateway/IPC/nav/nav_view_policy.hpp"
#include "gateway/IPC/nav/nav_view_publisher_shm.hpp"
#include "io/log/telemetry_timeline_logger.hpp"
#include "io/nav/nav_view_shm_source.hpp"
#include "shared/msg/nav_state.hpp"

namespace {

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " CHECK(" #cond ") failed\n";                                            \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_EQ(a, b)                                                                             \
    do {                                                                                          \
        const auto _va = (a);                                                                     \
        const auto _vb = (b);                                                                     \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed\n";                                          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

namespace cc = rovctrl::control_core;

constexpr std::uint32_t kNavStateMagic =
    (static_cast<std::uint32_t>('N') << 24) |
    (static_cast<std::uint32_t>('A') << 16) |
    (static_cast<std::uint32_t>('V') << 8) |
    (static_cast<std::uint32_t>('1'));
constexpr std::uint32_t kNavStateLayoutVersion = 1;
constexpr std::uint32_t kNavStatePayloadVersion = 2;

struct NavStateShmHeader final {
    std::atomic<std::uint64_t> seq{0};
    std::uint64_t mono_ns = 0;
    std::uint64_t wall_ns = 0;
    std::uint32_t magic = kNavStateMagic;
    std::uint32_t layout_ver = kNavStateLayoutVersion;
    std::uint32_t payload_ver = kNavStatePayloadVersion;
    std::uint32_t payload_size = static_cast<std::uint32_t>(sizeof(shared::msg::NavState));
    std::uint32_t payload_align = static_cast<std::uint32_t>(alignof(shared::msg::NavState));
    std::uint32_t reserved0 = 0;
};

struct NavStateShmLayout final {
    NavStateShmHeader hdr{};
    shared::msg::NavState payload{};
};

std::uint64_t unique_now_ns()
{
    using Clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

std::string unique_shm_name(const char* prefix)
{
    return std::string("/") + prefix + "_" + std::to_string(unique_now_ns());
}

std::filesystem::path unique_temp_path(const char* prefix, const char* suffix)
{
    return std::filesystem::temp_directory_path() /
           (std::string(prefix) + "_" + std::to_string(unique_now_ns()) + suffix);
}

class NavStateShmWriter final {
public:
    explicit NavStateShmWriter(std::string shm_name)
        : shm_name_(std::move(shm_name))
    {
    }

    ~NavStateShmWriter()
    {
        close();
    }

    bool init()
    {
        fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_ < 0) {
            std::perror("shm_open");
            return false;
        }

        if (::ftruncate(fd_, static_cast<off_t>(sizeof(NavStateShmLayout))) != 0) {
            std::perror("ftruncate");
            close();
            return false;
        }

        ptr_ = ::mmap(nullptr,
                      sizeof(NavStateShmLayout),
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED,
                      fd_,
                      0);
        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            std::perror("mmap");
            close();
            return false;
        }

        auto* layout = static_cast<NavStateShmLayout*>(ptr_);
        layout->hdr.magic = kNavStateMagic;
        layout->hdr.layout_ver = kNavStateLayoutVersion;
        layout->hdr.payload_ver = kNavStatePayloadVersion;
        layout->hdr.payload_size = static_cast<std::uint32_t>(sizeof(shared::msg::NavState));
        layout->hdr.payload_align = static_cast<std::uint32_t>(alignof(shared::msg::NavState));
        layout->hdr.seq.store(0, std::memory_order_relaxed);
        return true;
    }

    void write(const shared::msg::NavState& nav,
               std::uint64_t                pub_mono_ns,
               std::uint64_t                pub_wall_ns)
    {
        auto* layout = static_cast<NavStateShmLayout*>(ptr_);
        if (layout == nullptr) {
            return;
        }

        const auto seq0 = layout->hdr.seq.load(std::memory_order_relaxed);
        layout->hdr.seq.store(seq0 + 1, std::memory_order_release);
        layout->hdr.mono_ns = pub_mono_ns;
        layout->hdr.wall_ns = pub_wall_ns;
        std::memcpy(&layout->payload, &nav, sizeof(nav));
        layout->hdr.seq.store(seq0 + 2, std::memory_order_release);
    }

    const std::string& shm_name() const noexcept { return shm_name_; }

private:
    void close()
    {
        if (ptr_ != nullptr) {
            ::munmap(ptr_, sizeof(NavStateShmLayout));
            ptr_ = nullptr;
        }
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        if (!shm_name_.empty()) {
            ::shm_unlink(shm_name_.c_str());
        }
    }

    std::string shm_name_{};
    int         fd_{-1};
    void*       ptr_{nullptr};
};

struct NavViewDaemonHarness final {
    comm_gcs::ipc::nav::NavStateSubscriberShm nav_sub;
    comm_gcs::ipc::nav::NavViewPublisherShm nav_pub;
    shared::msg::NavStateView last_view{};
    bool has_last_view{false};
    std::uint64_t last_nav_pub_mono_ns{0};
    std::uint64_t start_mono_ns{0};
    comm_gcs::ipc::nav::NavViewDaemonPolicyConfig policy{};

    bool init(const std::string& nav_state_shm, const std::string& nav_view_shm)
    {
        comm_gcs::ipc::nav::NavStateSubscriberShm::Config scfg{};
        scfg.enable = true;
        scfg.shm_name = nav_state_shm;
        scfg.lazy_init = false;
        if (!nav_sub.init(scfg)) {
            return false;
        }

        comm_gcs::ipc::nav::NavViewPublisherShm::Config pcfg{};
        pcfg.enable = true;
        pcfg.shm_name = nav_view_shm;
        if (!nav_pub.init(pcfg)) {
            return false;
        }

        start_mono_ns = unique_now_ns();
        return true;
    }

    std::optional<shared::msg::NavStateView> step(std::uint64_t now_mono_ns,
                                                  bool diagnostic_slot_ready = true)
    {
        auto nav = nav_sub.poll(&last_nav_pub_mono_ns, nullptr);
        if (nav.has_value()) {
            last_view = comm_gcs::ipc::nav::NavViewBuilder::build(*nav);
            has_last_view = true;
        }

        const auto decision = comm_gcs::ipc::nav::evaluate_nav_view_publish(
            has_last_view ? &last_view : nullptr,
            last_nav_pub_mono_ns,
            start_mono_ns,
            now_mono_ns,
            policy,
            diagnostic_slot_ready);
        if (!decision.publish) {
            return std::nullopt;
        }
        if (!nav_pub.publish(decision.out)) {
            return std::nullopt;
        }
        return decision.out;
    }
};

shared::msg::NavState make_nav_reconnecting(std::uint64_t stamp_ns)
{
    shared::msg::NavState nav{};
    nav.t_ns = stamp_ns;
    nav.age_ms = 20;
    nav.valid = 0;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = shared::msg::NavRunState::kUninitialized;
    nav.health = shared::msg::NavHealth::UNINITIALIZED;
    nav.fault_code = shared::msg::NavFaultCode::kImuDisconnected;
    nav.status_flags = shared::msg::NAV_FLAG_IMU_RECONNECTING;
    nav.sensor_mask = shared::msg::NAV_SENSOR_NONE;
    return nav;
}

shared::msg::NavState make_nav_ok(std::uint64_t stamp_ns)
{
    shared::msg::NavState nav{};
    nav.t_ns = stamp_ns;
    nav.age_ms = 20;
    nav.valid = 1;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = shared::msg::NavRunState::kOk;
    nav.health = shared::msg::NavHealth::OK;
    nav.fault_code = shared::msg::NavFaultCode::kNone;
    nav.sensor_mask = shared::msg::NAV_SENSOR_IMU;
    nav.status_flags = shared::msg::NAV_FLAG_IMU_OK |
                       shared::msg::NAV_FLAG_IMU_DEVICE_ONLINE |
                       shared::msg::NAV_FLAG_ALIGN_DONE |
                       shared::msg::NAV_FLAG_ESKF_OK;
    nav.pos[0] = 12.3;
    nav.vel[0] = 0.4;
    nav.rpy[2] = 0.1;
    return nav;
}

shared::msg::NavState make_nav_mismatch(std::uint64_t stamp_ns)
{
    shared::msg::NavState nav{};
    nav.t_ns = stamp_ns;
    nav.age_ms = 30;
    nav.valid = 0;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = shared::msg::NavRunState::kInvalid;
    nav.health = shared::msg::NavHealth::INVALID;
    nav.fault_code = shared::msg::NavFaultCode::kDvlDeviceMismatch;
    nav.status_flags = shared::msg::NAV_FLAG_DVL_BIND_MISMATCH;
    nav.sensor_mask = shared::msg::NAV_SENSOR_IMU;
    return nav;
}

void write_nav_state_records(const std::filesystem::path&                     path,
                             const std::vector<shared::msg::NavState>& records)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("cannot open nav_state replay file");
    }
    out.write(reinterpret_cast<const char*>(records.data()),
              static_cast<std::streamsize>(records.size() * sizeof(shared::msg::NavState)));
}

std::vector<shared::msg::NavState> read_nav_state_records(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("cannot read nav_state replay file");
    }

    in.seekg(0, std::ios::end);
    const auto size = static_cast<std::size_t>(in.tellg());
    in.seekg(0, std::ios::beg);
    if ((size % sizeof(shared::msg::NavState)) != 0) {
        throw std::runtime_error("nav_state replay file has unexpected size");
    }

    std::vector<shared::msg::NavState> records(size / sizeof(shared::msg::NavState));
    if (!records.empty()) {
        in.read(reinterpret_cast<char*>(records.data()), static_cast<std::streamsize>(size));
    }
    return records;
}

cc::ControlIntent make_arm_cmd()
{
    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    return arm;
}

cc::ControlIntent make_auto_cmd()
{
    cc::ControlIntent cmd{};
    cmd.cmd_seq = 2;
    cmd.stamp_ns = 2;
    cmd.ttl_ms = 100;
    cmd.valid = true;
    cmd.has_mode_request = true;
    cmd.mode_request = cc::ControlMode::kAuto;
    return cmd;
}

shared::msg::TelemetryFrameV2 make_telemetry_from_nav(const rovctrl::io::NavStateView& nav_view,
                                                      cc::ControlMode effective_mode,
                                                      bool failsafe_active)
{
    shared::msg::TelemetryFrameV2 frame{};
    cc::TelemetryBuildInput input{};
    input.stamp_ns = unique_now_ns();
    input.active_mode = effective_mode;
    input.armed = true;
    input.estop_latched = false;
    input.failsafe_active = failsafe_active;
    input.pwm_ok = true;
    input.nav_snapshot = &nav_view;
    input.nav_age_ms = nav_view.total_age_ms();
    cc::fill_telemetry_frame_v2(frame, input);
    return frame;
}

int test_reconnecting_nav_fault_reaches_control_and_telemetry()
{
    const std::string nav_state_shm = unique_shm_name("codex_nav_state_reconnect");
    const std::string nav_view_shm = unique_shm_name("codex_nav_view_reconnect");

    NavStateShmWriter writer(nav_state_shm);
    TEST_CHECK(writer.init());

    NavViewDaemonHarness daemon{};
    daemon.policy.max_age_ms = 250;
    daemon.policy.warmup_ms = 0;
    daemon.policy.publish_when_stale = true;
    TEST_CHECK(daemon.init(nav_state_shm, nav_view_shm));

    const auto stamp_ns = unique_now_ns();
    writer.write(make_nav_reconnecting(stamp_ns), stamp_ns + 20'000'000ull, 0);
    const auto published = daemon.step(stamp_ns + 30'000'000ull);
    TEST_CHECK(published.has_value());
    TEST_EQ(published->fault_code, shared::msg::NavFaultCode::kImuDisconnected);

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = nav_view_shm;
    scfg.max_age_ms = 1000;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::NavStateView nav_view{};
    TEST_CHECK(src.read_latest(nav_view));
    TEST_EQ(nav_view.payload().valid, 0);
    TEST_EQ(nav_view.payload().fault_code, shared::msg::NavFaultCode::kImuDisconnected);
    TEST_CHECK(shared::msg::nav_flag_has(
        nav_view.payload().status_flags, shared::msg::NAV_FLAG_IMU_RECONNECTING));

    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};
    TEST_CHECK(guard.step(1, state, nullptr, make_arm_cmd()).armed);
    const auto out = guard.step(2, state, &nav_view.payload(), make_auto_cmd());
    TEST_EQ(static_cast<int>(out.effective_mode), static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(static_cast<int>(out.failsafe), static_cast<int>(cc::FailsafeAction::kZeroOutput));

    const auto frame = make_telemetry_from_nav(nav_view, out.effective_mode, true);
    TEST_EQ(frame.system.nav_valid, 0);
    TEST_EQ(frame.system.nav_stale, 0);
    TEST_EQ(frame.system.nav_degraded, 0);
    TEST_EQ(frame.system.nav_age_ms, nav_view.total_age_ms());
    TEST_EQ(frame.system.nav_fault_code,
            static_cast<std::uint16_t>(shared::msg::NavFaultCode::kImuDisconnected));
    TEST_CHECK(shared::msg::nav_flag_has(frame.system.nav_status_flags,
                                         shared::msg::NAV_FLAG_IMU_RECONNECTING));
    TEST_EQ(frame.system.health_state,
            static_cast<std::uint8_t>(shared::msg::HealthState::kFault));
    return 0;
}

int test_daemon_stale_publish_clears_old_kinematics_and_rejects_auto()
{
    const std::string nav_state_shm = unique_shm_name("codex_nav_state_stale");
    const std::string nav_view_shm = unique_shm_name("codex_nav_view_stale");

    NavStateShmWriter writer(nav_state_shm);
    TEST_CHECK(writer.init());

    NavViewDaemonHarness daemon{};
    daemon.policy.max_age_ms = 50;
    daemon.policy.warmup_ms = 0;
    daemon.policy.publish_when_stale = true;
    TEST_CHECK(daemon.init(nav_state_shm, nav_view_shm));

    const auto stamp_ns = unique_now_ns();
    const auto nav_pub_mono_ns = stamp_ns + 10'000'000ull;
    writer.write(make_nav_ok(stamp_ns), nav_pub_mono_ns, 0);

    const auto seeded = daemon.step(nav_pub_mono_ns + 5'000'000ull);
    TEST_CHECK(seeded.has_value());
    TEST_EQ(seeded->valid, 1);

    const auto stale_pub = daemon.step(nav_pub_mono_ns + 120'000'000ull);
    TEST_CHECK(stale_pub.has_value());
    TEST_EQ(stale_pub->valid, 0);
    TEST_EQ(stale_pub->stale, 1);
    TEST_EQ(stale_pub->fault_code, shared::msg::NavFaultCode::kNavViewStale);
    TEST_EQ(stale_pub->pos[0], 0.0);

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = nav_view_shm;
    scfg.max_age_ms = 1000;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::NavStateView nav_view{};
    TEST_CHECK(src.read_latest(nav_view));
    TEST_EQ(nav_view.payload().valid, 0);
    TEST_EQ(nav_view.payload().stale, 1);
    TEST_EQ(nav_view.payload().nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(nav_view.payload().pos[0], 0.0);
    TEST_CHECK(nav_view.payload().age_ms >= 120u);

    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};
    TEST_CHECK(guard.step(1, state, nullptr, make_arm_cmd()).armed);
    const auto out = guard.step(2, state, &nav_view.payload(), make_auto_cmd());
    TEST_EQ(static_cast<int>(out.effective_mode), static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(static_cast<int>(out.failsafe), static_cast<int>(cc::FailsafeAction::kZeroOutput));

    const auto frame = make_telemetry_from_nav(nav_view, out.effective_mode, true);
    TEST_EQ(frame.system.nav_valid, 0);
    TEST_EQ(frame.system.nav_stale, 1);
    TEST_EQ(frame.system.nav_degraded, 1);
    TEST_CHECK(frame.system.nav_age_ms >= 120u);
    TEST_EQ(frame.system.nav_fault_code,
            static_cast<std::uint16_t>(shared::msg::NavFaultCode::kNavViewStale));
    return 0;
}

int test_replay_bundle_file_reaches_status_telemetry_and_logger()
{
    const std::string nav_state_shm = unique_shm_name("codex_nav_state_replay");
    const std::string nav_view_shm = unique_shm_name("codex_nav_view_replay");
    const auto replay_file = unique_temp_path("nav_state_window", ".bin");
    const auto log_root = unique_temp_path("telemetry_replay", "");
    std::filesystem::create_directories(log_root);

    const std::vector<shared::msg::NavState> recorded{
        make_nav_ok(1'000'000'000ull),
        make_nav_reconnecting(1'050'000'000ull),
        make_nav_mismatch(1'100'000'000ull),
    };
    write_nav_state_records(replay_file, recorded);

    NavStateShmWriter writer(nav_state_shm);
    TEST_CHECK(writer.init());

    NavViewDaemonHarness daemon{};
    daemon.policy.max_age_ms = 80;
    daemon.policy.warmup_ms = 0;
    daemon.policy.publish_when_stale = true;
    TEST_CHECK(daemon.init(nav_state_shm, nav_view_shm));

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = nav_view_shm;
    scfg.max_age_ms = 1000;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::TelemetryTimelineLogger logger;
    TEST_CHECK(logger.init(log_root.string(), "replay"));

    const auto replayed = read_nav_state_records(replay_file);
    TEST_EQ(replayed.size(), recorded.size());

    writer.write(replayed[0], replayed[0].t_ns, 0);
    TEST_CHECK(daemon.step(replayed[0].t_ns + 1'000'000ull).has_value());

    writer.write(replayed[1], replayed[1].t_ns, 0);
    TEST_CHECK(daemon.step(replayed[1].t_ns + 1'000'000ull).has_value());

    rovctrl::io::NavStateView reconnect_view{};
    TEST_CHECK(src.read_latest(reconnect_view));
    TEST_EQ(reconnect_view.payload().valid, 0);
    TEST_EQ(reconnect_view.payload().fault_code, shared::msg::NavFaultCode::kImuDisconnected);
    TEST_CHECK(shared::msg::nav_flag_has(
        reconnect_view.payload().status_flags, shared::msg::NAV_FLAG_IMU_RECONNECTING));

    writer.write(replayed[2], replayed[2].t_ns, 0);
    TEST_CHECK(daemon.step(replayed[2].t_ns + 1'000'000ull).has_value());

    rovctrl::io::NavStateView mismatch_view{};
    TEST_CHECK(src.read_latest(mismatch_view));
    TEST_EQ(mismatch_view.payload().valid, 0);
    TEST_EQ(mismatch_view.payload().fault_code, shared::msg::NavFaultCode::kDvlDeviceMismatch);
    TEST_CHECK(shared::msg::nav_flag_has(
        mismatch_view.payload().status_flags, shared::msg::NAV_FLAG_DVL_BIND_MISMATCH));

    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};
    TEST_CHECK(guard.step(1, state, nullptr, make_arm_cmd()).armed);
    const auto out = guard.step(2, state, &mismatch_view.payload(), make_auto_cmd());
    TEST_EQ(static_cast<int>(out.effective_mode), static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(static_cast<int>(out.failsafe), static_cast<int>(cc::FailsafeAction::kZeroOutput));

    auto frame = make_telemetry_from_nav(mismatch_view, out.effective_mode, true);
    frame.seq = 7;
    frame.last_command_result.cmd_seq = 22;
    frame.last_command_result.intent_id = 22;
    frame.last_command_result.stamp_ns = replayed[2].t_ns + 2'000'000ull;
    frame.last_command_result.status =
        static_cast<std::uint8_t>(shared::msg::CommandResultCode::kRejected);
    frame.last_command_result.fault_code =
        static_cast<std::uint16_t>(shared::msg::FaultCode::kNavUntrusted);
    logger.log_frame(frame);

    const auto status = comm_gcs::telemetry::build_status_telemetry(frame, true, true);
    TEST_EQ(status.nav_fault_code,
            static_cast<std::uint16_t>(shared::msg::NavFaultCode::kDvlDeviceMismatch));
    TEST_EQ(status.nav_status_flags, shared::msg::NAV_FLAG_DVL_BIND_MISMATCH);
    TEST_EQ(status.command_status,
            static_cast<std::uint8_t>(shared::msg::CommandResultCode::kRejected));
    TEST_EQ(status.command_fault_code,
            static_cast<std::uint16_t>(shared::msg::FaultCode::kNavUntrusted));

    TEST_CHECK(daemon.step(replayed[2].t_ns + 250'000'000ull).has_value());
    rovctrl::io::NavStateView stale_view{};
    TEST_CHECK(src.read_latest(stale_view));
    TEST_EQ(stale_view.payload().stale, 1);
    TEST_EQ(stale_view.payload().fault_code, shared::msg::NavFaultCode::kNavViewStale);

    logger.close();

    std::filesystem::path timeline_file;
    std::filesystem::path events_file;
    for (const auto& entry : std::filesystem::directory_iterator(log_root)) {
        const auto name = entry.path().filename().string();
        if (name.find("_timeline_") != std::string::npos) {
            timeline_file = entry.path();
        } else if (name.find("_events_") != std::string::npos) {
            events_file = entry.path();
        }
    }
    TEST_CHECK(!timeline_file.empty());
    TEST_CHECK(!events_file.empty());

    std::ifstream timeline_in(timeline_file);
    std::string header;
    std::string row;
    std::getline(timeline_in, header);
    std::getline(timeline_in, row);
    TEST_CHECK(row.find(",14,512,") != std::string::npos);
    TEST_CHECK(row.find(",2,4,") != std::string::npos);

    std::error_code ec;
    std::filesystem::remove(replay_file, ec);
    std::filesystem::remove_all(log_root, ec);
    return 0;
}

} // namespace

int main()
{
    int rc = test_reconnecting_nav_fault_reaches_control_and_telemetry();
    if (rc != 0) {
        return rc;
    }
    rc = test_daemon_stale_publish_clears_old_kinematics_and_rejects_auto();
    if (rc != 0) {
        return rc;
    }
    rc = test_replay_bundle_file_reaches_status_telemetry_and_logger();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_nav_reconnect_pipeline] all tests passed.\n";
    return 0;
}
