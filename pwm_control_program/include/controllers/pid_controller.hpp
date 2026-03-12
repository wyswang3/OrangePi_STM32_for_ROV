#pragma once

#include "controllers/controller_base.hpp"

namespace rovctrl::controllers {

struct PidAxisConfig {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    bool   anti_windup_enabled{true};
    double i_min{-1.0};
    double i_max{1.0};

    double out_min{-1.0};
    double out_max{1.0};

    double error_deadband{0.0};

    bool   derivative_filter_enabled{false};
    double derivative_cutoff_hz{5.0};

    bool   rate_limit_enabled{false};
    double max_abs_du{0.0}; // output units / second

    bool   wrap_error{false};
    double wrap_min{-3.14159265358979323846};
    double wrap_max{ 3.14159265358979323846};

    bool   invert_error{false};
};

struct PidAxisState {
    double integrator{0.0};
    double prev_error{0.0};
    double filtered_derivative{0.0};
    double last_output{0.0};
    bool   has_prev_error{false};
    bool   has_last_output{false};
};

class PidAxis final {
public:
    explicit PidAxis(const PidAxisConfig& cfg = PidAxisConfig()) noexcept
        : cfg_(cfg) {}

    void reset() noexcept;
    double step(double reference, double measurement, double dt_sec) noexcept;

private:
    PidAxisConfig cfg_{};
    PidAxisState  st_{};
};

struct DepthHoldPidControllerConfig {
    PidAxisConfig depth{};
    bool          require_nav{true};
};

struct HeadingHoldPidControllerConfig {
    PidAxisConfig heading{};
    bool          require_nav{true};
};

struct DepthHeadingPidControllerConfig {
    PidAxisConfig depth{};
    PidAxisConfig heading{};
    bool          require_nav{true};
};

struct VelocityHoldControllerConfig {
    bool reserved{true};
};

class DepthHoldPidController final : public IController {
public:
    explicit DepthHoldPidController(const DepthHoldPidControllerConfig& cfg = {}) noexcept;

    const char* name() const noexcept override { return "depth_hold_pid"; }
    rovctrl::control_core::ControlMode mode() const noexcept override;
    void reset() noexcept override;

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt_sec) noexcept override;

private:
    DepthHoldPidControllerConfig cfg_{};
    PidAxis                      depth_axis_{};
    double                       depth_ref_m_{0.0};
    bool                         depth_ref_latched_{false};
};

class HeadingHoldPidController final : public IController {
public:
    explicit HeadingHoldPidController(const HeadingHoldPidControllerConfig& cfg = {}) noexcept;

    const char* name() const noexcept override { return "heading_hold_pid"; }
    rovctrl::control_core::ControlMode mode() const noexcept override;
    void reset() noexcept override;

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt_sec) noexcept override;

private:
    HeadingHoldPidControllerConfig cfg_{};
    PidAxis                        heading_axis_{};
    double                         heading_ref_rad_{0.0};
    bool                           heading_ref_latched_{false};
};

class DepthHeadingPidController final : public IController {
public:
    explicit DepthHeadingPidController(const DepthHeadingPidControllerConfig& cfg = {}) noexcept;

    const char* name() const noexcept override { return "depth_heading_pid"; }
    rovctrl::control_core::ControlMode mode() const noexcept override;
    void reset() noexcept override;

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt_sec) noexcept override;

private:
    DepthHeadingPidControllerConfig cfg_{};
    PidAxis                         depth_axis_{};
    PidAxis                         heading_axis_{};
    double                          depth_ref_m_{0.0};
    double                          heading_ref_rad_{0.0};
    bool                            depth_ref_latched_{false};
    bool                            heading_ref_latched_{false};
};

class PidController final : public IController {
public:
    explicit PidController(const DepthHeadingPidControllerConfig& cfg = {}) noexcept;

    const char* name() const noexcept override { return "pid"; }
    rovctrl::control_core::ControlMode mode() const noexcept override;
    void reset() noexcept override;

    bool compute(const rovctrl::control_core::ControlState&     state,
                 const rovctrl::control_core::ControlReference& ref,
                 rovctrl::control_core::ControlOutput&          out,
                 double                                         dt_sec) noexcept override;

private:
    DepthHeadingPidController impl_{};
};

} // namespace rovctrl::controllers
