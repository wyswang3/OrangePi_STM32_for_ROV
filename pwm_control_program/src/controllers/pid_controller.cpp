#include "controllers/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rovctrl::controllers {

using rovctrl::control_core::ControlMode;
using rovctrl::control_core::ControlOutput;
using rovctrl::control_core::ControlReference;
using rovctrl::control_core::ControlState;

namespace {

constexpr double kPi = 3.14159265358979323846;

inline bool finite(double v) noexcept
{
    return std::isfinite(v);
}

inline double clamp_dt(double dt_sec) noexcept
{
    if (!finite(dt_sec) || dt_sec <= 0.0) return 0.01;
    if (dt_sec > 0.2) return 0.2;
    return dt_sec;
}

inline double clamp(double v, double lo, double hi) noexcept
{
    return std::clamp(v, lo, hi);
}

inline double wrap_to_range(double v, double lo, double hi) noexcept
{
    if (!(finite(v) && finite(lo) && finite(hi)) || hi <= lo) return v;
    const double span = hi - lo;
    while (v > hi) v -= span;
    while (v < lo) v += span;
    return v;
}

inline double apply_deadband(double v, double db) noexcept
{
    if (db <= 0.0) return v;
    return (std::fabs(v) <= db) ? 0.0 : v;
}

inline bool extract_depth_reference(const ControlReference& ref,
                                    double&                 out_depth_m) noexcept
{
    if (!ref.use_pose_ref) return false;
    out_depth_m = ref.pose_ref.z;
    return finite(out_depth_m);
}

inline bool extract_heading_reference(const ControlReference& ref,
                                      double&                 out_heading_rad) noexcept
{
    if (!ref.use_pose_ref) return false;
    out_heading_rad = ref.pose_ref.yaw;
    return finite(out_heading_rad);
}

inline bool nav_ready(const ControlState& state, bool require_nav) noexcept
{
    if (require_nav) return state.nav_valid;
    return state.nav_valid || state.has_pose;
}

inline bool extract_depth_measurement(const ControlState& state,
                                      bool                require_nav,
                                      double&             out_depth_m) noexcept
{
    if (state.nav_valid) {
        out_depth_m = state.nav_depth;
        return finite(out_depth_m);
    }
    if (!require_nav && state.has_pose) {
        out_depth_m = state.pose.z;
        return finite(out_depth_m);
    }
    return false;
}

inline bool extract_heading_measurement(const ControlState& state,
                                        bool                require_nav,
                                        double&             out_heading_rad) noexcept
{
    if (state.nav_valid) {
        out_heading_rad = state.nav_rpy[2];
        return finite(out_heading_rad);
    }
    if (!require_nav && state.has_pose) {
        out_heading_rad = state.pose.yaw;
        return finite(out_heading_rad);
    }
    return false;
}

inline double latch_reference(bool   has_explicit_ref,
                              double explicit_ref,
                              double measurement,
                              double& latched_ref,
                              bool&   latched) noexcept
{
    if (has_explicit_ref && finite(explicit_ref)) {
        latched_ref = explicit_ref;
        latched = true;
        return latched_ref;
    }
    if (!latched || !finite(latched_ref)) {
        latched_ref = measurement;
        latched = true;
    }
    return latched_ref;
}

} // namespace

void PidAxis::reset() noexcept
{
    st_ = PidAxisState{};
}

double PidAxis::step(double reference, double measurement, double dt_sec) noexcept
{
    dt_sec = clamp_dt(dt_sec);

    double error = cfg_.invert_error ? (measurement - reference)
                                     : (reference - measurement);
    if (cfg_.wrap_error) {
        error = wrap_to_range(error, cfg_.wrap_min, cfg_.wrap_max);
    }
    error = apply_deadband(error, cfg_.error_deadband);

    double derivative = 0.0;
    if (st_.has_prev_error) {
        derivative = (error - st_.prev_error) / dt_sec;
    }

    if (cfg_.derivative_filter_enabled && cfg_.derivative_cutoff_hz > 0.0) {
        const double rc = 1.0 / (2.0 * kPi * cfg_.derivative_cutoff_hz);
        const double alpha = dt_sec / (rc + dt_sec);
        st_.filtered_derivative += alpha * (derivative - st_.filtered_derivative);
        derivative = st_.filtered_derivative;
    } else {
        st_.filtered_derivative = derivative;
    }

    double next_integrator = st_.integrator;
    if (cfg_.ki != 0.0) {
        next_integrator += error * dt_sec;
        if (cfg_.anti_windup_enabled) {
            next_integrator = clamp(next_integrator, cfg_.i_min, cfg_.i_max);
        }
    }

    const double unsat = cfg_.kp * error +
                         cfg_.ki * next_integrator +
                         cfg_.kd * derivative;
    double output = clamp(unsat, cfg_.out_min, cfg_.out_max);

    if (cfg_.anti_windup_enabled && output != unsat) {
        const bool pushing_positive_sat = (unsat > cfg_.out_max) && (error > 0.0);
        const bool pushing_negative_sat = (unsat < cfg_.out_min) && (error < 0.0);
        if (!(pushing_positive_sat || pushing_negative_sat)) {
            st_.integrator = next_integrator;
        }
    } else {
        st_.integrator = next_integrator;
    }

    if (cfg_.rate_limit_enabled && cfg_.max_abs_du > 0.0 && st_.has_last_output) {
        const double max_delta = cfg_.max_abs_du * dt_sec;
        output = clamp(output,
                       st_.last_output - max_delta,
                       st_.last_output + max_delta);
    }

    st_.prev_error = error;
    st_.has_prev_error = true;
    st_.last_output = output;
    st_.has_last_output = true;
    return output;
}

DepthHoldPidController::DepthHoldPidController(
    const DepthHoldPidControllerConfig& cfg) noexcept
    : cfg_(cfg),
      depth_axis_(cfg.depth)
{
    reset();
}

ControlMode DepthHoldPidController::mode() const noexcept
{
    return ControlMode::kAuto;
}

void DepthHoldPidController::reset() noexcept
{
    depth_axis_.reset();
    depth_ref_m_ = 0.0;
    depth_ref_latched_ = false;
}

bool DepthHoldPidController::compute(const ControlState&     state,
                                     const ControlReference& ref,
                                     ControlOutput&          out,
                                     double                  dt_sec) noexcept
{
    out = ControlOutput{};

    if (!nav_ready(state, cfg_.require_nav)) return false;

    double depth_meas = 0.0;
    if (!extract_depth_measurement(state, cfg_.require_nav, depth_meas)) return false;

    double depth_ref = 0.0;
    const bool has_depth_ref = extract_depth_reference(ref, depth_ref);
    depth_ref = latch_reference(has_depth_ref,
                                depth_ref,
                                depth_meas,
                                depth_ref_m_,
                                depth_ref_latched_);

    out.has_body_wrench = true;
    out.body_wrench.fill(0.0);
    out.body_wrench[2] = depth_axis_.step(depth_ref, depth_meas, dt_sec);
    return finite(out.body_wrench[2]);
}

HeadingHoldPidController::HeadingHoldPidController(
    const HeadingHoldPidControllerConfig& cfg) noexcept
    : cfg_(cfg),
      heading_axis_(cfg.heading)
{
    reset();
}

ControlMode HeadingHoldPidController::mode() const noexcept
{
    return ControlMode::kAuto;
}

void HeadingHoldPidController::reset() noexcept
{
    heading_axis_.reset();
    heading_ref_rad_ = 0.0;
    heading_ref_latched_ = false;
}

bool HeadingHoldPidController::compute(const ControlState&     state,
                                       const ControlReference& ref,
                                       ControlOutput&          out,
                                       double                  dt_sec) noexcept
{
    out = ControlOutput{};

    if (!nav_ready(state, cfg_.require_nav)) return false;

    double heading_meas = 0.0;
    if (!extract_heading_measurement(state, cfg_.require_nav, heading_meas)) return false;

    double heading_ref = 0.0;
    const bool has_heading_ref = extract_heading_reference(ref, heading_ref);
    heading_ref = latch_reference(has_heading_ref,
                                  heading_ref,
                                  heading_meas,
                                  heading_ref_rad_,
                                  heading_ref_latched_);

    out.has_body_wrench = true;
    out.body_wrench.fill(0.0);
    out.body_wrench[5] = heading_axis_.step(heading_ref, heading_meas, dt_sec);
    return finite(out.body_wrench[5]);
}

DepthHeadingPidController::DepthHeadingPidController(
    const DepthHeadingPidControllerConfig& cfg) noexcept
    : cfg_(cfg),
      depth_axis_(cfg.depth),
      heading_axis_(cfg.heading)
{
    reset();
}

ControlMode DepthHeadingPidController::mode() const noexcept
{
    return ControlMode::kAuto;
}

void DepthHeadingPidController::reset() noexcept
{
    depth_axis_.reset();
    heading_axis_.reset();
    depth_ref_m_ = 0.0;
    heading_ref_rad_ = 0.0;
    depth_ref_latched_ = false;
    heading_ref_latched_ = false;
}

bool DepthHeadingPidController::compute(const ControlState&     state,
                                        const ControlReference& ref,
                                        ControlOutput&          out,
                                        double                  dt_sec) noexcept
{
    out = ControlOutput{};

    if (!nav_ready(state, cfg_.require_nav)) return false;

    double depth_meas = 0.0;
    double heading_meas = 0.0;
    if (!extract_depth_measurement(state, cfg_.require_nav, depth_meas)) return false;
    if (!extract_heading_measurement(state, cfg_.require_nav, heading_meas)) return false;

    double depth_ref = 0.0;
    const bool has_depth_ref = extract_depth_reference(ref, depth_ref);
    depth_ref = latch_reference(has_depth_ref,
                                depth_ref,
                                depth_meas,
                                depth_ref_m_,
                                depth_ref_latched_);

    double heading_ref = 0.0;
    const bool has_heading_ref = extract_heading_reference(ref, heading_ref);
    heading_ref = latch_reference(has_heading_ref,
                                  heading_ref,
                                  heading_meas,
                                  heading_ref_rad_,
                                  heading_ref_latched_);

    out.has_body_wrench = true;
    out.body_wrench.fill(0.0);
    out.body_wrench[2] = depth_axis_.step(depth_ref, depth_meas, dt_sec);
    out.body_wrench[5] = heading_axis_.step(heading_ref, heading_meas, dt_sec);
    return finite(out.body_wrench[2]) && finite(out.body_wrench[5]);
}

PidController::PidController(const DepthHeadingPidControllerConfig& cfg) noexcept
    : impl_(cfg)
{}

ControlMode PidController::mode() const noexcept
{
    return impl_.mode();
}

void PidController::reset() noexcept
{
    impl_.reset();
}

bool PidController::compute(const ControlState&     state,
                            const ControlReference& ref,
                            ControlOutput&          out,
                            double                  dt_sec) noexcept
{
    return impl_.compute(state, ref, out, dt_sec);
}

} // namespace rovctrl::controllers
