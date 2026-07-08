#include "arms_control/pid_controller.hpp"

#include <algorithm>

namespace arms_control {

PIDController::PIDController(double kp, double ki, double kd, double output_limit)
: kp_(kp), ki_(ki), kd_(kd), output_limit_(std::abs(output_limit))
{}

double PIDController::compute(double error, double dt)
{
  if (dt <= 0.0) return 0.0;

  // Proportional
  double p = kp_ * error;

  // Integral with anti-windup clamping (frozen during detection hold)
  if (!integral_frozen_) {
    integral_ += error * dt;
  }
  double i_raw = ki_ * integral_;
  double i_clamped = std::clamp(i_raw, -output_limit_, output_limit_);
  if (ki_ != 0.0) {
    integral_ = i_clamped / ki_;
  }

  // Derivative (skip on first call to avoid derivative kick) + 저역통과 필터
  double d = 0.0;
  if (!first_call_) {
    double d_raw = (error - prev_error_) / dt;
    d_filt_ = deriv_alpha_ * d_raw + (1.0 - deriv_alpha_) * d_filt_;
    d = kd_ * d_filt_;
  }
  first_call_ = false;
  prev_error_ = error;

  double output = p + i_clamped + d;
  return std::clamp(output, -output_limit_, output_limit_);
}

void PIDController::reset()
{
  integral_   = 0.0;
  prev_error_ = 0.0;
  first_call_ = true;
  d_filt_     = 0.0;
}

}  // namespace arms_control
