#pragma once

namespace arms_control {

/**
 * Discrete PID controller with anti-windup (integral clamping).
 */
class PIDController {
public:
  PIDController(double kp, double ki, double kd, double output_limit);

  /** Compute control output for given error and time step [s]. */
  double compute(double error, double dt);

  /** 런타임 게인 변경 (재빌드 없이 ros2 param set 으로). */
  void set_gains(double kp, double ki, double kd, double output_limit) {
    kp_ = kp; ki_ = ki; kd_ = kd; output_limit_ = output_limit < 0 ? -output_limit : output_limit;
  }

  void reset();

private:
  double kp_, ki_, kd_;
  double output_limit_;

  double integral_{0.0};
  double prev_error_{0.0};
  bool first_call_{true};
};

}  // namespace arms_control
