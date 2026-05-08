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

  void reset();

private:
  double kp_, ki_, kd_;
  double output_limit_;

  double integral_{0.0};
  double prev_error_{0.0};
  bool first_call_{true};
};

}  // namespace arms_control
