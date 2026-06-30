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

  /** 미분항 저역통과 계수(0~1). 작을수록 부드러움(노이즈 억제). */
  void set_deriv_alpha(double a) { deriv_alpha_ = (a < 0.01 ? 0.01 : (a > 1.0 ? 1.0 : a)); }

  void reset();

private:
  double kp_, ki_, kd_;
  double output_limit_;

  double integral_{0.0};
  double prev_error_{0.0};
  bool first_call_{true};
  double d_filt_{0.0};        // 저역통과된 미분값 (노이즈 억제)
  double deriv_alpha_{0.25};  // 미분 LPF 계수
};

}  // namespace arms_control
