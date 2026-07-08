#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "arms_control/crsf_output.hpp"
#include "arms_control/pid_controller.hpp"
#include "arms_control/state_machine.hpp"
#include "arms_msgs/msg/detection_array.hpp"
#include "arms_msgs/msg/mission_state.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

namespace arms_control {

class ArmsControlNode : public rclcpp::Node {
 public:
  ArmsControlNode() : Node("arms_control_node") {
    // ----------------------------------------------------------------
    // Declare & load parameters
    // ----------------------------------------------------------------
    declare_parameter("mission.detection_confidence_threshold", 0.65);
    declare_parameter("mission.lock_duration_sec", 2.0);
    declare_parameter("mission.lost_frames_threshold", 10);
    declare_parameter("mission.lock_box_tolerance", 0.15);
    declare_parameter("mission.fire_distance_m", 5.0);

    declare_parameter("control.roll_pid.kp", 15.0);
    declare_parameter("control.roll_pid.ki", 0.5);
    declare_parameter("control.roll_pid.kd", 1.0);
    declare_parameter("control.roll_pid.output_limit", 90.0);
    declare_parameter("control.pitch_pid.kp", 15.0);
    declare_parameter("control.pitch_pid.ki", 0.5);
    declare_parameter("control.pitch_pid.kd", 1.0);
    declare_parameter("control.pitch_pid.output_limit", 90.0);
    declare_parameter("control.throttle", 0.55);
    declare_parameter("control.track_throttle", 0.60);
    declare_parameter("control.roll_sign", 1.0);
    declare_parameter("control.pitch_sign", 1.0);
    declare_parameter("control.error_lpf_alpha", 0.3);
    declare_parameter("control.err_sched_enable", true);
    declare_parameter("control.err_sched_full_err", 0.06);
    declare_parameter("control.err_sched_big_err", 0.35);
    declare_parameter("control.err_sched_min_ratio", 0.55);
    declare_parameter("control.control_rate_hz", 30.0);

    declare_parameter("control.kp_start", 60.0);
    declare_parameter("control.kp_max", 150.0);
    declare_parameter("control.kp_ramp_sec", 5.0);
    declare_parameter("control.deadzone", 0.04);
    declare_parameter("control.deriv_lpf_alpha", 0.25);
    declare_parameter("control.gain_sched_near_m", 4.0);
    declare_parameter("control.gain_sched_min_ratio", 0.35);

    declare_parameter("mission.sitl_auto_launch", false);
    declare_parameter("mission.auto_launch_delay_sec", 1.0);
    declare_parameter("mission.auto_arm_delay_sec", 5.0);

    declare_parameter("crsf.port", std::string("/tmp/crsf_tx"));
    declare_parameter("crsf.max_angle_deg", 35.0);

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    SMParams sm_params;
    sm_params.confidence_threshold =
        get_parameter("mission.detection_confidence_threshold").as_double();
    sm_params.lock_duration_sec =
        get_parameter("mission.lock_duration_sec").as_double();
    sm_params.lost_frames_threshold =
        get_parameter("mission.lost_frames_threshold").as_int();
    sm_params.lock_box_tolerance =
        get_parameter("mission.lock_box_tolerance").as_double();
    sm_params.fire_distance_m =
        get_parameter("mission.fire_distance_m").as_double();

    auto log_fn = [this](const std::string& msg) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    };
    sm_ = std::make_unique<StateMachine>(sm_params, log_fn);

    // ----------------------------------------------------------------
    // PID controllers
    // ----------------------------------------------------------------
    pid_roll_ = std::make_unique<PIDController>(
        get_parameter("control.roll_pid.kp").as_double(),
        get_parameter("control.roll_pid.ki").as_double(),
        get_parameter("control.roll_pid.kd").as_double(),
        get_parameter("control.roll_pid.output_limit").as_double());

    pid_pitch_ = std::make_unique<PIDController>(
        get_parameter("control.pitch_pid.kp").as_double(),
        get_parameter("control.pitch_pid.ki").as_double(),
        get_parameter("control.pitch_pid.kd").as_double(),
        get_parameter("control.pitch_pid.output_limit").as_double());

    rkp_ = get_parameter("control.roll_pid.kp").as_double();
    rki_ = get_parameter("control.roll_pid.ki").as_double();
    rkd_ = get_parameter("control.roll_pid.kd").as_double();
    rlim_ = get_parameter("control.roll_pid.output_limit").as_double();
    pkp_ = get_parameter("control.pitch_pid.kp").as_double();
    pki_ = get_parameter("control.pitch_pid.ki").as_double();
    pkd_ = get_parameter("control.pitch_pid.kd").as_double();
    plim_ = get_parameter("control.pitch_pid.output_limit").as_double();

    throttle_ = get_parameter("control.throttle").as_double();
    track_throttle_ = get_parameter("control.track_throttle").as_double();
    roll_sign_ = get_parameter("control.roll_sign").as_double();
    pitch_sign_ = get_parameter("control.pitch_sign").as_double();
    error_lpf_alpha_ = get_parameter("control.error_lpf_alpha").as_double();
    err_sched_enable_ = get_parameter("control.err_sched_enable").as_bool();
    err_sched_full_err_ =
        get_parameter("control.err_sched_full_err").as_double();
    err_sched_big_err_ = get_parameter("control.err_sched_big_err").as_double();
    err_sched_min_ratio_ =
        get_parameter("control.err_sched_min_ratio").as_double();
    control_rate_hz_ = get_parameter("control.control_rate_hz").as_double();

    kp_start_ = get_parameter("control.kp_start").as_double();
    kp_max_ = get_parameter("control.kp_max").as_double();
    kp_ramp_sec_ = get_parameter("control.kp_ramp_sec").as_double();
    deadzone_ = get_parameter("control.deadzone").as_double();
    deriv_lpf_alpha_ = get_parameter("control.deriv_lpf_alpha").as_double();
    gain_sched_near_m_ = get_parameter("control.gain_sched_near_m").as_double();
    gain_sched_min_ratio_ =
        get_parameter("control.gain_sched_min_ratio").as_double();
    pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
    pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);

    sitl_auto_launch_ = get_parameter("mission.sitl_auto_launch").as_bool();
    auto_launch_delay_sec_ =
        get_parameter("mission.auto_launch_delay_sec").as_double();
    auto_arm_delay_sec_ = get_parameter("mission.auto_arm_delay_sec").as_double();

    crsf_max_angle_ = get_parameter("crsf.max_angle_deg").as_double();
    crsf_out_ = std::make_unique<CrsfOutput>(get_parameter("crsf.port").as_string());

    // 런타임 파라미터 변경 콜백
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          bool pid_changed = false;
          for (const auto& p : params) {
            const std::string& n = p.get_name();
            if (n == "control.roll_sign")
              roll_sign_ = p.as_double();
            else if (n == "control.pitch_sign")
              pitch_sign_ = p.as_double();
            else if (n == "control.track_throttle")
              track_throttle_ = p.as_double();
            else if (n == "control.throttle")
              throttle_ = p.as_double();
            else if (n == "control.error_lpf_alpha")
              error_lpf_alpha_ = p.as_double();
            else if (n == "control.kp_start")
              kp_start_ = p.as_double();
            else if (n == "control.kp_max")
              kp_max_ = p.as_double();
            else if (n == "control.kp_ramp_sec")
              kp_ramp_sec_ = p.as_double();
            else if (n == "control.deadzone")
              deadzone_ = p.as_double();
            else if (n == "control.gain_sched_near_m")
              gain_sched_near_m_ = p.as_double();
            else if (n == "control.gain_sched_min_ratio")
              gain_sched_min_ratio_ = p.as_double();
            else if (n == "control.deriv_lpf_alpha") {
              deriv_lpf_alpha_ = p.as_double();
              pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
              pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);
            } else if (n == "control.roll_pid.kp") {
              rkp_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.roll_pid.ki") {
              rki_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.roll_pid.kd") {
              rkd_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.roll_pid.output_limit") {
              rlim_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.kp") {
              pkp_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.ki") {
              pki_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.kd") {
              pkd_ = p.as_double();
              pid_changed = true;
            } else if (n == "control.pitch_pid.output_limit") {
              plim_ = p.as_double();
              pid_changed = true;
            }
          }
          if (pid_changed) {
            pid_roll_->set_gains(rkp_, rki_, rkd_, rlim_);
            pid_pitch_->set_gains(pkp_, pki_, pkd_, plim_);
          }
          RCLCPP_INFO(get_logger(),
                      "param 적용: roll_sign=%.0f pitch_sign=%.0f rkp=%.1f "
                      "rkd=%.1f track_thr=%.2f",
                      roll_sign_, pitch_sign_, rkp_, rkd_, track_throttle_);
          return res;
        });

    // ----------------------------------------------------------------
    // ROS publishers
    // ----------------------------------------------------------------
    pub_state_ = create_publisher<arms_msgs::msg::MissionState>("/arms/mission_state", 10);
    pub_dbg_ = create_publisher<geometry_msgs::msg::Vector3>("/arms/control_debug", 10);

    // ----------------------------------------------------------------
    // ROS subscribers
    // ----------------------------------------------------------------
    auto best_effort_qos = rclcpp::QoS(1).best_effort();

    sub_detections_ = create_subscription<arms_msgs::msg::DetectionArray>(
        "/arms/detections", best_effort_qos,
        [this](arms_msgs::msg::DetectionArray::SharedPtr msg) {
          sm_->on_detection(msg->detections);
        });

    sub_distance_ = create_subscription<sensor_msgs::msg::Range>(
        "/arms/distance", best_effort_qos,
        [this](sensor_msgs::msg::Range::SharedPtr msg) {
          cache_distance(msg->range);
          sm_->on_distance(msg->range);
        });

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/arms/scan_raw", best_effort_qos,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          float best = std::numeric_limits<float>::infinity();
          for (float r : msg->ranges) {
            if (r >= msg->range_min && r <= msg->range_max && r < best) {
              best = r;
            }
          }
          if (std::isfinite(best)) {
            cache_distance(best);
            sm_->on_distance(best);
          }
        });

    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) {
          for (size_t i = 0; i < 4 && i < msg->axes.size(); ++i)
            joy_axes_[i] = msg->axes[i];

          auto btn = [&](int i) -> int {
            return (static_cast<size_t>(i) < msg->buttons.size()) ? msg->buttons[i] : 0;
          };
          int kill = btn(0), land = btn(1), mode = btn(2), launch = btn(3);

          if (mode && !prev_btn_[2]) {
            joy_manual_mode_ = !joy_manual_mode_;
            RCLCPP_INFO(get_logger(), "Mode: %s", joy_manual_mode_ ? "MANUAL" : "AUTO");
          }
          if (launch && !prev_btn_[3]) {
            sm_->on_launch_button();
          }

          joy_kill_   = static_cast<bool>(kill);
          joy_land_   = static_cast<bool>(land);
          joy_launch_ = static_cast<bool>(launch);
          prev_btn_[0] = kill; prev_btn_[1] = land;
          prev_btn_[2] = mode; prev_btn_[3] = launch;
        });

    sub_reset_ = create_subscription<std_msgs::msg::Empty>(
        "/arms/reset_cmd", 10, [this](std_msgs::msg::Empty::SharedPtr) {
          RCLCPP_WARN(get_logger(), "RESET command received -> force SEARCH.");
          sm_->force_search();
          rtl_sent_ = false;
          fire_sent_ = false;
          pid_roll_->reset();
          pid_pitch_->reset();
          filt_err_x_ = 0.0;
          filt_err_y_ = 0.0;
        });

    // ----------------------------------------------------------------
    // Control loop timer
    // ----------------------------------------------------------------
    last_tick_ = now();
    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ArmsControlNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "arms_control_node started.");
  }

 private:
  // ----------------------------------------------------------------
  // Helpers
  // ----------------------------------------------------------------
  void cache_distance(double d) {
    if (std::isfinite(d) && d > 0.0) {
      last_distance_ = d;
      last_distance_time_ = now();
    }
  }

  bool distance_valid() {
    if (last_distance_ <= 0.0) return false;
    return (now() - last_distance_time_).seconds() < 0.5;
  }

  static double apply_deadzone(double e, double dz) {
    if (dz <= 0.0) return e;
    if (e > dz) return e - dz;
    if (e < -dz) return e + dz;
    return 0.0;
  }

  // ----------------------------------------------------------------
  // 30 Hz control loop
  // ----------------------------------------------------------------
  void control_loop() {
    auto now_t = now();
    double dt = (now_t - last_tick_).seconds();
    last_tick_ = now_t;

    State state = sm_->state();

    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    float thrust = 0.f;

    // ---- TRACK 진입 감지: P 램프 타이머 리셋 ----
    if (state == State::TRACK && prev_state_ != State::TRACK &&
        prev_state_ != State::FIRE) {
      track_enter_time_ = now_t;
      RCLCPP_INFO(get_logger(), "TRACK 진입 → P 램프 시작 (%.1f→%.1f, %.1fs)",
                  kp_start_, kp_max_, kp_ramp_sec_);
    }
    prev_state_ = state;

    if (state == State::TRACK || state == State::FIRE) {
      double raw_ex = sm_->current_error_x();
      double raw_ey = sm_->current_error_y();
      filt_err_x_ =
          error_lpf_alpha_ * raw_ex + (1.0 - error_lpf_alpha_) * filt_err_x_;
      filt_err_y_ =
          error_lpf_alpha_ * raw_ey + (1.0 - error_lpf_alpha_) * filt_err_y_;

      // ---- 시간 기반 P 램프 ----
      double elapsed = (now_t - track_enter_time_).seconds();
      double ramp_t = (kp_ramp_sec_ > 1e-3)
                          ? std::clamp(elapsed / kp_ramp_sec_, 0.0, 1.0)
                          : 1.0;
      kp_now_ = kp_start_ + ramp_t * (kp_max_ - kp_start_);

      // ---- 거리 게인 스케줄링 ----
      double ratio = 1.0;
      if (distance_valid() && gain_sched_near_m_ > 1e-3 &&
          gain_sched_min_ratio_ < 0.999) {
        double tt = std::clamp(last_distance_ / gain_sched_near_m_, 0.0, 1.0);
        ratio = gain_sched_min_ratio_ + tt * (1.0 - gain_sched_min_ratio_);
      }
      gain_ratio_filt_ = 0.15 * ratio + 0.85 * gain_ratio_filt_;
      double kp_eff = kp_now_ * gain_ratio_filt_;

      // ---- err 기반 P 자동조절 ----
      if (err_sched_enable_) {
        double err_mag = std::hypot(filt_err_x_, filt_err_y_);
        double er = 1.0;
        if (err_sched_big_err_ > err_sched_full_err_ + 1e-6) {
          double u = std::clamp((err_mag - err_sched_full_err_) /
                                    (err_sched_big_err_ - err_sched_full_err_),
                                0.0, 1.0);
          er = 1.0 - u * (1.0 - err_sched_min_ratio_);
        }
        err_ratio_filt_ = 0.2 * er + 0.8 * err_ratio_filt_;
        kp_eff *= err_ratio_filt_;
      }

      pid_roll_->set_gains(kp_eff, rki_, rkd_, rlim_);
      const double pitch_gain_mul = 1.6;
      pid_pitch_->set_gains(kp_eff * pitch_gain_mul, pki_, pkd_, plim_);

      // ---- 중앙 데드존 ----
      double ex = apply_deadzone(filt_err_x_, deadzone_);
      double ey = apply_deadzone(filt_err_y_, deadzone_);

      roll_deg = roll_sign_ * pid_roll_->compute(ex, dt);
      pitch_deg = pitch_sign_ * pid_pitch_->compute(ey, dt);

      // ---- 정렬 게이트 + 라이다 거리제어 ----
      {
        const double up = track_throttle_;
        const double hover = 0.62;
        const double fire_d = 5.0;
        const double align_thr = 0.10;
        double emag = std::hypot(filt_err_x_, filt_err_y_);

        if (!align_locked_ && emag < align_thr) {
          align_locked_ = true;
          RCLCPP_INFO(get_logger(), "정렬 완료 (오차 %.2f) → 상승 요격 시작",
                      emag);
        }

        if (!align_locked_) {
          thrust = static_cast<float>(hover);
        } else {
          const double xy_gate = 0.22;
          double emag_now = std::hypot(filt_err_x_, filt_err_y_);
          if (emag_now > xy_gate) {
            thrust = static_cast<float>(hover);
          } else if (distance_valid()) {
            double d = last_distance_;
            if (d > fire_d + 3.0) {
              thrust = static_cast<float>(up);
            } else if (d > fire_d) {
              double t = (d - fire_d) / 3.0;
              thrust = static_cast<float>(hover + (up - hover) * t);
            } else {
              thrust = static_cast<float>(hover);
            }
          } else {
            thrust = static_cast<float>(up);
          }
        }
      }

      if (++dbg_count_ % 6 == 0) {
        RCLCPP_INFO(get_logger(),
                    "TRACK err=(%.2f,%.2f) kp=%.1f(errx%.2f) d=%.1fm -> "
                    "roll=%.1f pitch=%.1f",
                    filt_err_x_, filt_err_y_, kp_eff, err_ratio_filt_,
                    distance_valid() ? last_distance_ : -1.0, roll_deg,
                    pitch_deg);
      }
    } else {
      // IDLE / SEARCH / LOCK / RTL
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      kp_now_ = kp_start_;
      thrust = 0.f;
      align_locked_ = false;
    }

    // ---- Auto arm: IDLE → SEARCH after delay (auto mode only) ----
    if (state == State::IDLE && !joy_manual_mode_ && !joy_kill_) {
      if (!arm_timer_started_) {
        arm_timer_started_ = true;
        arm_enter_time_ = now_t;
      } else if (!auto_armed_ &&
                 (now_t - arm_enter_time_).seconds() >= auto_arm_delay_sec_) {
        auto_armed_ = true;
        sm_->arm();
        RCLCPP_INFO(get_logger(), "Auto arm: IDLE → SEARCH");
      }
    } else if (state != State::IDLE) {
      arm_timer_started_ = false;
    }

    // ---- SITL auto-launch ----
    if (sitl_auto_launch_) {
      if (state == State::LOCK) {
        if (!lock_timer_started_) {
          lock_enter_time_ = now_t;
          lock_timer_started_ = true;
        } else if (!auto_launched_ && (now_t - lock_enter_time_).seconds() >=
                                          auto_launch_delay_sec_) {
          auto_launched_ = true;
          RCLCPP_INFO(get_logger(), "SITL auto-launch: LOCK -> TRACK");
          sm_->on_launch_button();
        }
      } else if (state == State::IDLE || state == State::SEARCH) {
        lock_timer_started_ = false;
        auto_launched_ = false;
      }
    }

    // ---- CRSF output ----
    {
      CrsfOutput::Channels crsf{};
      crsf.fill(CrsfOutput::CRSF_MIN);

      if (joy_manual_mode_) {
        crsf[0] = CrsfOutput::norm_to_crsf(joy_axes_[0]);
        crsf[1] = CrsfOutput::norm_to_crsf(joy_axes_[1]);
        crsf[2] = CrsfOutput::norm_to_crsf(joy_axes_[2]);
        crsf[3] = CrsfOutput::norm_to_crsf(joy_axes_[3]);
      } else {
        crsf[0] = CrsfOutput::norm_to_crsf(roll_deg / crsf_max_angle_);
        crsf[1] = CrsfOutput::norm_to_crsf(pitch_deg / crsf_max_angle_);
        crsf[2] = CrsfOutput::thr_to_crsf(static_cast<double>(thrust));
        crsf[3] = CrsfOutput::CRSF_CENTER;  // yaw hold
      }

      // CH5: arm (IDLE → disarmed, else armed)
      crsf[4] = (state == State::IDLE) ? CrsfOutput::CRSF_MIN : CrsfOutput::CRSF_MAX;

      // CH6: land switch (RTL state or manual land button)
      crsf[5] = (state == State::RTL || joy_land_) ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH7: kill switch
      crsf[6] = joy_kill_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH8: launch/fire — hold high for 1s on FIRE, or follow manual button
      if (state == State::FIRE && !fire_sent_) {
        fire_hold_ticks_ = static_cast<int>(control_rate_hz_);
      }
      if (joy_manual_mode_) {
        crsf[7] = joy_launch_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;
      } else {
        crsf[7] = (fire_hold_ticks_ > 0) ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;
      }
      if (fire_hold_ticks_ > 0) --fire_hold_ticks_;

      crsf_out_->send(crsf);
    }

    // ---- Debug publish ----
    geometry_msgs::msg::Vector3 dbg;
    dbg.x = roll_deg;
    dbg.y = pitch_deg;
    dbg.z = thrust;
    pub_dbg_->publish(dbg);

    // ---- FIRE one-shot ----
    if (state == State::FIRE && !fire_sent_) {
      fire_sent_ = true;
      RCLCPP_INFO(get_logger(), "FIRE state — payload trigger signaled via mission_state.");
      sm_->on_fire_complete();
    }

    // ---- RTL one-shot (CH6 land switch handled by sitl_bridge_node) ----
    if (state == State::RTL && !rtl_sent_) {
      rtl_sent_ = true;
      RCLCPP_INFO(get_logger(), "RTL state — CH6 land switch activated.");
    }

    if (state == State::IDLE) {
      fire_sent_ = false;
      rtl_sent_ = false;
    }

    // ---- Publish mission state ----
    arms_msgs::msg::MissionState msg;
    msg.header.stamp = now_t;
    msg.state = to_string(state);
    msg.lock_elapsed_sec = static_cast<float>(sm_->lock_elapsed_sec());
    msg.error_x = static_cast<float>(sm_->current_error_x());
    msg.error_y = static_cast<float>(sm_->current_error_y());
    msg.target_locked = sm_->target_locked();
    msg.kp_now = static_cast<float>(kp_now_);
    pub_state_->publish(msg);
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<PIDController> pid_roll_;
  std::unique_ptr<PIDController> pid_pitch_;

  double throttle_{0.55};
  double track_throttle_{0.60};
  double roll_sign_{1.0};
  double pitch_sign_{1.0};
  double error_lpf_alpha_{0.3};
  double filt_err_x_{0.0};
  double filt_err_y_{0.0};
  double kp_start_{60.0};
  double kp_max_{150.0};
  double kp_ramp_sec_{5.0};
  double kp_now_{60.0};
  double deadzone_{0.04};
  double deriv_lpf_alpha_{0.25};
  double gain_sched_near_m_{4.0};
  double gain_sched_min_ratio_{0.35};
  double gain_ratio_filt_{1.0};
  bool err_sched_enable_{true};
  double err_sched_full_err_{0.06};
  double err_sched_big_err_{0.35};
  double err_sched_min_ratio_{0.30};
  double err_ratio_filt_{1.0};
  double last_distance_{0.0};
  rclcpp::Time last_distance_time_;
  rclcpp::Time track_enter_time_;
  int dbg_count_{0};
  double control_rate_hz_{30.0};

  State prev_state_{State::IDLE};

  bool align_locked_{false};

  bool sitl_auto_launch_{false};
  double auto_launch_delay_sec_{1.0};
  bool lock_timer_started_{false};
  bool auto_launched_{false};
  rclcpp::Time lock_enter_time_;

  // Auto arm
  double auto_arm_delay_sec_{5.0};
  bool arm_timer_started_{false};
  bool auto_armed_{false};
  rclcpp::Time arm_enter_time_;

  // Joy state
  std::array<float, 4> joy_axes_{};
  bool joy_kill_{false};
  bool joy_land_{false};
  bool joy_launch_{false};
  bool joy_manual_mode_{false};
  std::array<int, 4> prev_btn_{};

  // CRSF output
  std::unique_ptr<CrsfOutput> crsf_out_;
  double crsf_max_angle_{35.0};
  int fire_hold_ticks_{0};

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_dbg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double rkp_{0}, rki_{0}, rkd_{0}, rlim_{0};
  double pkp_{0}, pki_{0}, pkd_{0}, plim_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  bool fire_sent_{false};
  bool rtl_sent_{false};
};

}  // namespace arms_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_control::ArmsControlNode>());
  rclcpp::shutdown();
  return 0;
}
