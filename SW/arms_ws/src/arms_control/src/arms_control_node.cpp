#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
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
#include "std_msgs/msg/float64.hpp"

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
    declare_parameter("mission.detection_timeout_sec", 1.0);
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
    declare_parameter("control.lead_gain", 0.0);
    declare_parameter("control.roll_sign", 1.0);
    declare_parameter("control.pitch_sign", 1.0);
    declare_parameter("control.error_lpf_alpha", 0.3);
    declare_parameter("control.control_rate_hz", 30.0);
    declare_parameter("control.deadzone", 0.04);
    declare_parameter("control.deriv_lpf_alpha", 0.25);

    declare_parameter("mission.sitl_auto_launch", false);
    declare_parameter("mission.auto_launch_delay_sec", 1.0);

    declare_parameter("crsf.port", std::string("/tmp/crsf_tx"));
    declare_parameter("crsf.baud", 400000);
    declare_parameter("crsf.max_angle_deg", 35.0);

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    SMParams sm_params;
    sm_params.confidence_threshold =
        get_parameter("mission.detection_confidence_threshold").as_double();
    sm_params.lock_duration_sec =
        get_parameter("mission.lock_duration_sec").as_double();
    sm_params.detection_timeout_sec =
        get_parameter("mission.detection_timeout_sec").as_double();
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
    lead_gain_ = get_parameter("control.lead_gain").as_double();
    roll_sign_ = get_parameter("control.roll_sign").as_double();
    pitch_sign_ = get_parameter("control.pitch_sign").as_double();
    error_lpf_alpha_ = get_parameter("control.error_lpf_alpha").as_double();
    control_rate_hz_ = get_parameter("control.control_rate_hz").as_double();
    deadzone_ = get_parameter("control.deadzone").as_double();
    deriv_lpf_alpha_ = get_parameter("control.deriv_lpf_alpha").as_double();
    pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
    pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);

    sitl_auto_launch_ = get_parameter("mission.sitl_auto_launch").as_bool();
    auto_launch_delay_sec_ =
        get_parameter("mission.auto_launch_delay_sec").as_double();

    crsf_max_angle_ = get_parameter("crsf.max_angle_deg").as_double();
    crsf_out_ = std::make_unique<CrsfOutput>(
        get_parameter("crsf.port").as_string(),
        static_cast<int>(get_parameter("crsf.baud").as_int()));

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
            else if (n == "control.lead_gain")
              lead_gain_ = p.as_double();
            else if (n == "control.throttle")
              throttle_ = p.as_double();
            else if (n == "control.error_lpf_alpha")
              error_lpf_alpha_ = p.as_double();
            else if (n == "control.deadzone")
              deadzone_ = p.as_double();
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
    pub_dbg_  = create_publisher<geometry_msgs::msg::Vector3>("/arms/control_debug", 10);
    pub_dist_ = create_publisher<std_msgs::msg::Float64>("/arms/debug_distance", 10);

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

    // BEST_EFFORT 로 구독: uart 노드(SensorDataQoS=best_effort)와 GUI(reliable)
    // 양쪽 발행자 모두와 호환된다. (reliable 구독은 best_effort 발행을 못 받음)
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/arms/command", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Joy::SharedPtr msg) {
          for (size_t i = 0; i < 4 && i < msg->axes.size(); ++i)
            joy_axes_[i] = msg->axes[i];

          auto btn = [&](int i) -> int {
            return (static_cast<size_t>(i) < msg->buttons.size()) ? msg->buttons[i] : 0;
          };
          int kill = btn(0), arm = btn(1), mode = btn(2), launch = btn(3);

          // MODE: 레벨 스위치. 0=auto(영상유도, PX4 Manual), 1=manual(손제어, PX4 Altitude)
          bool new_manual = static_cast<bool>(mode);
          if (new_manual != joy_manual_mode_) {
            joy_manual_mode_ = new_manual;
            RCLCPP_INFO(get_logger(), "Mode: %s",
                        joy_manual_mode_ ? "MANUAL (Altitude)" : "AUTO (Manual)");
          }
          if (launch && !prev_btn_[3]) {
            sm_->on_launch_button();
          }

          joy_kill_ = static_cast<bool>(kill);
          joy_arm_  = static_cast<bool>(arm);
          prev_btn_[0] = kill; prev_btn_[1] = arm;
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

  /* 수신된 CRSF 텔레메트리와 통신 오류를 시험용 로그로 출력한다. */
  void handle_crsf_input() {
    const auto result = crsf_out_->receive();                     // UART에 쌓인 응답을 모두 가져온다.

    crsf_rx_bytes_ += result.bytes_read;                          // 누적 수신량을 갱신한다.
    crsf_rx_echoes_ += result.echo_frames;                        // 자체 송신 에코 수를 갱신한다.
    crsf_rx_crc_errors_ += result.crc_errors;                     // CRC 오류 수를 갱신한다.
    crsf_rx_framing_errors_ += result.framing_errors;             // 프레임 길이 오류 수를 갱신한다.
    crsf_rx_valid_frames_ += result.frames.size();                // 정상 텔레메트리 수를 갱신한다.

    if (result.bytes_read > 0 || result.crc_errors > 0 ||
        result.framing_errors > 0)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CRSF RX: bytes=%zu valid=%zu echo=%zu crc_err=%zu frame_err=%zu",
        crsf_rx_bytes_, crsf_rx_valid_frames_, crsf_rx_echoes_,
        crsf_rx_crc_errors_, crsf_rx_framing_errors_);            // 2초마다 누적 통계를 보여준다.
    }

    for (const auto & frame : result.frames) {
      if (!crsf_seen_types_[frame.type]) {
        crsf_seen_types_[frame.type] = true;                      // 타입별 첫 수신 여부를 기록한다.
        RCLCPP_INFO(
          get_logger(), "CRSF telemetry detected: addr=0x%02X type=0x%02X payload=%zu",
          static_cast<unsigned int>(frame.address),
          static_cast<unsigned int>(frame.type), frame.payload.size());   // 새 텔레메트리 타입을 한 번 알린다.
      }

      if (frame.type == 0x14 && frame.payload.size() >= 10) {
        const int up_rssi_1 = -static_cast<int>(frame.payload[0]);         // 상향 안테나 1 RSSI를 dBm으로 바꾼다.
        const int up_rssi_2 = -static_cast<int>(frame.payload[1]);         // 상향 안테나 2 RSSI를 dBm으로 바꾼다.
        const int up_lq = static_cast<int>(frame.payload[2]);              // 상향 링크 품질을 백분율로 읽는다.
        const int up_snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // 상향 SNR의 부호를 복원한다.
        const int down_rssi = -static_cast<int>(frame.payload[7]);         // 하향 RSSI를 dBm으로 바꾼다.
        const int down_lq = static_cast<int>(frame.payload[8]);            // 하향 링크 품질을 백분율로 읽는다.
        const int down_snr = static_cast<int>(static_cast<int8_t>(frame.payload[9]));   // 하향 SNR의 부호를 복원한다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ELRS link: up_rssi=(%d,%d)dBm up_lq=%d%% up_snr=%ddB "
          "down_rssi=%ddBm down_lq=%d%% down_snr=%ddB",
          up_rssi_1, up_rssi_2, up_lq, up_snr,
          down_rssi, down_lq, down_snr);                          // 링크 상태를 1초마다 보여준다.
      } else if ((frame.type == 0x1C || frame.type == 0x1D) &&
                 frame.payload.size() >= 5)
      {
        const int rssi = -static_cast<int>(frame.payload[0]);             // 단일 링크 RSSI를 dBm으로 바꾼다.
        const int rssi_percent = static_cast<int>(frame.payload[1]);       // RSSI 백분율을 읽는다.
        const int link_quality = static_cast<int>(frame.payload[2]);       // 링크 품질을 백분율로 읽는다.
        const int snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // SNR의 부호를 복원한다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ELRS %s link: rssi=%ddBm rssi_pct=%d%% lq=%d%% snr=%ddB",
          frame.type == 0x1C ? "RX" : "TX",
          rssi, rssi_percent, link_quality, snr);                 // 분리형 링크 상태를 1초마다 보여준다.
      }
    }
  }

  // ----------------------------------------------------------------
  // 30 Hz control loop
  // ----------------------------------------------------------------
  void control_loop() {
    handle_crsf_input();                                         // 직전 송신 이후 도착한 텔레메트리를 처리한다.

    auto now_t = now();
    double dt = (now_t - last_tick_).seconds();
    last_tick_ = now_t;

    State state = sm_->state();

    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    float thrust = 0.f;

    prev_state_ = state;

    if (state == State::TRACK || state == State::FIRE) {
      double raw_ex = sm_->current_error_x();
      double raw_ey = sm_->current_error_y();
      filt_err_x_ =
          error_lpf_alpha_ * raw_ex + (1.0 - error_lpf_alpha_) * filt_err_x_;
      filt_err_y_ =
          error_lpf_alpha_ * raw_ey + (1.0 - error_lpf_alpha_) * filt_err_y_;

      kp_now_ = rkp_;

      // ---- 중앙 데드존 ----
      double ex = apply_deadzone(filt_err_x_, deadzone_);
      double ey = apply_deadzone(filt_err_y_, deadzone_);
      if (dt > 1e-6) {
        // 생 미분은 탐지 노이즈로 ±1.5까지 튐 → 강한 LPF로 표적 이동 추세만 추출
        const double DOT_A = 0.12;
        double raw_dx = (filt_err_x_ - prev_err_x_) / dt;
        double raw_dy = (filt_err_y_ - prev_err_y_) / dt;
        err_dot_x_ = DOT_A * raw_dx + (1.0 - DOT_A) * err_dot_x_;
        err_dot_y_ = DOT_A * raw_dy + (1.0 - DOT_A) * err_dot_y_;
      }
      prev_err_x_ = filt_err_x_;
      prev_err_y_ = filt_err_y_;

      bool held = sm_->is_detection_held();
      pid_roll_->set_integral_frozen(held);
      pid_pitch_->set_integral_frozen(held);
      double emag_g = std::hypot(ex, ey);
      double gain_scale = 1.0;
      if (emag_g < 0.08)      gain_scale = 0.5;
      else if (emag_g < 0.20) gain_scale = 0.75;
      double ex_lead = ex + lead_gain_ * err_dot_x_;
      double ey_lead = ey + lead_gain_ * err_dot_y_;
      roll_deg = roll_sign_ * gain_scale * pid_roll_->compute(ex_lead, dt);
      double pitch_scale = 1.0;
      if (fabs(ex) > 0.12)      pitch_scale = 0.8;
      else if (fabs(ex) > 0.06) pitch_scale = 0.9;
      pitch_deg = pitch_sign_ * gain_scale * pitch_scale * pid_pitch_->compute(ey_lead, dt);

      // ---- 정렬 게이트 + 라이다 거리제어 ----
      {
        const double up = track_throttle_;
        const double hover = 0.62;
        const double fire_d = 5.0;
        const double align_thr = 0.05;
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
                    "TRACK err=(%.2f,%.2f) edot=(%.2f,%.2f) d=%.1fm roll=%.1f pitch=%.1f",
                    filt_err_x_, filt_err_y_, err_dot_x_, err_dot_y_,
                    distance_valid() ? last_distance_ : -1.0, roll_deg,
                    pitch_deg);
      }
    } else {
      // IDLE / SEARCH / LOCK / RTL
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      kp_now_ = rkp_;
      thrust = 0.f;
      align_locked_ = false;
    }

    // ---- IDLE → SEARCH: 즉시 전이 (auto 모드) ----
    if (state == State::IDLE && !joy_manual_mode_ && !auto_armed_) {
      auto_armed_ = true;
      sm_->arm();
      RCLCPP_INFO(get_logger(), "IDLE → SEARCH");
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
        // Mode2 스틱 → AETR 채널 매핑.
        //   L-stick: X=axes[0]=yaw,      Y=axes[1]=throttle
        //   R-stick: X=axes[2]=roll,     Y=axes[3]=pitch
        crsf[0] = CrsfOutput::norm_to_crsf(joy_axes_[2]);  // CH1 roll  = R-stick X
        crsf[1] = CrsfOutput::norm_to_crsf(joy_axes_[3]);  // CH2 pitch = R-stick Y
        crsf[2] = CrsfOutput::norm_to_crsf(joy_axes_[1]);  // CH3 thr   = L-stick Y (스프링, 중앙=50%)
        crsf[3] = CrsfOutput::norm_to_crsf(joy_axes_[0]);  // CH4 yaw   = L-stick X
      } else {
        crsf[0] = CrsfOutput::norm_to_crsf(roll_deg / crsf_max_angle_);
        crsf[1] = CrsfOutput::norm_to_crsf(pitch_deg / crsf_max_angle_);
        crsf[2] = CrsfOutput::thr_to_crsf(static_cast<double>(thrust));
        crsf[3] = CrsfOutput::CRSF_CENTER;  // yaw hold
      }

      // CH5: arm — auto(영상유도)=상태머신, manual(손제어)=arm 스위치.
      //   auto 모드에서는 arm 스위치 값 무시(상태머신이 LOCK 이상에서 arm).
      bool fc_armed = (state == State::LOCK || state == State::TRACK ||
                       state == State::FIRE  || state == State::RTL);
      if (joy_manual_mode_) {
        crsf[4] = joy_arm_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;
      } else {
        crsf[4] = fc_armed ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;
      }

      // CH6: flight mode — auto=PX4 Manual(172), manual=PX4 Altitude(1811).
      //   모드 스위치(joy_manual_mode_)와 1:1. PX4쪽 RC_MAP_FLTMODE=6 필요.
      crsf[5] = joy_manual_mode_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH7: kill switch (양쪽 모드 공통, FC가 직접 모터 차단)
      crsf[6] = joy_kill_ ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

      // CH8: 미사용 (crsf.fill(CRSF_MIN)으로 172 고정)

      if (!crsf_out_->send(crsf)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CRSF TX failed: check UART port and baud rate.");      // 송신 실패를 과도한 반복 없이 알린다.
      }
    }

    // ---- Debug publish ----
    geometry_msgs::msg::Vector3 dbg;
    dbg.x = roll_deg;
    dbg.y = pitch_deg;
    dbg.z = thrust;
    pub_dbg_->publish(dbg);

    std_msgs::msg::Float64 dist_msg;
    dist_msg.data = distance_valid() ? last_distance_ : -1.0;
    pub_dist_->publish(dist_msg);

    // ---- FIRE one-shot ----
    if (state == State::FIRE && !fire_sent_) {
      fire_sent_ = true;
      RCLCPP_INFO(get_logger(), "FIRE state — payload trigger signaled via mission_state.");
      sm_->on_fire_complete();
    }

    // ---- RTL one-shot ----
    // NOTE: land/RTL 전용 CRSF 채널은 제거됨(CH6는 flight mode). 자동 착륙은
    // 별도 경로(예: manual 모드 Altitude 전환 후 수동 착륙)로 처리 필요.
    if (state == State::RTL && !rtl_sent_) {
      rtl_sent_ = true;
      RCLCPP_INFO(get_logger(), "RTL state — no dedicated land channel (see NOTE).");
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
  double lead_gain_{0.0};
  double roll_sign_{1.0};
  double pitch_sign_{1.0};
  double error_lpf_alpha_{0.3};
  double filt_err_x_{0.0};
  double filt_err_y_{0.0};
  double prev_err_x_{0.0};
  double prev_err_y_{0.0};
  double err_dot_x_{0.0};
  double err_dot_y_{0.0};
  double kp_now_{0.0};
  double deadzone_{0.04};
  double deriv_lpf_alpha_{0.25};
  double last_distance_{0.0};
  rclcpp::Time last_distance_time_;
  int dbg_count_{0};
  double control_rate_hz_{30.0};

  State prev_state_{State::IDLE};

  bool align_locked_{false};

  bool sitl_auto_launch_{false};
  double auto_launch_delay_sec_{1.0};
  bool lock_timer_started_{false};
  bool auto_launched_{false};
  rclcpp::Time lock_enter_time_;

  // Auto arm (IDLE→SEARCH 즉시 1회)
  bool auto_armed_{false};

  // Joy state
  std::array<float, 4> joy_axes_{};
  bool joy_kill_{false};
  bool joy_arm_{false};
  bool joy_manual_mode_{false};
  std::array<int, 4> prev_btn_{};

  // CRSF 송수신 상태
  std::unique_ptr<CrsfOutput> crsf_out_;                          // CRSF UART 송수신기를 소유한다.
  double crsf_max_angle_{35.0};                                  // 자동 제어 각도의 CRSF 정규화 기준을 둔다.
  std::array<bool, 256> crsf_seen_types_{};                       // 처음 발견한 텔레메트리 타입을 구분한다.
  std::size_t crsf_rx_bytes_{0};                                 // 누적 UART 수신량을 센다.
  std::size_t crsf_rx_valid_frames_{0};                          // 정상 텔레메트리 프레임 수를 센다.
  std::size_t crsf_rx_echoes_{0};                                // 자체 송신 에코 프레임 수를 센다.
  std::size_t crsf_rx_crc_errors_{0};                            // CRC 오류 프레임 수를 센다.
  std::size_t crsf_rx_framing_errors_{0};                        // 길이 오류 프레임 수를 센다.

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_dbg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dist_;
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
