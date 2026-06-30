#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "arms_msgs/msg/detection_array.hpp"
#include "arms_msgs/msg/mission_state.hpp"

#include "arms_control/mavlink_interface.hpp"
#include "arms_control/pid_controller.hpp"
#include "arms_control/state_machine.hpp"

using namespace std::chrono_literals;

namespace arms_control {

class ArmsControlNode : public rclcpp::Node {
public:
  ArmsControlNode()
  : Node("arms_control_node")
  {
    // ----------------------------------------------------------------
    // Declare & load parameters
    // ----------------------------------------------------------------
    declare_parameter("mission.detection_confidence_threshold", 0.65);
    declare_parameter("mission.lock_duration_sec",              2.0);
    declare_parameter("mission.lost_frames_threshold",          10);
    declare_parameter("mission.lock_box_tolerance",             0.15);
    declare_parameter("mission.fire_distance_m",                5.0);

    declare_parameter("control.roll_pid.kp",           15.0);
    declare_parameter("control.roll_pid.ki",           0.5);
    declare_parameter("control.roll_pid.kd",           1.0);
    declare_parameter("control.roll_pid.output_limit", 90.0);
    declare_parameter("control.pitch_pid.kp",          15.0);
    declare_parameter("control.pitch_pid.ki",          0.5);
    declare_parameter("control.pitch_pid.kd",          1.0);
    declare_parameter("control.pitch_pid.output_limit",90.0);
    declare_parameter("control.throttle",              0.55);
    declare_parameter("control.track_throttle",        0.60);
    declare_parameter("control.boost_throttle",        0.90);
    declare_parameter("control.boost_kp",              8.0);
    declare_parameter("control.boost_angle_limit",     15.0);
    declare_parameter("control.boost_deviation_thresh",0.25);
    declare_parameter("control.roll_sign",             1.0);
    declare_parameter("control.pitch_sign",            1.0);
    declare_parameter("control.error_lpf_alpha",       0.3);
    // err 기반 P 자동조절: |err| 크면 P 낮춤(발산 방지), 작으면 풀 P(빠른 마무리)
    declare_parameter("control.err_sched_enable",   true);   // 켜고 끄기
    declare_parameter("control.err_sched_full_err", 0.06);   // 이 오차 이하 = 풀 P
    declare_parameter("control.err_sched_big_err",  0.35);   // 이 오차 이상 = 최소 P
    declare_parameter("control.err_sched_min_ratio",0.30);   // 큰 오차일 때 P 비율(30%)
    declare_parameter("control.control_rate_hz",       30.0);

    // ---- 시간 기반 P 램프 (시작 약한 P → 설정 시간 동안 최대 P까지 증가) ----
    declare_parameter("control.kp_start",      60.0);   // TRACK 진입 시 P (약하게 시작 — 중심 안 잃게)
    declare_parameter("control.kp_max",        150.0);  // 램프 끝(최대) P
    declare_parameter("control.kp_ramp_sec",   5.0);    // kp_start→kp_max 증가 시간 [s] (패널 조절)
    // ---- 중앙 데드존 + 미분필터 + 거리 게인 스케줄링 (B/C 해결) ----
    declare_parameter("control.deadzone",            0.04);  // |오차|<이 값이면 명령 0 (중앙 박스)
    declare_parameter("control.deriv_lpf_alpha",     0.25);  // 미분항 LPF (작을수록 부드러움)
    declare_parameter("control.gain_sched_near_m",   4.0);   // 이 거리 안으로 들어오면 P 감쇠 시작 [m]
    declare_parameter("control.gain_sched_min_ratio",0.35);  // 가장 가까울 때 P 비율 (1.0=감쇠끔)

    declare_parameter("mission.sitl_auto_launch",      false);
    declare_parameter("mission.auto_launch_delay_sec", 1.0);
    declare_parameter("mission.boost_duration_sec",    2.0);

    declare_parameter("mavlink.connection",  std::string("udp:127.0.0.1:14550"));
    declare_parameter("mavlink.baud",        115200);

    declare_parameter("gpio.enabled",            false);
    declare_parameter("gpio.launch_button_pin",  18);

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    SMParams sm_params;
    sm_params.confidence_threshold  = get_parameter("mission.detection_confidence_threshold").as_double();
    sm_params.lock_duration_sec     = get_parameter("mission.lock_duration_sec").as_double();
    sm_params.lost_frames_threshold = get_parameter("mission.lost_frames_threshold").as_int();
    sm_params.lock_box_tolerance    = get_parameter("mission.lock_box_tolerance").as_double();
    sm_params.fire_distance_m       = get_parameter("mission.fire_distance_m").as_double();

    auto log_fn = [this](const std::string & msg) {
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

    rkp_  = get_parameter("control.roll_pid.kp").as_double();
    rki_  = get_parameter("control.roll_pid.ki").as_double();
    rkd_  = get_parameter("control.roll_pid.kd").as_double();
    rlim_ = get_parameter("control.roll_pid.output_limit").as_double();
    pkp_  = get_parameter("control.pitch_pid.kp").as_double();
    pki_  = get_parameter("control.pitch_pid.ki").as_double();
    pkd_  = get_parameter("control.pitch_pid.kd").as_double();
    plim_ = get_parameter("control.pitch_pid.output_limit").as_double();

    throttle_       = get_parameter("control.throttle").as_double();
    track_throttle_ = get_parameter("control.track_throttle").as_double();
    boost_throttle_ = get_parameter("control.boost_throttle").as_double();
    boost_kp_       = get_parameter("control.boost_kp").as_double();
    boost_angle_limit_     = get_parameter("control.boost_angle_limit").as_double();
    boost_deviation_thresh_= get_parameter("control.boost_deviation_thresh").as_double();
    roll_sign_      = get_parameter("control.roll_sign").as_double();
    pitch_sign_     = get_parameter("control.pitch_sign").as_double();
    error_lpf_alpha_= get_parameter("control.error_lpf_alpha").as_double();
    err_sched_enable_    = get_parameter("control.err_sched_enable").as_bool();
    err_sched_full_err_  = get_parameter("control.err_sched_full_err").as_double();
    err_sched_big_err_   = get_parameter("control.err_sched_big_err").as_double();
    err_sched_min_ratio_ = get_parameter("control.err_sched_min_ratio").as_double();
    control_rate_hz_= get_parameter("control.control_rate_hz").as_double();

    kp_start_       = get_parameter("control.kp_start").as_double();
    kp_max_         = get_parameter("control.kp_max").as_double();
    kp_ramp_sec_    = get_parameter("control.kp_ramp_sec").as_double();
    deadzone_           = get_parameter("control.deadzone").as_double();
    deriv_lpf_alpha_    = get_parameter("control.deriv_lpf_alpha").as_double();
    gain_sched_near_m_  = get_parameter("control.gain_sched_near_m").as_double();
    gain_sched_min_ratio_ = get_parameter("control.gain_sched_min_ratio").as_double();
    pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
    pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);

    sitl_auto_launch_     = get_parameter("mission.sitl_auto_launch").as_bool();
    auto_launch_delay_sec_= get_parameter("mission.auto_launch_delay_sec").as_double();
    boost_duration_sec_   = get_parameter("mission.boost_duration_sec").as_double();

    // ----------------------------------------------------------------
    // MAVLink interface
    // ----------------------------------------------------------------
    auto mav_log_fn = [this](const std::string & msg) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    };
    mav_ = std::make_unique<MavlinkInterface>(
      get_parameter("mavlink.connection").as_string(), mav_log_fn);

    // ----------------------------------------------------------------
    // GPIO (launch button)
    // ----------------------------------------------------------------
    gpio_enabled_ = get_parameter("gpio.enabled").as_bool();
    gpio_pin_     = get_parameter("gpio.launch_button_pin").as_int();
    if (gpio_enabled_) {
      setup_gpio();
    }

    // 런타임 파라미터 변경 콜백 (ros2 param set / 패널 버튼이 실제로 먹게)
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        bool pid_changed = false;
        for (const auto & p : params) {
          const std::string & n = p.get_name();
          if      (n == "control.roll_sign")        roll_sign_       = p.as_double();
          else if (n == "control.pitch_sign")       pitch_sign_      = p.as_double();
          else if (n == "control.track_throttle")   track_throttle_  = p.as_double();
          else if (n == "control.throttle")         throttle_        = p.as_double();
          else if (n == "control.error_lpf_alpha")  error_lpf_alpha_ = p.as_double();
          else if (n == "control.kp_start")  kp_start_       = p.as_double();
          else if (n == "control.kp_max")    kp_max_         = p.as_double();
          else if (n == "control.kp_ramp_sec") kp_ramp_sec_  = p.as_double();
          else if (n == "control.deadzone")            deadzone_             = p.as_double();
          else if (n == "control.gain_sched_near_m")   gain_sched_near_m_    = p.as_double();
          else if (n == "control.gain_sched_min_ratio")gain_sched_min_ratio_ = p.as_double();
          else if (n == "control.deriv_lpf_alpha") {
            deriv_lpf_alpha_ = p.as_double();
            pid_roll_->set_deriv_alpha(deriv_lpf_alpha_);
            pid_pitch_->set_deriv_alpha(deriv_lpf_alpha_);
          }
          else if (n == "control.roll_pid.kp")  { rkp_ = p.as_double(); pid_changed = true; }
          else if (n == "control.roll_pid.ki")  { rki_ = p.as_double(); pid_changed = true; }
          else if (n == "control.roll_pid.kd")  { rkd_ = p.as_double(); pid_changed = true; }
          else if (n == "control.roll_pid.output_limit")  { rlim_ = p.as_double(); pid_changed = true; }
          else if (n == "control.pitch_pid.kp") { pkp_ = p.as_double(); pid_changed = true; }
          else if (n == "control.pitch_pid.ki") { pki_ = p.as_double(); pid_changed = true; }
          else if (n == "control.pitch_pid.kd") { pkd_ = p.as_double(); pid_changed = true; }
          else if (n == "control.pitch_pid.output_limit") { plim_ = p.as_double(); pid_changed = true; }
        }
        if (pid_changed) {
          pid_roll_->set_gains(rkp_, rki_, rkd_, rlim_);
          pid_pitch_->set_gains(pkp_, pki_, pkd_, plim_);
        }
        RCLCPP_INFO(get_logger(),
          "param 적용: roll_sign=%.0f pitch_sign=%.0f rkp=%.1f rkd=%.1f track_thr=%.2f",
          roll_sign_, pitch_sign_, rkp_, rkd_, track_throttle_);
        return res;
      });

    // ----------------------------------------------------------------
    // ROS subscribers & publisher
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

    // SITL: convert single-beam LaserScan to distance
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/arms/scan_raw", best_effort_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 콘 라이다의 여러 빔 중 "가장 가까운 유효 거리"를 타겟 거리로 사용
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

    pub_state_ = create_publisher<arms_msgs::msg::MissionState>("/arms/mission_state", 10);
    pub_dbg_   = create_publisher<geometry_msgs::msg::Vector3>("/arms/control_debug", 10);

    // SITL: launch 버튼을 GPIO 대신 토픽으로 대체
    //   ros2 topic pub --once /arms/launch_cmd std_msgs/msg/Empty {}
    sub_launch_ = create_subscription<std_msgs::msg::Empty>(
      "/arms/launch_cmd", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        RCLCPP_INFO(get_logger(), "Launch command received (topic).");
        sm_->on_launch_button();
      });

    // RESET: 패널 RESET 버튼이 발행. 상태머신을 SEARCH 로 강제하고
    //   RTL/FIRE 잔여 플래그를 풀고, PX4 를 offboard 로 되돌려 재무장한다.
    //   (RTL 중에 텔레포트만 하면 PX4 가 발작하므로 상태/모드를 같이 리셋)
    sub_reset_ = create_subscription<std_msgs::msg::Empty>(
      "/arms/reset_cmd", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        RCLCPP_WARN(get_logger(), "RESET command received -> force SEARCH.");
        sm_->force_search();
        rtl_sent_  = false;
        fire_sent_ = false;
        pid_roll_->reset();
        pid_pitch_->reset();
        filt_err_x_ = 0.0;
        filt_err_y_ = 0.0;
        // 멈춘(disarm/hold) 드론 되살리기: arm 먼저 → offboard 진입.
        //   stream_thread 가 계속 setpoint 를 보내고 있으므로 offboard 진입 가능.
        if (mav_->is_connected()) {
          mav_->arm();
          mav_->set_offboard_mode();
        }
      });

    // ----------------------------------------------------------------
    // Connect MAVLink & start offboard stream
    // ----------------------------------------------------------------
    if (!mav_->connect()) {
      RCLCPP_ERROR(get_logger(), "MAVLink connection failed. Continuing without FC.");
    } else {
      mav_->start_offboard_stream(static_cast<float>(control_rate_hz_));

      if (!gpio_enabled_) {
        // SITL: auto-arm after giving PX4 time to accept the offboard stream
        std::this_thread::sleep_for(2s);
        mav_->set_offboard_mode();
        std::this_thread::sleep_for(500ms);
        mav_->arm();
        sm_->arm();
        RCLCPP_INFO(get_logger(), "SITL: armed and OFFBOARD mode set.");
      }
    }

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
  // GPIO setup (Jetson, libgpiod)
  // ----------------------------------------------------------------
  void setup_gpio()
  {
#ifdef HAS_GPIO
    // libgpiod example — adjust chip/line per hardware
    gpio_chip_   = gpiod_chip_open_by_name("gpiochip0");
    if (!gpio_chip_) {
      RCLCPP_WARN(get_logger(), "Cannot open GPIO chip. Running without GPIO.");
      gpio_enabled_ = false;
      return;
    }
    gpio_line_ = gpiod_chip_get_line(gpio_chip_, gpio_pin_);
    gpiod_line_request_rising_edge_events(gpio_line_, "arms_launch_button");

    gpio_thread_ = std::thread([this]() {
      struct timespec ts{0, 100'000'000};  // 100 ms poll timeout
      while (gpio_enabled_) {
        int ret = gpiod_line_event_wait(gpio_line_, &ts);
        if (ret > 0) {
          struct gpiod_line_event event{};
          gpiod_line_event_read(gpio_line_, &event);
          RCLCPP_INFO(get_logger(), "Launch button pressed.");
          sm_->on_launch_button();
        }
      }
    });
    RCLCPP_INFO(get_logger(), "GPIO launch button on pin %d initialized.", gpio_pin_);
#else
    RCLCPP_WARN(get_logger(), "GPIO support not compiled in (HAS_GPIO not defined).");
    gpio_enabled_ = false;
#endif
  }

  // ----------------------------------------------------------------
  // 거리 캐시 + 데드존 헬퍼 (게인 스케줄링/중앙박스용)
  // ----------------------------------------------------------------
  void cache_distance(double d)
  {
    if (std::isfinite(d) && d > 0.0) {
      last_distance_      = d;
      last_distance_time_ = now();
    }
  }
  // 최근 0.5초 안에 유효 거리값을 받았는가
  bool distance_valid()
  {
    if (last_distance_ <= 0.0) return false;
    return (now() - last_distance_time_).seconds() < 0.5;
  }
  // 소프트 데드존: |e|<dz → 0, 넘으면 dz 만큼 빼서 연속적으로
  static double apply_deadzone(double e, double dz)
  {
    if (dz <= 0.0) return e;
    if (e >  dz) return e - dz;
    if (e < -dz) return e + dz;
    return 0.0;
  }

  // ----------------------------------------------------------------
  // 30 Hz control loop
  // ----------------------------------------------------------------
  void control_loop()
  {
    auto now_t = now();
    double dt  = (now_t - last_tick_).seconds();
    last_tick_ = now_t;

    State state = sm_->state();

    // ---- Compute roll / pitch commands ----
    double roll_deg  = 0.0;
    double pitch_deg = 0.0;
    float  thrust    = 0.f;

    // ---- BOOST 진입 감지: 발사 순간 풍선 각도 캡처 ----
    if (state == State::BOOST && prev_state_ != State::BOOST) {
      boost_start_time_ = now_t;
      launch_err_x_ = sm_->current_error_x();
      launch_err_y_ = sm_->current_error_y();
      // 발사 각도 = 캡처한 픽셀 오차 × boost_kp (clamp), 부호 적용
      boost_roll_  = roll_sign_  * std::clamp(boost_kp_ * launch_err_x_,
                                              -boost_angle_limit_, boost_angle_limit_);
      boost_pitch_ = pitch_sign_ * std::clamp(boost_kp_ * launch_err_y_,
                                              -boost_angle_limit_, boost_angle_limit_);
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = launch_err_x_;
      filt_err_y_ = launch_err_y_;
      RCLCPP_INFO(get_logger(),
        "BOOST 발사: roll=%.1f pitch=%.1f throttle=%.2f (각도 고정 직진)",
        boost_roll_, boost_pitch_, boost_throttle_);
    }
    // ---- TRACK 진입 감지: P 램프 타이머 리셋 (FIRE 로 넘어갈 땐 유지) ----
    if (state == State::TRACK && prev_state_ != State::TRACK && prev_state_ != State::FIRE) {
      track_enter_time_ = now_t;
      RCLCPP_INFO(get_logger(), "TRACK 진입 → P 램프 시작 (%.1f→%.1f, %.1fs)",
                  kp_start_, kp_max_, kp_ramp_sec_);
    }
    prev_state_ = state;

    if (state == State::BOOST) {
      // 발사각 고정 + 풀스로틀로 직진
      roll_deg  = boost_roll_;
      pitch_deg = boost_pitch_;
      thrust    = static_cast<float>(boost_throttle_);

      double elapsed = (now_t - boost_start_time_).seconds();
      double dev = std::hypot(sm_->current_error_x() - launch_err_x_,
                              sm_->current_error_y() - launch_err_y_);
      // 2초 경과 OR 발사각에서 너무 빗나감 → 위치보정(TRACK)으로
      if (elapsed >= boost_duration_sec_ || dev >= boost_deviation_thresh_) {
        RCLCPP_INFO(get_logger(),
          "BOOST 종료 (elapsed=%.2fs dev=%.2f) → TRACK 위치보정", elapsed, dev);
        sm_->on_boost_complete();
      }
    } else if (state == State::TRACK || state == State::FIRE) {
      // 위치보정: 픽셀 에러 LPF → PID → 각도
      double raw_ex = sm_->current_error_x();
      double raw_ey = sm_->current_error_y();
      filt_err_x_ = error_lpf_alpha_ * raw_ex + (1.0 - error_lpf_alpha_) * filt_err_x_;
      filt_err_y_ = error_lpf_alpha_ * raw_ey + (1.0 - error_lpf_alpha_) * filt_err_y_;

      // ---- 시간 기반 P 램프 ----
      //   TRACK 진입부터 경과시간에 따라 kp_start → kp_max 로 선형 증가.
      //   시작하자마자 센 P 걸면 중심 잃으니까, 설정 시간(kp_ramp_sec) 동안 천천히 올림.
      double elapsed = (now_t - track_enter_time_).seconds();
      double ramp_t  = (kp_ramp_sec_ > 1e-3)
                       ? std::clamp(elapsed / kp_ramp_sec_, 0.0, 1.0)
                       : 1.0;   // 시간 0이면 즉시 최대 P
      kp_now_ = kp_start_ + ramp_t * (kp_max_ - kp_start_);

      // ---- 거리 게인 스케줄링 (C: 근접 발산 방지) ----
      //   가까울수록 픽셀당 각도가 커져 발산 → 거리 가까우면 P 자동 감쇠.
      //   거리 신호 없으면(센서 미수신) 비율 1.0 = 풀게인(안전 폴백).
      double ratio = 1.0;
      if (distance_valid() && gain_sched_near_m_ > 1e-3 && gain_sched_min_ratio_ < 0.999) {
        double tt = std::clamp(last_distance_ / gain_sched_near_m_, 0.0, 1.0);
        ratio = gain_sched_min_ratio_ + tt * (1.0 - gain_sched_min_ratio_);
      }
      gain_ratio_filt_ = 0.15 * ratio + 0.85 * gain_ratio_filt_;  // 부드럽게
      double kp_eff = kp_now_ * gain_ratio_filt_;

      // ---- err 기반 P 자동조절 (핵심: 어떤 err 이든 안 터지고 중앙으로) ----
      //   |err| 이 클수록 P 를 낮춰서 발산 방지. 중앙 가까우면 풀 P 로 빠르게 마무리.
      //   err_mag(현재 오차 크기) 를 full_err~big_err 구간에서 1.0~min_ratio 로 선형 매핑.
      if (err_sched_enable_) {
        double err_mag = std::hypot(filt_err_x_, filt_err_y_);
        double er = 1.0;  // err 비율 (1.0=풀P, min_ratio=최소P)
        if (err_sched_big_err_ > err_sched_full_err_ + 1e-6) {
          double u = std::clamp(
              (err_mag - err_sched_full_err_) /
              (err_sched_big_err_ - err_sched_full_err_), 0.0, 1.0);
          er = 1.0 - u * (1.0 - err_sched_min_ratio_);
        }
        err_ratio_filt_ = 0.2 * er + 0.8 * err_ratio_filt_;  // 급변 방지(부드럽게)
        kp_eff *= err_ratio_filt_;
      }

      // kp 만 (램프×거리감쇠) 값으로 교체, kd/ki/limit 유지
      pid_roll_->set_gains(kp_eff, rki_, rkd_, rlim_);
      pid_pitch_->set_gains(kp_eff, pki_, pkd_, plim_);

      // ---- 중앙 데드존 박스 (B: 중앙 미세떨림/헌팅 방지) ----
      //   |오차|<deadzone 이면 0, 넘으면 deadzone 만큼 빼서 연속적으로(소프트).
      double ex = apply_deadzone(filt_err_x_, deadzone_);
      double ey = apply_deadzone(filt_err_y_, deadzone_);

      roll_deg  = roll_sign_  * pid_roll_->compute(ex, dt);
      pitch_deg = pitch_sign_ * pid_pitch_->compute(ey, dt);
      thrust    = static_cast<float>(track_throttle_);

      // 디버그: 명령 + 거리/감쇠비율 을 0.2초마다 출력
      if (++dbg_count_ % 6 == 0) {
        RCLCPP_INFO(get_logger(),
          "TRACK err=(%.2f,%.2f) kp=%.1f(errx%.2f) d=%.1fm -> roll=%.1f pitch=%.1f",
          filt_err_x_, filt_err_y_, kp_eff, err_ratio_filt_,
          distance_valid() ? last_distance_ : -1.0, roll_deg, pitch_deg);
      }
    } else {
      // IDLE / SEARCH / LOCK / RTL : 프로펠러 OFF (지상 발사 대기)
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      kp_now_ = kp_start_;
      thrust = 0.f;
    }

    // ---- SITL auto-launch (기본 off: 패널 LAUNCH 버튼으로 발사) ----
    if (sitl_auto_launch_ && !gpio_enabled_) {
      if (state == State::LOCK) {
        if (!lock_timer_started_) {
          lock_enter_time_   = now_t;
          lock_timer_started_ = true;
        } else if (!auto_launched_ &&
                   (now_t - lock_enter_time_).seconds() >= auto_launch_delay_sec_) {
          auto_launched_ = true;
          RCLCPP_INFO(get_logger(), "SITL auto-launch: LOCK -> BOOST");
          sm_->on_launch_button();
        }
      } else if (state == State::IDLE || state == State::SEARCH) {
        lock_timer_started_ = false;
        auto_launched_      = false;
      }
    }

    // ---- Send to MAVLink stream ----
    AttitudeCmd cmd;
    cmd.roll_deg  = static_cast<float>(roll_deg);
    cmd.pitch_deg = static_cast<float>(pitch_deg);
    cmd.yaw_deg   = 0.f;
    cmd.thrust    = thrust;
    mav_->set_attitude_command(cmd);

    // 디버그: 실제 나가는 명령을 토픽으로 (UI 화살표용). x=roll y=pitch z=thrust
    geometry_msgs::msg::Vector3 dbg;
    dbg.x = roll_deg;
    dbg.y = pitch_deg;
    dbg.z = thrust;
    pub_dbg_->publish(dbg);

    // ---- State-specific one-shot actions ----
    if (state == State::FIRE && !fire_sent_) {
      fire_sent_ = true;
      mav_->trigger_payload();
      RCLCPP_INFO(get_logger(), "Payload triggered.");
      sm_->on_fire_complete();
    }

    if (state == State::RTL && !rtl_sent_) {
      rtl_sent_ = true;
      mav_->send_rtl();
    }

    if (state == State::IDLE) {
      fire_sent_ = false;
      rtl_sent_  = false;
    }

    // ---- Publish mission state ----
    arms_msgs::msg::MissionState msg;
    msg.header.stamp    = now_t;
    msg.state           = to_string(state);
    msg.lock_elapsed_sec= static_cast<float>(sm_->lock_elapsed_sec());
    msg.error_x         = static_cast<float>(sm_->current_error_x());
    msg.error_y         = static_cast<float>(sm_->current_error_y());
    msg.target_locked   = sm_->target_locked();
    msg.kp_now          = static_cast<float>(kp_now_);
    pub_state_->publish(msg);
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  std::unique_ptr<StateMachine>    sm_;
  std::unique_ptr<PIDController>   pid_roll_;
  std::unique_ptr<PIDController>   pid_pitch_;
  std::unique_ptr<MavlinkInterface>mav_;

  double throttle_{0.55};
  double track_throttle_{0.60};
  double boost_throttle_{0.90};
  double boost_kp_{8.0};
  double boost_angle_limit_{15.0};
  double boost_deviation_thresh_{0.25};
  double roll_sign_{1.0};
  double pitch_sign_{1.0};
  double error_lpf_alpha_{0.3};
  double filt_err_x_{0.0};
  double filt_err_y_{0.0};
  // 시간 기반 P 램프
  double kp_start_{60.0};
  double kp_max_{150.0};
  double kp_ramp_sec_{5.0};
  double kp_now_{60.0};
  // 데드존 / 미분필터 / 거리 게인 스케줄링
  double deadzone_{0.04};
  double deriv_lpf_alpha_{0.25};
  double gain_sched_near_m_{4.0};
  double gain_sched_min_ratio_{0.35};
  double gain_ratio_filt_{1.0};
  bool   err_sched_enable_{true};
  double err_sched_full_err_{0.06};
  double err_sched_big_err_{0.35};
  double err_sched_min_ratio_{0.30};
  double err_ratio_filt_{1.0};
  double last_distance_{0.0};
  rclcpp::Time last_distance_time_;
  rclcpp::Time track_enter_time_;
  int    dbg_count_{0};
  double control_rate_hz_{30.0};

  // BOOST 단계 상태
  State        prev_state_{State::IDLE};
  rclcpp::Time boost_start_time_;
  double       boost_duration_sec_{2.0};
  double       launch_err_x_{0.0};
  double       launch_err_y_{0.0};
  double       boost_roll_{0.0};
  double       boost_pitch_{0.0};

  bool         sitl_auto_launch_{false};
  double       auto_launch_delay_sec_{1.0};
  bool         lock_timer_started_{false};
  bool         auto_launched_{false};
  rclcpp::Time lock_enter_time_;

  bool gpio_enabled_{false};
  int  gpio_pin_{18};

#ifdef HAS_GPIO
  struct gpiod_chip * gpio_chip_{nullptr};
  struct gpiod_line * gpio_line_{nullptr};
  std::thread         gpio_thread_;
#endif

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr        sub_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr          sub_launch_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr          sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr      pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr       pub_dbg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // 런타임 PID 게인 보관 (param 변경 시 set_gains 에 재적용)
  double rkp_{0}, rki_{0}, rkd_{0}, rlim_{0};
  double pkp_{0}, pki_{0}, pkd_{0}, plim_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  bool fire_sent_{false};
  bool rtl_sent_{false};
};

}  // namespace arms_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_control::ArmsControlNode>());
  rclcpp::shutdown();
  return 0;
}
