#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "arms_control/mavlink_interface.hpp"
#include "arms_control/pid_controller.hpp"
#include "arms_control/state_machine.hpp"
#include "arms_msgs/msg/detection_array.hpp"
#include "arms_msgs/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // [옵션B] 좌표 요격
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
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
    // [옵션B] 좌표 기반 요격 (homing)
    declare_parameter("control.homing_enable",
                      false);  // 좌표유도 off(B발산) → attitude(카메라PID)복귀
    declare_parameter("control.homing_speed", 4.0);      // 접근 속도 [m/s]
    declare_parameter("control.homing_fire_dist", 2.0);  // 요격 거리 [m]
    declare_parameter("control.homing_max_speed", 6.0);  // 속도 상한 [m/s]
    declare_parameter("control.homing_vz_boost", 2.0);   // 수직(상승) 속도 배수
    declare_parameter("control.homing_kp",
                      0.8);  // 거리비례 속도게인(speed=dist*kp)
    declare_parameter("control.homing_min_speed", 0.5);  // 속도 하한 [m/s]

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
    declare_parameter("control.boost_throttle", 0.90);
    declare_parameter("control.boost_kp", 8.0);
    declare_parameter("control.boost_angle_limit", 15.0);
    declare_parameter("control.boost_deviation_thresh", 0.25);
    declare_parameter("control.roll_sign", 1.0);
    declare_parameter("control.pitch_sign", 1.0);
    declare_parameter("control.error_lpf_alpha", 0.3);
    // err 기반 P 자동조절: |err| 크면 P 낮춤(발산 방지), 작으면 풀 P(빠른
    // 마무리)
    declare_parameter("control.err_sched_enable", true);  // 켜고 끄기
    declare_parameter("control.err_sched_full_err",
                      0.06);  // 이 오차 이하 = 풀 P
    declare_parameter("control.err_sched_big_err",
                      0.35);  // 이 오차 이상 = 최소 P
    declare_parameter("control.err_sched_min_ratio",
                      0.55);  // 큰 오차일 때 P 비율(30%)
    declare_parameter("control.control_rate_hz", 30.0);

    // ---- 시간 기반 P 램프 (시작 약한 P → 설정 시간 동안 최대 P까지 증가) ----
    declare_parameter("control.kp_start",
                      60.0);  // TRACK 진입 시 P (약하게 시작 — 중심 안 잃게)
    declare_parameter("control.kp_max", 150.0);  // 램프 끝(최대) P
    declare_parameter("control.kp_ramp_sec",
                      5.0);  // kp_start→kp_max 증가 시간 [s] (패널 조절)
    // ---- 중앙 데드존 + 미분필터 + 거리 게인 스케줄링 (B/C 해결) ----
    declare_parameter("control.deadzone",
                      0.04);  // |오차|<이 값이면 명령 0 (중앙 박스)
    declare_parameter("control.deriv_lpf_alpha",
                      0.25);  // 미분항 LPF (작을수록 부드러움)
    declare_parameter("control.gain_sched_near_m",
                      4.0);  // 이 거리 안으로 들어오면 P 감쇠 시작 [m]
    declare_parameter("control.gain_sched_min_ratio",
                      0.35);  // 가장 가까울 때 P 비율 (1.0=감쇠끔)

    declare_parameter("mission.sitl_auto_launch", false);
    declare_parameter("mission.auto_launch_delay_sec", 1.0);
    declare_parameter("mission.boost_duration_sec", 2.0);

    declare_parameter("mavlink.connection", std::string("udp:127.0.0.1:14550"));
    declare_parameter("mavlink.baud", 115200);

    declare_parameter("gpio.enabled", false);
    declare_parameter("gpio.launch_button_pin", 18);

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
    boost_throttle_ = get_parameter("control.boost_throttle").as_double();
    boost_kp_ = get_parameter("control.boost_kp").as_double();
    boost_angle_limit_ = get_parameter("control.boost_angle_limit").as_double();
    boost_deviation_thresh_ =
        get_parameter("control.boost_deviation_thresh").as_double();
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
    boost_duration_sec_ =
        get_parameter("mission.boost_duration_sec").as_double();

    // ----------------------------------------------------------------
    // MAVLink interface
    // ----------------------------------------------------------------
    auto mav_log_fn = [this](const std::string& msg) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    };
    mav_ = std::make_unique<MavlinkInterface>(
        get_parameter("mavlink.connection").as_string(), mav_log_fn);

    // ----------------------------------------------------------------
    // GPIO (launch button)
    // ----------------------------------------------------------------
    gpio_enabled_ = get_parameter("gpio.enabled").as_bool();
    gpio_pin_ = get_parameter("gpio.launch_button_pin").as_int();
    if (gpio_enabled_) {
      setup_gpio();
    }

    // 런타임 파라미터 변경 콜백 (ros2 param set / 패널 버튼이 실제로 먹게)
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

    pub_state_ = create_publisher<arms_msgs::msg::MissionState>(
        "/arms/mission_state", 10);
    pub_dbg_ = create_publisher<geometry_msgs::msg::Vector3>(
        "/arms/control_debug", 10);

    // [옵션B] 좌표 요격 파라미터 로드 + 풍선/드론 좌표 구독
    homing_enable_ = get_parameter("control.homing_enable").as_bool();
    homing_speed_ = get_parameter("control.homing_speed").as_double();
    homing_fire_dist_ = get_parameter("control.homing_fire_dist").as_double();
    homing_max_speed_ = get_parameter("control.homing_max_speed").as_double();
    homing_vz_boost_ = get_parameter("control.homing_vz_boost").as_double();
    homing_kp_ = get_parameter("control.homing_kp").as_double();
    homing_min_speed_ = get_parameter("control.homing_min_speed").as_double();

    rclcpp::QoS pose_qos(rclcpp::KeepLast(10));
    sub_target_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/arms/target_pose", pose_qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          tgt_x_ = m->pose.position.x;
          tgt_y_ = m->pose.position.y;
          tgt_z_ = m->pose.position.z;
          have_target_pose_ = true;
        });
    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/arms/drone_pose", pose_qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          drn_x_ = m->pose.position.x;
          drn_y_ = m->pose.position.y;
          drn_z_ = m->pose.position.z;
          have_drone_pose_ = true;
        });

    // SITL: launch 버튼을 GPIO 대신 토픽으로 대체
    //   ros2 topic pub --once /arms/launch_cmd std_msgs/msg/Empty {}
    sub_launch_ = create_subscription<std_msgs::msg::Empty>(
        "/arms/launch_cmd", 10, [this](std_msgs::msg::Empty::SharedPtr) {
          RCLCPP_INFO(get_logger(), "Launch command received (topic).");
          sm_->on_launch_button();
        });

    // RESET: 패널 RESET 버튼이 발행. 상태머신을 SEARCH 로 강제하고
    //   RTL/FIRE 잔여 플래그를 풀고, PX4 를 offboard 로 되돌려 재무장한다.
    //   (RTL 중에 텔레포트만 하면 PX4 가 발작하므로 상태/모드를 같이 리셋)
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
          // 멈춘(disarm/hold) 드론 되살리기: arm 먼저 → offboard 진입.
          //   stream_thread 가 계속 setpoint 를 보내고 있으므로 offboard 진입
          //   가능.
          if (mav_->is_connected()) {
            mav_->arm();
            mav_->set_offboard_mode();
          }
        });

    // ----------------------------------------------------------------
    // Connect MAVLink & start offboard stream
    // ----------------------------------------------------------------
    if (!mav_->connect()) {
      RCLCPP_ERROR(get_logger(),
                   "MAVLink connection failed. Continuing without FC.");
    } else {
      mav_->start_offboard_stream(static_cast<float>(control_rate_hz_));

      if (!gpio_enabled_) {
        // SITL: 스트림 안정 대기 → arm 먼저 → OFFBOARD 재시도 진입(성공확인).
        //   arm 된 상태여야 offboard 진입 성공률이 높음. set_offboard_mode 가
        //   성공할 때까지 재시도하므로 EKF 타이밍과 무관하게 항상 진입.
        std::this_thread::sleep_for(3s);  // EKF 수렴 여유
        mav_->arm();                      // arm 먼저
        std::this_thread::sleep_for(500ms);
        bool ob = mav_->set_offboard_mode();  // 재시도 진입
        sm_->arm();
        if (ob)
          RCLCPP_INFO(get_logger(), "SITL: armed and OFFBOARD 진입 성공.");
        else
          RCLCPP_ERROR(get_logger(),
                       "SITL: OFFBOARD 진입 실패 — 드론 안 움직일 수 있음. "
                       "run_arms 재시작 권장.");
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
  void setup_gpio() {
#ifdef HAS_GPIO
    // libgpiod example — adjust chip/line per hardware
    gpio_chip_ = gpiod_chip_open_by_name("gpiochip0");
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
    RCLCPP_INFO(get_logger(), "GPIO launch button on pin %d initialized.",
                gpio_pin_);
#else
    RCLCPP_WARN(get_logger(),
                "GPIO support not compiled in (HAS_GPIO not defined).");
    gpio_enabled_ = false;
#endif
  }

  // ----------------------------------------------------------------
  // 거리 캐시 + 데드존 헬퍼 (게인 스케줄링/중앙박스용)
  // ----------------------------------------------------------------
  void cache_distance(double d) {
    if (std::isfinite(d) && d > 0.0) {
      last_distance_ = d;
      last_distance_time_ = now();
    }
  }
  // 최근 0.5초 안에 유효 거리값을 받았는가
  bool distance_valid() {
    if (last_distance_ <= 0.0) return false;
    return (now() - last_distance_time_).seconds() < 0.5;
  }
  // 소프트 데드존: |e|<dz → 0, 넘으면 dz 만큼 빼서 연속적으로
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

    // ---- Compute roll / pitch commands ----
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    float thrust = 0.f;

    // ---- BOOST 진입 감지 ----
    if (state == State::BOOST && prev_state_ != State::BOOST) {
      // TODO: BOOST 활성화 시 아래 주석 해제
      // boost_start_time_ = now_t;
      // launch_err_x_ = sm_->current_error_x();
      // launch_err_y_ = sm_->current_error_y();
      // boost_roll_  = roll_sign_  * std::clamp(boost_kp_ * launch_err_x_,
      //                                         -boost_angle_limit_,
      //                                         boost_angle_limit_);
      // boost_pitch_ = pitch_sign_ * std::clamp(boost_kp_ * launch_err_y_,
      //                                         -boost_angle_limit_,
      //                                         boost_angle_limit_);
      // pid_roll_->reset();
      // pid_pitch_->reset();
      // filt_err_x_ = launch_err_x_;
      // filt_err_y_ = launch_err_y_;
      // RCLCPP_INFO(get_logger(),
      //   "BOOST 발사: roll=%.1f pitch=%.1f throttle=%.2f (각도 고정 직진)",
      //   boost_roll_, boost_pitch_, boost_throttle_);

      RCLCPP_INFO(get_logger(), "BOOST 진입 → 즉시 TRACK 전환 (boost 비활성)");
      sm_->on_boost_complete();
      state = sm_->state();
    }
    // ---- TRACK 진입 감지: P 램프 타이머 리셋 (FIRE 로 넘어갈 땐 유지) ----
    if (state == State::TRACK && prev_state_ != State::TRACK &&
        prev_state_ != State::FIRE) {
      track_enter_time_ = now_t;
      RCLCPP_INFO(get_logger(), "TRACK 진입 → P 램프 시작 (%.1f→%.1f, %.1fs)",
                  kp_start_, kp_max_, kp_ramp_sec_);
    }
    prev_state_ = state;

    if (state == State::BOOST) {
      // 발사각 고정 + 풀스로틀로 직진
      roll_deg = boost_roll_;
      pitch_deg = boost_pitch_;
      thrust = static_cast<float>(boost_throttle_);

      double elapsed = (now_t - boost_start_time_).seconds();
      double dev = std::hypot(sm_->current_error_x() - launch_err_x_,
                              sm_->current_error_y() - launch_err_y_);
      // 2초 경과 OR 발사각에서 너무 빗나감 → 위치보정(TRACK)으로
      if (elapsed >= boost_duration_sec_ || dev >= boost_deviation_thresh_) {
        RCLCPP_INFO(get_logger(),
                    "BOOST 종료 (elapsed=%.2fs dev=%.2f) → TRACK 위치보정",
                    elapsed, dev);
        sm_->on_boost_complete();
      }
    } else if (state == State::TRACK || state == State::FIRE) {
      // 위치보정: 픽셀 에러 LPF → PID → 각도
      double raw_ex = sm_->current_error_x();
      double raw_ey = sm_->current_error_y();
      filt_err_x_ =
          error_lpf_alpha_ * raw_ex + (1.0 - error_lpf_alpha_) * filt_err_x_;
      filt_err_y_ =
          error_lpf_alpha_ * raw_ey + (1.0 - error_lpf_alpha_) * filt_err_y_;

      // ---- 시간 기반 P 램프 ----
      //   TRACK 진입부터 경과시간에 따라 kp_start → kp_max 로 선형 증가.
      //   시작하자마자 센 P 걸면 중심 잃으니까, 설정 시간(kp_ramp_sec) 동안
      //   천천히 올림.
      double elapsed = (now_t - track_enter_time_).seconds();
      double ramp_t = (kp_ramp_sec_ > 1e-3)
                          ? std::clamp(elapsed / kp_ramp_sec_, 0.0, 1.0)
                          : 1.0;  // 시간 0이면 즉시 최대 P
      kp_now_ = kp_start_ + ramp_t * (kp_max_ - kp_start_);

      // ---- 거리 게인 스케줄링 (C: 근접 발산 방지) ----
      //   가까울수록 픽셀당 각도가 커져 발산 → 거리 가까우면 P 자동 감쇠.
      //   거리 신호 없으면(센서 미수신) 비율 1.0 = 풀게인(안전 폴백).
      double ratio = 1.0;
      if (distance_valid() && gain_sched_near_m_ > 1e-3 &&
          gain_sched_min_ratio_ < 0.999) {
        double tt = std::clamp(last_distance_ / gain_sched_near_m_, 0.0, 1.0);
        ratio = gain_sched_min_ratio_ + tt * (1.0 - gain_sched_min_ratio_);
      }
      gain_ratio_filt_ = 0.15 * ratio + 0.85 * gain_ratio_filt_;  // 부드럽게
      double kp_eff = kp_now_ * gain_ratio_filt_;

      // ---- err 기반 P 자동조절 (핵심: 어떤 err 이든 안 터지고 중앙으로) ----
      //   |err| 이 클수록 P 를 낮춰서 발산 방지. 중앙 가까우면 풀 P 로 빠르게
      //   마무리. err_mag(현재 오차 크기) 를 full_err~big_err
      //   구간에서 1.0~min_ratio 로 선형 매핑.
      if (err_sched_enable_) {
        double err_mag = std::hypot(filt_err_x_, filt_err_y_);
        double er = 1.0;  // err 비율 (1.0=풀P, min_ratio=최소P)
        if (err_sched_big_err_ > err_sched_full_err_ + 1e-6) {
          double u = std::clamp((err_mag - err_sched_full_err_) /
                                    (err_sched_big_err_ - err_sched_full_err_),
                                0.0, 1.0);
          er = 1.0 - u * (1.0 - err_sched_min_ratio_);
        }
        err_ratio_filt_ =
            0.2 * er + 0.8 * err_ratio_filt_;  // 급변 방지(부드럽게)
        kp_eff *= err_ratio_filt_;
      }

      // kp 만 (램프×거리감쇠) 값으로 교체, kd/ki/limit 유지
      pid_roll_->set_gains(kp_eff, rki_, rkd_, rlim_);
      // pitch(전진) 게인만 강화 → 공쪽으로 더 세게 꺾어 비스듬히 접근
      const double pitch_gain_mul = 1.6;  // pitch kp = roll kp × 이 배수
      pid_pitch_->set_gains(kp_eff * pitch_gain_mul, pki_, pkd_, plim_);

      // ---- 중앙 데드존 박스 (B: 중앙 미세떨림/헌팅 방지) ----
      //   |오차|<deadzone 이면 0, 넘으면 deadzone 만큼 빼서 연속적으로(소프트).
      double ex = apply_deadzone(filt_err_x_, deadzone_);
      double ey = apply_deadzone(filt_err_y_, deadzone_);

      roll_deg = roll_sign_ * pid_roll_->compute(ex, dt);
      pitch_deg = pitch_sign_ * pid_pitch_->compute(ey, dt);
      // ---- 정렬 게이트 + 라이다 거리제어 v4 ----
      //   핵심: 상승 전에 공을 화면중앙 정렬(err<align_thr). 정렬 전엔 제자리
      //   호버로 조준만 → 시작오차 크든작든 항상 작은오차로 상승시작 → 항상
      //   성공.
      {
        const double up = track_throttle_;  // 상승 추력
        const double hover = 0.62;          // 호버(제자리, 안떨어지는 최소)
        const double fire_d = 5.0;          // FIRE 거리 (근접)
        const double align_thr =
            0.10;  // 정렬 완료 기준 (오차 이 이하면 정렬됨)
        double emag = std::hypot(filt_err_x_, filt_err_y_);  // 현재 오차크기

        // 정렬 완료 래치: 한번 정렬되면 유지 (다시 흐트러져도 상승 계속)
        if (!homing_locked_ && emag < align_thr) {
          homing_locked_ = true;  // 정렬완료 플래그로 재사용
          RCLCPP_INFO(get_logger(), "정렬 완료 (오차 %.2f) → 상승 요격 시작",
                      emag);
        }

        if (!homing_locked_) {
          // [정렬 단계] 아직 정렬 안됨 → 제자리 호버로 조준만 (상승X)
          thrust = static_cast<float>(hover);
        } else {
          // [요격 단계] 정렬됨 → 상승. 단, 상승중 오차(x또는y) 크게 폭발하면
          //   상승멈추고 제자리서 정렬 회복. (err_x만 봤더니 err_y 폭발을 놓침)
          const double xy_gate =
              0.22;  // err 전체 이 이상이면 상승멈추고 정렬회복
          double emag_now = std::hypot(filt_err_x_, filt_err_y_);
          if (emag_now > xy_gate) {
            // 좌우/상하 크게 틀어짐 → 상승 멈추고 호버로 정렬 회복
            thrust = static_cast<float>(hover);
          } else if (distance_valid()) {
            double d = last_distance_;
            if (d > fire_d + 3.0) {
              thrust = static_cast<float>(up);  // 멀다→상승
            } else if (d > fire_d) {
              double t = (d - fire_d) / 3.0;
              thrust = static_cast<float>(hover + (up - hover) * t);  // 감속
            } else {
              thrust = static_cast<float>(hover);  // 도달→호버(FIRE)
            }
          } else {
            thrust = static_cast<float>(up);  // 라이다 못잡음→상승
          }
        }
      }

      // 디버그: 명령 + 거리/감쇠비율 을 0.2초마다 출력
      if (++dbg_count_ % 6 == 0) {
        RCLCPP_INFO(get_logger(),
                    "TRACK err=(%.2f,%.2f) kp=%.1f(errx%.2f) d=%.1fm -> "
                    "roll=%.1f pitch=%.1f",
                    filt_err_x_, filt_err_y_, kp_eff, err_ratio_filt_,
                    distance_valid() ? last_distance_ : -1.0, roll_deg,
                    pitch_deg);
      }
    } else {
      // IDLE / SEARCH / LOCK / RTL : 프로펠러 OFF (지상 발사 대기)
      pid_roll_->reset();
      pid_pitch_->reset();
      filt_err_x_ = 0.0;
      filt_err_y_ = 0.0;
      kp_now_ = kp_start_;
      thrust = 0.f;
      homing_locked_ = false;  // 정렬 게이트 리셋 (다음 발사때 새로 정렬)
    }

    // ---- SITL auto-launch (기본 off: 패널 LAUNCH 버튼으로 발사) ----
    if (sitl_auto_launch_ && !gpio_enabled_) {
      if (state == State::LOCK) {
        if (!lock_timer_started_) {
          lock_enter_time_ = now_t;
          lock_timer_started_ = true;
        } else if (!auto_launched_ && (now_t - lock_enter_time_).seconds() >=
                                          auto_launch_delay_sec_) {
          auto_launched_ = true;
          RCLCPP_INFO(get_logger(), "SITL auto-launch: LOCK -> BOOST");
          sm_->on_launch_button();
        }
      } else if (state == State::IDLE || state == State::SEARCH) {
        lock_timer_started_ = false;
        auto_launched_ = false;
      }
    }

    // ===== [옵션B] 좌표 기반 요격 유도 (homing) =====
    //   TRACK/FIRE 상태 + homing 켜짐 + 풍선/드론 좌표 수신 → 픽셀 PID 대신
    //   좌표 직진. 풍선이 화면 어디 있든 좌표로 정확히 방향+거리 계산 →
    //   속도명령으로 직진. 거리는 sm_->on_distance() 로 주입 → 기존 FIRE
    //   로직(fire_distance_m)이 자동 처리.
    bool homing_active = false;
    // TRACK/FIRE 가 아니면 목표 고정 해제 (다음 요격 위해)
    if (state != State::TRACK && state != State::FIRE) {
      homing_locked_ = false;
      homing_fired_ = false;
      phoriz_min_ = 1e9;
      phoriz_rise_cnt_ = 0;
    }
    if (homing_enable_ && (state == State::TRACK || state == State::FIRE) &&
        have_target_pose_ && have_drone_pose_) {
      // 요격 벡터 (Gazebo ENU)
      double ex = tgt_x_ - drn_x_;
      double ey = tgt_y_ - drn_y_;
      double ez = tgt_z_ - drn_z_;
      double dist = std::sqrt(ex * ex + ey * ey + ez * ez);
      homing_dist_ = dist;

      // 좌표 거리를 상태머신에 주입. 래치 FIRE 면 0 주입해 확실히 FIRE 트리거.
      sm_->on_distance(homing_fired_ ? 0.0 : dist);

      if (dist > 1e-3) {
        // ---- 목표고정 위치명령 (직선수렴) + 도착 FIRE ----
        //   속도명령은 추적곡선이라 비스듬한 풍선에 빙 돌며 발산.
        //   위치명령은 목표점에 직선 수렴(떨림X 공전X 발산X).
        //   z 상승한계는 PX4 MPC_Z_VEL_MAX_UP 로 해결(pxh 콘솔서 param set).
        float cur_n, cur_e, cur_d;
        if (mav_->get_position_ned(cur_n, cur_e, cur_d)) {
          if (!homing_locked_) {
            double rel_n = (tgt_y_ - drn_y_);   // ENU_y → NED_north
            double rel_e = (tgt_x_ - drn_x_);   // ENU_x → NED_east
            double rel_d = -(tgt_z_ - drn_z_);  // ENU_z → NED_down
            lock_n_ = cur_n + static_cast<float>(rel_n);
            lock_e_ = cur_e + static_cast<float>(rel_e);
            // 고도는 풍선보다 offset 낮게 (down +가 아래) → 추락방지·요격자세
            lock_d_ = cur_d + static_cast<float>(rel_d) +
                      static_cast<float>(homing_alt_offset_);
            homing_locked_ = true;
            RCLCPP_INFO(
                get_logger(),
                "HOMING 목표고정: 풍선 PX4절대=(%.1f,%.1f,%.1f) [dist=%.1fm]",
                lock_n_, lock_e_, lock_d_, dist);
          }

          // 고정목표까지 거리 — 3D 와 수평 둘 다 계산
          double pn = lock_n_ - cur_n, pe = lock_e_ - cur_e,
                 pd = lock_d_ - cur_d;
          double pdist = std::sqrt(pn * pn + pe * pe + pd * pd);  // 3D
          double phoriz = std::sqrt(pn * pn + pe * pe);           // 수평만

          // ── FIRE 판정 — Gazebo 실제거리(dist) 기준 ──
          //   dist = 공-드론 실제 3D 거리(ENU, 580번). PX4 pdist 와 달리
          //   화면 실제 거리와 정확히 일치 → 진짜 만나야 요격.
          // (A) 근접 명중: 드론이 공 4m 안에 실제로 오면 무조건 FIRE.
          if (dist < 4.0) {
            homing_fired_ = true;
          }
          // (B) 최저점 자동감지 — dist(Gazebo 실거리) 기준.
          //   드론이 공에 실제로 가장 가까워진 순간에 FIRE (공이 옆 지나가는
          //   순간).
          if (dist < homing_near_dist_) {     // 실제로 충분히 접근했을때만
            if (dist < phoriz_min_ - 0.03) {  // 최저 실거리 갱신
              phoriz_min_ = dist;
              phoriz_rise_cnt_ = 0;
            } else if (dist > phoriz_min_ + 0.05) {  // 최저보다 5cm 이상 멀어짐
              if (++phoriz_rise_cnt_ >=
                  3) {  // 연속 3프레임 = 최저점(가장가까움) 지남
                homing_fired_ = true;
              }
            }
          }

          if (homing_fired_) {
            mav_->set_velocity_ned(0.0f, 0.0f, 0.0f,
                                   0.0f);  // 정지 호버 → FIRE 유지
            homing_active = true;
            if (++dbg_count_ % 6 == 0)
              RCLCPP_INFO(
                  get_logger(),
                  "HOMING FIRE확정! 공까지 실제거리=%.2fm (최저 %.2f) → 요격",
                  dist, phoriz_min_);
          } else {
            mav_->set_position_ned(lock_n_, lock_e_, lock_d_,
                                   0.0f);  // 직선 수렴
            homing_active = true;
            if (++dbg_count_ % 6 == 0)
              RCLCPP_INFO(
                  get_logger(),
                  "HOMING(pos) 실거리=%.2f pdist=%.2f cur=(%.1f,%.1f,%.1f)",
                  dist, pdist, cur_n, cur_e, cur_d);
          }
        } else {
          if (++dbg_count_ % 30 == 0)
            RCLCPP_WARN(get_logger(), "HOMING: PX4 위치 telemetry 대기중...");
        }
      }
    }

    // ---- Send to MAVLink stream ----
    if (!homing_active) {
      AttitudeCmd cmd;
      cmd.roll_deg = static_cast<float>(roll_deg);
      cmd.pitch_deg = static_cast<float>(pitch_deg);
      cmd.yaw_deg = 0.f;
      cmd.thrust = thrust;
      mav_->set_attitude_command(cmd);
    }

    // 디버그: 실제 나가는 명령을 토픽으로 (UI 화살표용). x=roll y=pitch
    // z=thrust
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
  std::unique_ptr<MavlinkInterface> mav_;

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

  // BOOST 단계 상태
  State prev_state_{State::IDLE};
  rclcpp::Time boost_start_time_;
  double boost_duration_sec_{2.0};
  double launch_err_x_{0.0};
  double launch_err_y_{0.0};
  double boost_roll_{0.0};
  double boost_pitch_{0.0};

  bool sitl_auto_launch_{false};
  double auto_launch_delay_sec_{1.0};
  bool lock_timer_started_{false};
  bool auto_launched_{false};
  rclcpp::Time lock_enter_time_;

  bool gpio_enabled_{false};
  int gpio_pin_{18};

#ifdef HAS_GPIO
  struct gpiod_chip* gpio_chip_{nullptr};
  struct gpiod_line* gpio_line_{nullptr};
  std::thread gpio_thread_;
#endif

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr
      sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_launch_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_dbg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;

  // 런타임 PID 게인 보관 (param 변경 시 set_gains 에 재적용)
  double rkp_{0}, rki_{0}, rkd_{0}, rlim_{0};
  double pkp_{0}, pki_{0}, pkd_{0}, plim_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  bool fire_sent_{false};
  bool rtl_sent_{false};
  // [옵션B] 좌표 요격
  bool homing_enable_{true};
  double homing_speed_{4.0};
  double homing_fire_dist_{2.0};
  double homing_max_speed_{6.0};
  double homing_vz_boost_{2.0};
  double homing_kp_{0.8};
  double homing_min_speed_{0.5};
  bool have_target_pose_{false};
  bool have_drone_pose_{false};
  double tgt_x_{0}, tgt_y_{0}, tgt_z_{0};  // 풍선 ENU
  double drn_x_{0}, drn_y_{0}, drn_z_{0};  // 드론 ENU
  double homing_dist_{-1.0};               // 최근 요격거리
  bool homing_locked_{false};              // 목표 절대좌표 고정여부
  float lock_n_{0}, lock_e_{0}, lock_d_{0};
  bool homing_fired_{false};
  double phoriz_min_{1e9};  // 지금까지 최저 수평거리
  int phoriz_rise_cnt_{0};  // phoriz 가 연속 증가한 프레임 수
  double homing_near_dist_{6.0};  // 이 거리 안에서만 최저점 감지(오판방지)  //
                                  // FIRE 래치(한번 도달하면 유지)
  double homing_alt_offset_{6.0};  // 목표를 풍선보다 이만큼 아래로(추락방지) //
                                   // 고정된 풍선 PX4 절대좌표
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_target_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_drone_pose_;
};

}  // namespace arms_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_control::ArmsControlNode>());
  rclcpp::shutdown();
  return 0;
}
