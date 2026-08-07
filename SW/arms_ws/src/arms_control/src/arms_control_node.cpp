#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "arms_control/crsf_output.hpp"
#include "arms_control/pid_controller.hpp"
#include "arms_control/servo_lock.hpp"
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
    declare_parameter("mission.fire_align_tol", 0.2);  // FIRE 정렬 허용오차(가짜명중 방지)

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

    // 자동 모드 CH5(FC arm)를 켜는 상태 목록. 자동 모드에선 arm 을 스위치와 분리해
    //   컨트롤 노드가 상태머신 상태로 결정한다. 기본=발사(TRACK)부터 무장.
    //   나중에 SEARCH 등에서 arm 하도록 바꾸려면 이 목록만 수정하면 된다.
    declare_parameter("control.auto_arm_states",
                      std::vector<std::string>{"TRACK", "FIRE", "RTL"});

    // 발사 잠금장치 서보 (Jetson 하드웨어 PWM, sysfs). SITL 은 enabled=false.
    declare_parameter("servo.enabled", false);
    declare_parameter("servo.chip_path", std::string("/sys/class/pwm/pwmchip0"));
    declare_parameter("servo.channel", 0);
    declare_parameter("servo.period_ns", 20000000);    // 50Hz = 20ms
    declare_parameter("servo.lock_duty_ns", 1500000);  // 90°  = 1.5ms (LOCK 기본)
    declare_parameter("servo.open_duty_ns", 2500000);  // 180° = 2.5ms (OPEN 기본)

    declare_parameter("crsf.port", std::string("/tmp/crsf_tx"));
    declare_parameter("crsf.baud", 400000);
    declare_parameter("crsf.max_angle_deg", 35.0);
    // ACRO(각속도) 제어: true 면 자동요격 출력을 '각속도[deg/s]' 명령으로 내보냄
    //   (PX4 ACRO 모드용). false 면 기존 '각도[deg]' 명령(Stabilized/Manual).
    //   ★ 브리지(sitl_bridge)의 autonomous_acro 와 반드시 일치시킬 것.
    declare_parameter("control.acro_mode", true);
    declare_parameter("control.max_rate_dps", 120.0);  // 풀스틱 각속도[deg/s], PX4 MC_ACRO_*_MAX 와 일치
    // 추격 궤적(Pursuit): 정렬 기다리지 않고 조준하며 처음부터 상승.
    declare_parameter("control.hover_throttle", 0.62);  // 겨냥 안 될 때 최소 상승(호버)
    declare_parameter("control.pursuit_gate", 0.25);    // 오차 이 이하면 full 상승(공쪽으로 돌진)
    // 예측조준(Lead): 움직이는 공의 미래 위치를 겨냥. lead_gain(리드 세기)은 위에 이미 있음.
    declare_parameter("control.lead_dot_alpha", 0.2);   // 표적 속도추정 LPF(클수록 민첩/노이즈↑)
    declare_parameter("control.lead_clamp", 0.6);       // 속도추정 스파이크 제한[1/s]
    // 자세 피드백(각속도 제어 캐스케이드): 비전=목표기울기 → 자세루프가 rate 산출
    declare_parameter("control.att_p", 6.0);            // 자세루프 게인: rate=att_p×(목표각−실제각)
    declare_parameter("control.max_tilt_deg", 35.0);    // 목표 기울기 제한[deg]
    declare_parameter("control.att_roll_sign", 1.0);    // 자세 부호(실측: roll +, pitch -)
    declare_parameter("control.att_pitch_sign", -1.0);
    // 유도(Guidance): ① 자세보정 LOS  ② 거리비례 예측
    declare_parameter("control.att_comp", 0.0);         // 자세보정 게인(≈1/half_FOV≈0.017, 0=off)
    declare_parameter("control.lead_dist", 0.02);       // 거리비례 예측: 리드시간 += 이값×거리[m]
    // 유도 방식: 0=추미(Pursuit, 현위치추종/기존), 1=비례항법(PN, 시선각속도 제거/빠른표적)
    declare_parameter("control.guidance_mode", 0);
    declare_parameter("control.pn_nav_gain", 150.0);    // PN 항법이득 N (LOS각속도→기울기). 못따라가면↑ 떨리면↓
    declare_parameter("control.pn_center_gain", 40.0);  // 표적 중심유지(화면 이탈 방지)
    declare_parameter("control.pn_alpha", 0.5);         // alpha-beta 위치이득 (표적 상태추정)
    declare_parameter("control.pn_beta", 0.15);         // alpha-beta 속도이득 (LOS각속도 추정)
    declare_parameter("control.pn_los_clamp", 1.5);     // PN 시선각속도 제한(종말 폭주방지, 0=무제한)
    // 종말 커밋(Terminal Commit): 신뢰거리가 이 값 이내면 losdot 을 강클램프 →
    //   가로지르는 표적을 쫓다 튕기지 않고 '충돌 코스로 직진'해서 실제로 박는다.
    declare_parameter("control.pn_commit_dist", 5.0);      // 이 거리[m] 이내면 종말 부스트
    declare_parameter("control.pn_commit_clamp", 0.5);     // (미사용) 구 종말커밋 클램프
    declare_parameter("control.pn_commit_att_p", 6.5);     // 종말 자세게인(빠른 크로싱 추종). 짧은순간만 → 발진 X
    declare_parameter("control.pn_commit_max_tilt", 58.0); // 종말 최대기울기(더 강한 횡가속)

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
    sm_params.fire_align_tol =
        get_parameter("mission.fire_align_tol").as_double();

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

    {
      const auto v = get_parameter("control.auto_arm_states").as_string_array();
      auto_arm_states_ = std::set<std::string>(v.begin(), v.end());
    }

    crsf_max_angle_ = get_parameter("crsf.max_angle_deg").as_double();
    acro_mode_ = get_parameter("control.acro_mode").as_bool();
    max_rate_dps_ = get_parameter("control.max_rate_dps").as_double();
    hover_throttle_ = get_parameter("control.hover_throttle").as_double();
    pursuit_gate_ = get_parameter("control.pursuit_gate").as_double();
    lead_dot_alpha_ = get_parameter("control.lead_dot_alpha").as_double();
    lead_clamp_ = get_parameter("control.lead_clamp").as_double();
    att_p_ = get_parameter("control.att_p").as_double();
    max_tilt_deg_ = get_parameter("control.max_tilt_deg").as_double();
    att_roll_sign_ = get_parameter("control.att_roll_sign").as_double();
    att_pitch_sign_ = get_parameter("control.att_pitch_sign").as_double();
    att_comp_ = get_parameter("control.att_comp").as_double();
    lead_dist_ = get_parameter("control.lead_dist").as_double();
    guidance_mode_ = static_cast<int>(get_parameter("control.guidance_mode").as_int());
    pn_nav_gain_ = get_parameter("control.pn_nav_gain").as_double();
    pn_center_gain_ = get_parameter("control.pn_center_gain").as_double();
    pn_alpha_ = get_parameter("control.pn_alpha").as_double();
    pn_beta_ = get_parameter("control.pn_beta").as_double();
    pn_los_clamp_ = get_parameter("control.pn_los_clamp").as_double();
    pn_commit_dist_ = get_parameter("control.pn_commit_dist").as_double();
    pn_commit_clamp_ = get_parameter("control.pn_commit_clamp").as_double();
    pn_commit_att_p_ = get_parameter("control.pn_commit_att_p").as_double();
    pn_commit_max_tilt_ = get_parameter("control.pn_commit_max_tilt").as_double();
    crsf_out_ = std::make_unique<CrsfOutput>(
        get_parameter("crsf.port").as_string(),
        static_cast<int>(get_parameter("crsf.baud").as_int()));

    // ---- 발사 잠금장치 서보 ----
    ServoLock::Params servo_params;
    servo_params.enabled     = get_parameter("servo.enabled").as_bool();
    servo_params.chip_path   = get_parameter("servo.chip_path").as_string();
    servo_params.channel     = static_cast<int>(get_parameter("servo.channel").as_int());
    servo_params.period_ns   = static_cast<long>(get_parameter("servo.period_ns").as_int());
    servo_params.lock_duty_ns = static_cast<long>(get_parameter("servo.lock_duty_ns").as_int());
    servo_params.open_duty_ns = static_cast<long>(get_parameter("servo.open_duty_ns").as_int());
    servo_ = std::make_unique<ServoLock>(servo_params, log_fn);
    servo_->init();

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
            else if (n == "control.acro_mode")
              acro_mode_ = p.as_bool();
            else if (n == "control.max_rate_dps")
              max_rate_dps_ = p.as_double();
            else if (n == "control.hover_throttle")
              hover_throttle_ = p.as_double();
            else if (n == "control.pursuit_gate")
              pursuit_gate_ = p.as_double();
            else if (n == "control.lead_dot_alpha")
              lead_dot_alpha_ = p.as_double();
            else if (n == "control.lead_clamp")
              lead_clamp_ = p.as_double();
            else if (n == "control.att_p")
              att_p_ = p.as_double();
            else if (n == "control.max_tilt_deg")
              max_tilt_deg_ = p.as_double();
            else if (n == "control.att_roll_sign")
              att_roll_sign_ = p.as_double();
            else if (n == "control.att_pitch_sign")
              att_pitch_sign_ = p.as_double();
            else if (n == "control.att_comp")
              att_comp_ = p.as_double();
            else if (n == "control.lead_dist")
              lead_dist_ = p.as_double();
            else if (n == "control.guidance_mode")
              guidance_mode_ = static_cast<int>(p.as_int());
            else if (n == "control.pn_nav_gain")
              pn_nav_gain_ = p.as_double();
            else if (n == "control.pn_center_gain")
              pn_center_gain_ = p.as_double();
            else if (n == "control.pn_alpha")
              pn_alpha_ = p.as_double();
            else if (n == "control.pn_beta")
              pn_beta_ = p.as_double();
            else if (n == "control.pn_los_clamp")
              pn_los_clamp_ = p.as_double();
            else if (n == "control.pn_commit_dist")
              pn_commit_dist_ = p.as_double();
            else if (n == "control.pn_commit_clamp")
              pn_commit_clamp_ = p.as_double();
            else if (n == "control.pn_commit_att_p")
              pn_commit_att_p_ = p.as_double();
            else if (n == "control.pn_commit_max_tilt")
              pn_commit_max_tilt_ = p.as_double();
            else if (n == "mission.fire_distance_m")
              sm_->set_fire_distance(p.as_double());
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

    // 심판이 주는 실제 표적거리(ground truth). 시뮬 라이다가 기울면서 표적을 놓쳐
    // d=-1.0/엉뚱한 값이 되는 문제를 우회 → 신뢰 거리로 FIRE 판정.
    sub_range_ = create_subscription<std_msgs::msg::Float64>(
        "/arms/target_range", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
          cache_distance(msg->data);
          sm_->on_distance(msg->data);
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
          // ★ SITL: 라이다는 기울면 표적을 놓쳐 엉뚱한 값(51m 등)을 줘서 심판 실제거리와
          //   충돌(거리 널뛰기)한다. 그래서 거리 판정은 심판 target_range 만 쓰고 라이다는
          //   피드하지 않는다. (실기체에선 이 라이다/레이더가 거리원이 됨)
          (void)best;
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
            // ---- ARM 재토글 안전장치 ----
            //   모드 전환 순간 ARM 스위치가 올라가 있어도 즉시 arm 되면 위험하다
            //   (예: auto+SEARCH=ARM 상태에서 manual 로 바꾸면 CH5 가 곧바로 arm).
            //   모드가 바뀌면 래치를 걸어, DISARM 으로 내렸다가 다시 올려야만 arm 되게 한다.
            require_arm_reset_ = true;
            RCLCPP_INFO(get_logger(), "Mode: %s (ARM 재토글 필요)",
                        joy_manual_mode_ ? "MANUAL (Altitude)" : "AUTO (Manual)");
          }
          if (launch && !prev_btn_[3]) {
            sm_->on_launch_button();
          }

          joy_kill_ = static_cast<bool>(kill);
          joy_arm_  = static_cast<bool>(arm);
          // ARM 스위치를 기본 위치(DISARM)로 내리면 재토글 래치 해제.
          if (!joy_arm_) require_arm_reset_ = false;
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

    // 심판의 실제충돌(직격) 통지 → FIRE→RTL (라이다 판정과 무관, 진짜 명중)
    sub_hit_ = create_subscription<std_msgs::msg::Empty>(
        "/arms/hit", 10, [this](std_msgs::msg::Empty::SharedPtr) {
          RCLCPP_INFO(get_logger(), "직격 명중 통지 수신 → FIRE.");
          sm_->on_external_hit();
        });

    // 드론 실제 자세(브리지가 PX4 ATTITUDE 를 발행) → 자세 피드백 각속도 제어에 사용
    sub_attitude_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/arms/attitude", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
          att_roll_deg_  = att_roll_sign_  * msg->x;
          att_pitch_deg_ = att_pitch_sign_ * msg->y;
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
        const int up_rssi_1 = static_cast<int>(static_cast<int8_t>(frame.payload[0]));   // 상향 안테나 1 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int up_rssi_2 = static_cast<int>(static_cast<int8_t>(frame.payload[1]));   // 상향 안테나 2 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int up_lq = static_cast<int>(frame.payload[2]);              // 상향 링크 품질을 백분율로 읽는다.
        const int up_snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // 상향 SNR의 부호를 복원한다.
        const int down_rssi = static_cast<int>(static_cast<int8_t>(frame.payload[7]));   // 하향 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
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
        const int rssi = static_cast<int>(static_cast<int8_t>(frame.payload[0]));   // 단일 링크 RSSI를 int8 부호로 복원해 dBm으로 읽는다.
        const int rssi_percent = static_cast<int>(frame.payload[1]);       // RSSI 백분율을 읽는다.
        const int link_quality = static_cast<int>(frame.payload[2]);       // 링크 품질을 백분율로 읽는다.
        const int snr = static_cast<int>(static_cast<int8_t>(frame.payload[3]));   // SNR의 부호를 복원한다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ELRS %s link: rssi=%ddBm rssi_pct=%d%% lq=%d%% snr=%ddB",
          frame.type == 0x1C ? "RX" : "TX",
          rssi, rssi_percent, link_quality, snr);                 // 분리형 링크 상태를 1초마다 보여준다.
      } else if (frame.type == 0x08 && frame.payload.size() >= 8) {
        const int voltage_dv =
          (static_cast<int>(frame.payload[0]) << 8) | frame.payload[1];   // 전압을 0.1V 단위(big-endian)로 읽는다.
        const int current_da =
          (static_cast<int>(frame.payload[2]) << 8) | frame.payload[3];   // 전류를 0.1A 단위(big-endian)로 읽는다.
        const int capacity_mah =
          (static_cast<int>(frame.payload[4]) << 16) |
          (static_cast<int>(frame.payload[5]) << 8) | frame.payload[6];    // 사용 용량을 mAh(3바이트 big-endian)로 읽는다.
        const int remaining_pct = static_cast<int>(frame.payload[7]);      // 남은 배터리 백분율을 읽는다.

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CRSF battery: %.1fV %.1fA used=%dmAh remain=%d%%",
          voltage_dv / 10.0, current_da / 10.0,
          capacity_mah, remaining_pct);                            // 배터리 텔레메트리를 1초마다 보여준다.
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

      // ---- ① 자세 보정 LOS (카메라-기울기 결합 제거) ----
      //   픽셀 오차는 기체가 기울면 오염된다(기울면 카메라도 기울어 표적이 딴 데 있는 듯 보임).
      //   실제 자세(att_*_deg_, 브리지가 발행)로 그 성분을 되돌려 '공의 진짜 방향(LOS)'을 얻음.
      //   att_comp ≈ 1/half_FOV. 0이면 보정 off(기존 동작). 과하면 발산 → 0부터 올려 튜닝.
      double los_x = filt_err_x_ + att_comp_ * att_roll_deg_;
      double los_y = filt_err_y_ + att_comp_ * att_pitch_deg_;

      // ---- 중앙 데드존 (보정된 LOS 기준) ----
      double ex = apply_deadzone(los_x, deadzone_);
      double ey = apply_deadzone(los_y, deadzone_);
      if (dt > 1e-6) {
        // 표적 시선각속도(LOS rate) 추정 = 예측조준의 핵심 신호 (보정 LOS 의 미분).
        const double DOT_A = lead_dot_alpha_;
        double raw_dx = (los_x - prev_err_x_) / dt;
        double raw_dy = (los_y - prev_err_y_) / dt;
        err_dot_x_ = DOT_A * raw_dx + (1.0 - DOT_A) * err_dot_x_;
        err_dot_y_ = DOT_A * raw_dy + (1.0 - DOT_A) * err_dot_y_;
        err_dot_x_ = std::clamp(err_dot_x_, -lead_clamp_, lead_clamp_);
        err_dot_y_ = std::clamp(err_dot_y_, -lead_clamp_, lead_clamp_);
      }
      prev_err_x_ = los_x;
      prev_err_y_ = los_y;

      bool held = sm_->is_detection_held();
      pid_roll_->set_integral_frozen(held);
      pid_pitch_->set_integral_frozen(held);

      // ---- 표적 상태추정: alpha-beta 필터 (매끈한 LOS + LOS각속도) ----
      //   거친 미분(+클램프) 대신 예측-보정 필터로 시선각속도를 깔끔히 추정한다.
      //   빠른 표적 추적의 핵심 신호 → PN 유도가 이 losf_*_dot_ 을 쓴다.
      if (!pn_init_) {
        losf_x_ = los_x; losf_y_ = los_y;
        losf_x_dot_ = 0.0; losf_y_dot_ = 0.0;
        pn_init_ = true;
      } else if (dt > 1e-6) {
        double xp = losf_x_ + losf_x_dot_ * dt;      // 예측
        double rx = los_x - xp;                       // 잔차(측정−예측)
        losf_x_ = xp + pn_alpha_ * rx;
        losf_x_dot_ += (pn_beta_ / dt) * rx;
        double yp = losf_y_ + losf_y_dot_ * dt;
        double ry = los_y - yp;
        losf_y_ = yp + pn_alpha_ * ry;
        losf_y_dot_ += (pn_beta_ / dt) * ry;
        // 검출이 튀면(표적 프레임 이탈/재획득) 추정치가 ±10 로 폭주해 드론이 날뛴다 → 안전 상한.
        losf_x_dot_ = std::clamp(losf_x_dot_, -3.0, 3.0);
        losf_y_dot_ = std::clamp(losf_y_dot_, -3.0, 3.0);
      }

      if (guidance_mode_ == 1) {
        // ===== 비례항법 (Proportional Navigation) =====
        //   추미(현위치 추종)와 달리 '시선각속도(losf_dot)를 0으로' 만들도록 조종한다.
        //   → 자동으로 미래 충돌점을 앞질러 겨냥 = 가로지르는 빠른 표적에 훨씬 유리.
        //   횡가속(=기울기) 명령 = N×LOS각속도 + 약한 중심유지(표적 화면이탈 방지).
        //   LOS각속도 클램프로 노이즈 폭주 방지.
        double ldx = losf_x_dot_, ldy = losf_y_dot_;
        if (pn_los_clamp_ > 0.0) {
          ldx = std::clamp(ldx, -pn_los_clamp_, pn_los_clamp_);
          ldy = std::clamp(ldy, -pn_los_clamp_, pn_los_clamp_);
        }
        // ★ 종말 부스트: 신뢰거리가 commit_dist 이내면 기울기 한계를 높여(더 강한 횡가속)
        //   빠른 크로싱을 끝까지 추종해 박는다. (짧은 순간만이라 발진 안 함)
        terminal_active_ = distance_valid() && last_distance_ < pn_commit_dist_;
        double eff_max_tilt = terminal_active_ ? pn_commit_max_tilt_ : max_tilt_deg_;
        double rc = pn_nav_gain_ * ldx + pn_center_gain_ * los_x;
        double pc = pn_nav_gain_ * ldy + pn_center_gain_ * los_y;
        roll_deg  = roll_sign_  * std::clamp(rc, -eff_max_tilt, eff_max_tilt);
        pitch_deg = pitch_sign_ * std::clamp(pc, -eff_max_tilt, eff_max_tilt);
      } else {
        // ===== 추미 (Pursuit): PID + 시간기반 리드 (기존/검증됨) =====
        terminal_active_ = false;   // 종말 부스트는 PN 전용
        double emag_g = std::hypot(ex, ey);
        double gain_scale = 1.0;
        if (emag_g < 0.08)      gain_scale = 0.5;
        else if (emag_g < 0.20) gain_scale = 0.75;
        double t_lead = lead_gain_;
        if (distance_valid()) t_lead += lead_dist_ * std::min(last_distance_, 15.0);
        double ex_lead = ex + t_lead * err_dot_x_;
        double ey_lead = ey + t_lead * err_dot_y_;
        roll_deg = roll_sign_ * gain_scale * pid_roll_->compute(ex_lead, dt);
        double pitch_scale = 1.0;
        if (fabs(ex) > 0.12)      pitch_scale = 0.8;
        else if (fabs(ex) > 0.06) pitch_scale = 0.9;
        pitch_deg = pitch_sign_ * gain_scale * pitch_scale * pid_pitch_->compute(ey_lead, dt);
      }

      // ---- 추격 궤적(Pursuit): 조준하며 처음부터 상승 ----
      //   정렬 완료를 기다리지 않는다. 추력은 기체가 기울어진 방향(=공을 겨눈 방향)으로
      //   나가므로, "공쪽으로 기울이기(추적)" + "상승" 을 동시에 하면 드론이 공을 향해
      //   대각선으로 날아간다(추격). 공을 잘 겨눌수록(center_q↑) 그쪽으로 더 세게 상승하고,
      //   빗나가면 상승을 줄여(hover) 재조준을 우선한다. → 수직으로 떴다 늦게 꺾는 문제 해결.
      {
        const double up = track_throttle_;      // 잘 겨눴을 때 최대 상승(=공쪽 돌진)
        const double hover = hover_throttle_;   // 빗나갔을 때 최소(재조준)
        double emag = std::hypot(filt_err_x_, filt_err_y_);
        double center_q =
            std::clamp(1.0 - emag / std::max(pursuit_gate_, 1e-3), 0.0, 1.0);
        thrust = static_cast<float>(hover + (up - hover) * center_q);

        if (!align_locked_ && emag < 0.15) {
          align_locked_ = true;
          RCLCPP_INFO(get_logger(), "추격 시작 (오차 %.2f) → 공 향해 대각선 상승", emag);
        }
      }

      if (++dbg_count_ % 6 == 0) {
        RCLCPP_INFO(get_logger(),
                    "TRACK[%s] err=(%.2f,%.2f) losdot=(%.2f,%.2f) d=%.1fm roll=%.1f pitch=%.1f",
                    guidance_mode_ == 1 ? "PN" : "PUR",
                    filt_err_x_, filt_err_y_,
                    guidance_mode_ == 1 ? losf_x_dot_ : err_dot_x_,
                    guidance_mode_ == 1 ? losf_y_dot_ : err_dot_y_,
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
      terminal_active_ = false;
      pn_init_ = false;   // alpha-beta 추정기 재초기화 (다음 TRACK 진입 시)
      losf_x_ = losf_y_ = losf_x_dot_ = losf_y_dot_ = 0.0;
    }

    // ---- 유효 스위치 = 상태 스위치 && 재토글 래치 해제 ----
    //   모드 전환/부팅 직후엔 스위치가 올라가 있어도 즉시 적용 안 됨 → 재토글(DOWN→UP) 필요.
    //   · 자동: idle/search 게이트에 적용 (기본 IDLE, SEARCH 재토글 안전장치)
    //   · 수동: CH5 arm 에 적용 (기본 DISARM, arm 재토글 안전장치)
    //   ※ 자동 CH5 arm 자체는 스위치와 분리되어 상태머신 상태(auto_arm_states_)로 결정한다.
    const bool effective_arm = joy_arm_ && !require_arm_reset_;

    // ---- 상태 스위치 = idle/search 게이트 (auto 모드) ----
    //   자동 모드에서 이 스위치는 arm 과 분리되어 미션 상태(IDLE↔SEARCH)만 제어한다.
    //   재토글 안전장치 적용: 모드 전환/부팅 직후 스위치가 SEARCH(위)여도 기본 IDLE 이고,
    //   DISARM 으로 내렸다 다시 올려야 SEARCH 진입 (수동 arm 과 동일 매커니즘).
    if (!joy_manual_mode_) {
      if (effective_arm && state == State::IDLE && !auto_armed_) {
        auto_armed_ = true;
        sm_->arm();
        RCLCPP_INFO(get_logger(), "SEARCH ON (IDLE → SEARCH)");
      } else if (!effective_arm) {
        if (state != State::IDLE) {
          sm_->disarm();
          RCLCPP_INFO(get_logger(), "SEARCH OFF → IDLE");
        }
        auto_armed_ = false;
      }
    } else {
      // ---- 수동 모드: 자동 상태머신을 IDLE 로 고정 ----
      //   수동에서는 미션 상태가 의미 없으므로 IDLE 로 둔다(서보도 열림). 영상/방아쇠로
      //   미션이 멋대로 진행되는 것도 막는다. 자동으로 넘어오면 상태 스위치가 곧바로
      //   idle/search 를 제어한다(자동에선 스위치가 arm 과 분리되어 재토글 불필요).
      if (state != State::IDLE) {
        sm_->disarm();
        RCLCPP_INFO(get_logger(), "MANUAL → 자동 상태머신 IDLE 고정");
      }
      auto_armed_ = false;
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

    // ---- CH5(FC arm) 결정 ----
    //   수동: 스위치(effective_arm, 재토글 안전장치) 그대로.
    //   자동: 스위치와 분리 — 상태머신 상태 기반(auto_arm_states_)으로 컨트롤 노드가 결정.
    //   발행하는 mission_state.state 와 동일한 state 로 판정해 armed/state 를 일관되게 유지.
    const bool ch5_armed =
        joy_manual_mode_
            ? effective_arm
            : (auto_arm_states_.count(to_string(state)) > 0);

    // ---- CRSF output ----
    {
      CrsfOutput::Channels crsf{};
      crsf.fill(CrsfOutput::CRSF_MIN);

      if (joy_manual_mode_) {
        // 수동 스틱 → AETR 채널 매핑.
        //   L-stick: X=axes[0]=yaw,      Y=axes[1]=throttle
        //   R-stick: X=axes[2]=roll,     Y=axes[3]=pitch
        crsf[0] = CrsfOutput::norm_to_crsf(joy_axes_[2]);  // CH1 roll  = R-stick X
        crsf[1] = CrsfOutput::norm_to_crsf(joy_axes_[3]);  // CH2 pitch = R-stick Y
        crsf[2] = CrsfOutput::norm_to_crsf(joy_axes_[1]);  // CH3 thr   = L-stick Y (스프링, 중앙=50%)
        crsf[3] = CrsfOutput::norm_to_crsf(joy_axes_[0]);  // CH4 yaw   = L-stick X
      } else {
        // AUTO(영상유도) 출력.
        //   acro_mode : roll_deg/pitch_deg 를 '각속도[deg/s]' 명령으로 해석 → max_rate_dps 로
        //               정규화 → PX4 ACRO 가 그 각속도로 회전 (자동수평 없음, 민첩).
        //   아니면    : '각도[deg]' 명령 → max_angle 정규화 → PX4 Stabilized(자동수평).
        if (acro_mode_) {
          // ---- 자세 피드백 캐스케이드 (제대로 된 각속도 제어) ----
          //   비전 PID 출력(roll_deg)=목표 기울기[deg] → ±max_tilt 제한 →
          //   회전명령 = att_p×(목표기울기 − 실제기울기). 목표 도달 시 rate 0 → 기울기 '유지'.
          //   → 움직이는 공 속도에 맞는 steady lean 을 잡고 유지 = 뒤처짐 해결.
          // 종말 부스트: 근거리(commit_dist 이내)에선 기울기·자세게인을 순간 상향해
          //   빠른 크로싱을 끝까지 추종(박기). 짧은 순간이라 발진 없음.
          double eff_tilt  = terminal_active_ ? pn_commit_max_tilt_ : max_tilt_deg_;
          double eff_att_p = terminal_active_ ? pn_commit_att_p_     : att_p_;
          double roll_des  = std::clamp(roll_deg,  -eff_tilt, eff_tilt);
          double pitch_des = std::clamp(pitch_deg, -eff_tilt, eff_tilt);
          double roll_rate  = eff_att_p * (roll_des  - att_roll_deg_);
          double pitch_rate = eff_att_p * (pitch_des - att_pitch_deg_);
          crsf[0] = CrsfOutput::norm_to_crsf(roll_rate  / max_rate_dps_);
          crsf[1] = CrsfOutput::norm_to_crsf(pitch_rate / max_rate_dps_);
        } else {
          crsf[0] = CrsfOutput::norm_to_crsf(roll_deg / crsf_max_angle_);   // Stabilized: 각도
          crsf[1] = CrsfOutput::norm_to_crsf(pitch_deg / crsf_max_angle_);
        }
        crsf[2] = CrsfOutput::thr_to_crsf(static_cast<double>(thrust));
        crsf[3] = CrsfOutput::CRSF_CENTER;  // yaw 중앙 = yaw rate 0(헤딩 유지)
      }

      // CH5: arm — 수동은 스위치(재토글 안전장치), 자동은 상태머신 상태 기반.
      //   자동: 기본 발사(TRACK)부터 arm (control.auto_arm_states 로 조정).
      crsf[4] = ch5_armed ? CrsfOutput::CRSF_MAX : CrsfOutput::CRSF_MIN;

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
    msg.armed = ch5_armed;                // UI 효과음/표시용 (실제 CH5 arm 상태)
    msg.manual_mode = joy_manual_mode_;   // UI 효과음/표시용 (모드)
    pub_state_->publish(msg);

    // ---- 발사 잠금장치 서보 ----
    //   상태 전이 엣지로만 판정한다(방아쇠 버튼이 아니라 LOCK→TRACK 전이가 열림 트리거).
    //     · IDLE 진입            → OPEN (무조건 열림, 기체 장착/탈거)
    //     · IDLE→SEARCH (auto)   → LOCK (상승엣지, 발사기 고정)
    //     · LOCK→TRACK (발사)    → OPEN (기체 놓아줌)
    //   in-tick 전이(arm/disarm 등) 이후의 최신 상태로 판정한다.
    State servo_state = sm_->state();
    if (!servo_init_) {
      servo_init_ = true;
      servo_->open();  // 시작 상태 IDLE → OPEN
    } else if (servo_state != servo_prev_state_) {
      if (servo_state == State::IDLE) {
        servo_->open();
      } else if (servo_prev_state_ == State::IDLE &&
                 servo_state == State::SEARCH && !joy_manual_mode_) {
        servo_->lock();
      } else if (servo_prev_state_ == State::LOCK &&
                 servo_state == State::TRACK) {
        servo_->open();
      }
    }
    servo_prev_state_ = servo_state;
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<PIDController> pid_roll_;
  std::unique_ptr<PIDController> pid_pitch_;
  std::unique_ptr<ServoLock> servo_;

  // 서보 전이감지 상태 (기존 prev_state_ 는 매 틱 덮어써져 전이감지에 못 씀 → 별도 멤버).
  State servo_prev_state_{State::IDLE};
  bool  servo_init_{false};

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

  // 자동 모드에서 CH5(FC arm)를 켜는 상태 집합 (스위치와 분리, 컨트롤 노드가 결정)
  std::set<std::string> auto_arm_states_;

  // Joy state
  std::array<float, 4> joy_axes_{};
  bool joy_kill_{false};
  bool joy_arm_{false};
  bool joy_manual_mode_{false};
  std::array<int, 4> prev_btn_{};
  // ARM 재토글 안전장치: true 면 ARM 스위치가 올라가 있어도 arm 안 됨.
  //   부팅 시 true(스위치 이미 올라가 있어도 즉시 arm 방지), 모드 전환 시 true,
  //   ARM 스위치를 DISARM 으로 내리면 false. → 재토글해야만 arm.
  bool require_arm_reset_{true};

  // CRSF 송수신 상태
  std::unique_ptr<CrsfOutput> crsf_out_;                          // CRSF UART 송수신기를 소유한다.
  double crsf_max_angle_{35.0};                                  // 자동 제어 각도의 CRSF 정규화 기준을 둔다.
  std::array<bool, 256> crsf_seen_types_{};                       // 처음 발견한 텔레메트리 타입을 구분한다.
  std::size_t crsf_rx_bytes_{0};                                 // 누적 UART 수신량을 센다.
  std::size_t crsf_rx_valid_frames_{0};                          // 정상 텔레메트리 프레임 수를 센다.
  std::size_t crsf_rx_echoes_{0};                                // 자체 송신 에코 프레임 수를 센다.
  std::size_t crsf_rx_crc_errors_{0};                            // CRC 오류 프레임 수를 센다.
  std::size_t crsf_rx_framing_errors_{0};                        // 길이 오류 프레임 수를 센다.
  // 자동요격 제어(ACRO 각속도 + 자세 피드백 + 유도)
  bool   acro_mode_{true};
  double max_rate_dps_{120.0};
  double hover_throttle_{0.62};
  double pursuit_gate_{0.25};
  double lead_dot_alpha_{0.2};
  double lead_clamp_{0.6};
  double att_p_{6.0};
  double max_tilt_deg_{35.0};
  double att_roll_sign_{1.0};
  double att_pitch_sign_{-1.0};
  double att_comp_{0.0};
  double lead_dist_{0.02};
  // 유도(Guidance): 추미(0) / 비례항법 PN(1)
  int    guidance_mode_{0};
  double pn_nav_gain_{150.0};
  double pn_center_gain_{40.0};
  double pn_alpha_{0.5};
  double pn_beta_{0.15};
  double pn_los_clamp_{1.5};
  double pn_commit_dist_{5.0};
  double pn_commit_clamp_{0.5};
  double pn_commit_att_p_{6.5};
  double pn_commit_max_tilt_{58.0};
  bool   terminal_active_{false};   // 종말 부스트 활성(PN 근거리) → 캐스케이드가 참조
  // alpha-beta 표적 상태추정 (매끈한 LOS + LOS각속도)
  double losf_x_{0.0}, losf_y_{0.0};
  double losf_x_dot_{0.0}, losf_y_dot_{0.0};
  bool   pn_init_{false};
  double att_roll_deg_{0.0};    // 브리지에서 받은 드론 실제 자세
  double att_pitch_deg_{0.0};

  rclcpp::Subscription<arms_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_distance_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_range_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_hit_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_attitude_;
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
