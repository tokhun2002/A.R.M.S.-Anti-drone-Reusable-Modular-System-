#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

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
    declare_parameter("control.roll_pid.output_limit", 30.0);
    declare_parameter("control.pitch_pid.kp",          15.0);
    declare_parameter("control.pitch_pid.ki",          0.5);
    declare_parameter("control.pitch_pid.kd",          1.0);
    declare_parameter("control.pitch_pid.output_limit",30.0);
    declare_parameter("control.throttle",              0.55);
    declare_parameter("control.control_rate_hz",       30.0);

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

    throttle_       = get_parameter("control.throttle").as_double();
    control_rate_hz_= get_parameter("control.control_rate_hz").as_double();

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
        sm_->on_distance(msg->range);
      });

    // SITL: convert single-beam LaserScan to distance
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/arms/scan_raw", best_effort_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty()) {
          float dist = msg->ranges[0];
          if (dist >= msg->range_min && dist <= msg->range_max) {
            sm_->on_distance(dist);
          }
        }
      });

    pub_state_ = create_publisher<arms_msgs::msg::MissionState>("/arms/mission_state", 10);

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

    if (state == State::LOCK || state == State::TRACK || state == State::FIRE) {
      roll_deg  = pid_roll_->compute(sm_->current_error_x(), dt);
      pitch_deg = pid_pitch_->compute(sm_->current_error_y(), dt);
      thrust    = static_cast<float>(throttle_);
    } else {
      pid_roll_->reset();
      pid_pitch_->reset();
      thrust = (state == State::SEARCH) ? static_cast<float>(throttle_) : 0.f;
    }

    // ---- Send to MAVLink stream ----
    AttitudeCmd cmd;
    cmd.roll_deg  = static_cast<float>(roll_deg);
    cmd.pitch_deg = static_cast<float>(pitch_deg);
    cmd.yaw_deg   = 0.f;
    cmd.thrust    = thrust;
    mav_->set_attitude_command(cmd);

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
  double control_rate_hz_{30.0};

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
  rclcpp::Publisher<arms_msgs::msg::MissionState>::SharedPtr      pub_state_;

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
