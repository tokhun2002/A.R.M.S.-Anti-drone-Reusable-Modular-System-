#include "arms_command/ads1115.hpp"
#include "arms_command/gpio_input.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace arms_command
{

class ArmsCommandNode : public rclcpp::Node
{
public:
  ArmsCommandNode()
  : Node("arms_command_hw_node")
  {
    fake_mode_ = declare_parameter<bool>("fake_mode", true);
    topic_name_ = declare_parameter<std::string>("topic_name", "/arms/command");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    i2c_device_ = declare_parameter<std::string>("i2c.device", "/dev/i2c-1");
    i2c_address_ = declare_parameter<int>("i2c.address", 0x48);

    gpio_chip_ = declare_parameter<std::string>("gpio.chip", "gpiochip0");
    gpio_active_low_ = declare_parameter<bool>("gpio.active_low", true);
    gpio_request_pullup_ = declare_parameter<bool>("gpio.request_pullup", true);

    raw_min_ = vectorToArray(declare_parameter<std::vector<int64_t>>("adc.raw_min", {0, 0, 0, 0}));
    raw_center_ = vectorToArray(declare_parameter<std::vector<int64_t>>("adc.raw_center", {13200, 13200, 13200, 13200}));
    raw_max_ = vectorToArray(declare_parameter<std::vector<int64_t>>("adc.raw_max", {26400, 26400, 26400, 26400}));
    invert_axes_ = vectorToBoolArray(declare_parameter<std::vector<bool>>("adc.invert_axes", {false, false, false, false}));

    const auto gpio_lines_vec = declare_parameter<std::vector<int64_t>>("gpio.lines", {-1, -1, -1, -1});
    gpio_lines_ = vectorTo4Array(gpio_lines_vec);

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(topic_name_, 10);

    if (!fake_mode_) {
      ads1115_ = std::make_unique<ADS1115>(i2c_device_, i2c_address_);
      gpio_ = std::make_unique<GpioInput>(gpio_chip_, gpio_lines_, gpio_active_low_, gpio_request_pullup_);

      ads1115_->openDevice();
      gpio_->openDevice();

      RCLCPP_INFO(get_logger(), "Real hardware mode enabled: ADS1115 %s addr=0x%02X, GPIO chip=%s",
        i2c_device_.c_str(), i2c_address_, gpio_chip_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "fake_mode=true: publishing simulated ADS1115 raw values and simulated switch states.");
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControllerInputNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Publishing sensor_msgs/msg/Joy on %s at %.1f Hz", topic_name_.c_str(), publish_rate_hz_);
  }

private:
  static std::array<int, 4> vectorToArray(const std::vector<int64_t> & values)
  {
    if (values.size() != 4) {
      throw std::runtime_error("ADC parameter array must have exactly 4 elements");
    }
    return {
      static_cast<int>(values[0]),
      static_cast<int>(values[1]),
      static_cast<int>(values[2]),
      static_cast<int>(values[3])
    };
  }

  static std::array<bool, 4> vectorToBoolArray(const std::vector<bool> & values)
  {
    if (values.size() != 4) {
      throw std::runtime_error("invert_axes parameter array must have exactly 4 elements");
    }
    return {values[0], values[1], values[2], values[3]};
  }

  static std::array<int, 4> vectorTo4Array(const std::vector<int64_t> & values)
  {
    if (values.size() != 4) {
      throw std::runtime_error("GPIO line parameter array must have exactly 4 elements");
    }
    return {
      static_cast<int>(values[0]),
      static_cast<int>(values[1]),
      static_cast<int>(values[2]),
      static_cast<int>(values[3])
    };
  }

  float normalizeAxis(int raw, int raw_min, int raw_center, int raw_max, bool invert) const
  {
    float value = 0.0F;

    if (raw >= raw_center) {
      const int denom = std::max(1, raw_max - raw_center);
      value = static_cast<float>(raw - raw_center) / static_cast<float>(denom);
    } else {
      const int denom = std::max(1, raw_center - raw_min);
      value = -static_cast<float>(raw_center - raw) / static_cast<float>(denom);
    }

    value = std::clamp(value, -1.0F, 1.0F);
    return invert ? -value : value;
  }

  std::array<int, 4> readAdcRaw()
  {
    if (fake_mode_) {
      return fakeAdcRaw();
    }

    std::array<int, 4> raw{};
    for (int ch = 0; ch < 4; ++ch) {
      raw[ch] = static_cast<int>(ads1115_->readSingleEnded(ch));
    }
    return raw;
  }

  SwitchState readSwitches()
  {
    if (fake_mode_) {
      return fakeSwitches();
    }
    return gpio_->readSwitches();
  }

  std::array<int, 4> fakeAdcRaw()
  {
    fake_t_ += 1.0 / std::max(1.0, publish_rate_hz_);

    constexpr int center = 13200;
    constexpr int min_raw = 0;
    constexpr int max_raw = 26400;

    const int left_x = center + static_cast<int>(7000.0 * std::sin(fake_t_ * 1.2));
    const int left_y = center + static_cast<int>(7000.0 * std::cos(fake_t_ * 1.1));
    const int right_x = center + static_cast<int>(6500.0 * std::sin(fake_t_ * 0.8));

    const double throttle01 = 0.5 * (std::sin(fake_t_ * 0.35) + 1.0);
    const int throttle = min_raw + static_cast<int>(throttle01 * static_cast<double>(max_raw - min_raw));

    return {
      std::clamp(left_x, min_raw, max_raw),
      std::clamp(left_y, min_raw, max_raw),
      std::clamp(right_x, min_raw, max_raw),
      std::clamp(throttle, min_raw, max_raw)
    };
  }

  SwitchState fakeSwitches() const
  {
    const int sec = static_cast<int>(std::floor(fake_t_));
    SwitchState state;
    state.kill   = 0;
    state.land   = ((sec / 6) % 2 == 1) ? 1 : 0;
    state.mode   = ((sec / 4) % 2 == 1) ? 1 : 0;
    state.launch = 0;
    return state;
  }

  void timerCallback()
  {
    try {
      const auto raw = readAdcRaw();
      const auto switches = readSwitches();

      sensor_msgs::msg::Joy msg;
      msg.header.stamp = now();
      msg.header.frame_id = "controller_input";
      msg.axes.resize(4);
      msg.buttons.resize(4);

      for (int i = 0; i < 4; ++i) {
        msg.axes[i] = normalizeAxis(raw[i], raw_min_[i], raw_center_[i], raw_max_[i], invert_axes_[i]);
      }

      msg.buttons[0] = switches.kill;
      msg.buttons[1] = switches.land;
      msg.buttons[2] = switches.mode;
      msg.buttons[3] = switches.launch;

      joy_pub_->publish(msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "raw=[%d,%d,%d,%d] axes=[%.2f,%.2f,%.2f,%.2f] buttons=[%d,%d,%d,%d]",
        raw[0], raw[1], raw[2], raw[3],
        msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3],
        msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "controller_input read failed: %s", e.what());
    }
  }

  bool fake_mode_;
  std::string topic_name_;
  double publish_rate_hz_;

  std::string i2c_device_;
  int i2c_address_;

  std::string gpio_chip_;
  std::array<int, 4> gpio_lines_;
  bool gpio_active_low_;
  bool gpio_request_pullup_;

  std::array<int, 4> raw_min_;
  std::array<int, 4> raw_center_;
  std::array<int, 4> raw_max_;
  std::array<bool, 4> invert_axes_;

  double fake_t_ = 0.0;

  std::unique_ptr<ADS1115> ads1115_;
  std::unique_ptr<GpioInput> gpio_;

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arms_command

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arms_command::ArmsCommandNode>());
  rclcpp::shutdown();
  return 0;
}
