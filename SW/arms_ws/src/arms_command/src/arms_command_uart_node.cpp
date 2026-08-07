#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace arms_command
{

using SteadyClock = std::chrono::steady_clock;

// ESP32 펌웨어가 보내는 CSV 패킷 필드(고정, 변경 불가).
//   throttle: 0..1000,  roll/pitch/yaw: -1000..1000,  버튼: 0/1
//   CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill
// control 노드 Joy 규약으로의 변환은 publishJoy 에서 수행한다.
struct ControllerPacket
{
  uint32_t seq = 0;
  int throttle = 0;
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int fire = 0;
  int mode = 0;
  int eland = 0;
  int kill = 0;
};

class ArmsCommandUartNode : public rclcpp::Node
{
public:
  ArmsCommandUartNode()
  : Node("arms_command_hw_node")
  {
    topic_name_ = declare_parameter<std::string>("topic_name", "/joy");

    serial_device_ =
      declare_parameter<std::string>("serial.device", "/dev/ttyACM0");
    serial_baud_ = declare_parameter<int>("serial.baud", 115200);
    poll_rate_hz_ = declare_parameter<double>("serial.poll_rate_hz", 200.0);
    max_line_length_ =
      declare_parameter<int>("serial.max_line_length", 128);

    failsafe_timeout_ms_ =
      declare_parameter<int>("failsafe.timeout_ms", 300);
    failsafe_publish_rate_hz_ =
      declare_parameter<double>("failsafe.publish_rate_hz", 20.0);

    if (poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("serial.poll_rate_hz must be greater than 0");
    }
    if (max_line_length_ < 32) {
      throw std::runtime_error("serial.max_line_length must be at least 32");
    }
    if (failsafe_timeout_ms_ <= 0) {
      throw std::runtime_error("failsafe.timeout_ms must be greater than 0");
    }
    if (failsafe_publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("failsafe.publish_rate_hz must be greater than 0");
    }

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      topic_name_,
      rclcpp::SensorDataQoS().keep_last(1)
    );

    openSerialPort();

    const auto now_steady = SteadyClock::now();
    last_valid_packet_time_ = now_steady;
    last_failsafe_publish_time_ = now_steady - std::chrono::seconds(1);

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ArmsCommandUartNode::pollSerial, this));

    RCLCPP_INFO(
      get_logger(),
      "USB controller node started: device=%s, baud=%d, topic=%s, poll=%.1f Hz",
      serial_device_.c_str(), serial_baud_, topic_name_.c_str(), poll_rate_hz_);
    RCLCPP_INFO(
      get_logger(),
      "Expected packet: CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill");
  }

  ~ArmsCommandUartNode() override
  {
    closeSerialPort();
  }

private:
  static speed_t baudToTermios(int baud)
  {
    switch (baud) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
#ifdef B230400
      case 230400:
        return B230400;
#endif
      default:
        throw std::runtime_error("Unsupported serial baud rate: " + std::to_string(baud));
    }
  }

  void openSerialPort()
  {
    serial_fd_ = ::open(
      serial_device_.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);

    if (serial_fd_ < 0) {
      throw std::runtime_error(
              "Failed to open " + serial_device_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      const std::string error = std::strerror(errno);
      closeSerialPort();
      throw std::runtime_error("tcgetattr failed: " + error);
    }

    cfmakeraw(&tty);

    const speed_t speed = baudToTermios(serial_baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8 data bits, no parity, 1 stop bit, no flow control (8N1).
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cflag |= CLOCAL | CREAD;

    // Non-blocking read. The ROS timer performs polling.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      const std::string error = std::strerror(errno);
      closeSerialPort();
      throw std::runtime_error("tcsetattr failed: " + error);
    }

    // ESP32-S3 USB CDC는 호스트가 DTR을 활성화해야 데이터를 내보내는 경우가 있다.
    // RTS는 비활성화하여 불필요한 리셋 신호를 피한다.
    int modem_bits = 0;
    if (::ioctl(serial_fd_, TIOCMGET, &modem_bits) == 0) {
      modem_bits |= TIOCM_DTR;
      modem_bits &= ~TIOCM_RTS;
      if (::ioctl(serial_fd_, TIOCMSET, &modem_bits) != 0) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to set DTR/RTS on %s: %s",
          serial_device_.c_str(), std::strerror(errno));
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Failed to read DTR/RTS state on %s: %s",
        serial_device_.c_str(), std::strerror(errno));
    }

    tcflush(serial_fd_, TCIOFLUSH);
  }

  void closeSerialPort()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void pollSerial()
  {
    readAvailableBytes();
    processLatestLine();   // 밀린 백로그는 버리고 최신 패킷만 처리 (제어 지연·이력 재생 방지)
    // 미완성 꼬리가 비정상적으로 커지면(프레이밍 붕괴) 버린다.
    constexpr std::size_t kMaximumRxBuffer = 4096;
    if (rx_buffer_.size() > kMaximumRxBuffer) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "USB serial buffer overflow (%zu bytes, no newline) — clearing.",
        rx_buffer_.size());
      rx_buffer_.clear();
    }
    publishFailsafeWhenTimedOut();
  }

  void readAvailableBytes()
  {
    if (serial_fd_ < 0) {
      return;
    }

    std::array<char, 256> buffer{};

    while (true) {
      const ssize_t count = ::read(serial_fd_, buffer.data(), buffer.size());

      if (count > 0) {
        rx_buffer_.append(buffer.data(), static_cast<std::size_t>(count));
        continue;
      }

      if (count == 0) {
        break;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "USB serial read failed on %s: %s",
        serial_device_.c_str(), std::strerror(errno));
      break;
    }
    // 버퍼 상한 처리는 processLatestLine 이후(최신 패킷 추출 후) pollSerial 에서 한다.
  }

  // 밀린 백로그(오래된 패킷)는 버리고 가장 최신 완성 패킷 1개만 처리한다.
  //   조종 입력은 "현재 스위치/스틱 위치"만 의미가 있으므로 과거 값을 재생할 필요가 없다.
  //   → 제어 지연 최소화 + 시작 시 누적된 과거 이력 재생(효과음 폭주)·오작동 방지.
  void processLatestLine()
  {
    const std::size_t last_nl = rx_buffer_.rfind('\n');
    if (last_nl == std::string::npos) {
      return;  // 아직 완성된 줄이 없음 → 다음 폴링까지 대기
    }

    // 마지막 '\n' 으로 끝나는(=가장 최신) 줄의 시작 위치를 찾는다.
    std::size_t line_start = 0;
    if (last_nl > 0) {
      const std::size_t prev_nl = rx_buffer_.rfind('\n', last_nl - 1);
      if (prev_nl != std::string::npos) {
        line_start = prev_nl + 1;
      }
    }

    std::string line = rx_buffer_.substr(line_start, last_nl - line_start);
    // 최신 줄 + 그 앞의 모든 백로그를 버리고, 미완성 꼬리만 남긴다.
    rx_buffer_.erase(0, last_nl + 1);

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      return;
    }
    if (line.size() > static_cast<std::size_t>(max_line_length_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding overlong USB serial line (%zu bytes)", line.size());
      return;
    }

    ControllerPacket packet;
    std::string parse_error;
    if (!parsePacket(line, packet, parse_error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding invalid USB serial packet: %s; line='%s'",
        parse_error.c_str(), line.c_str());
      return;
    }

    handleValidPacket(packet);
  }

  static std::vector<std::string_view> splitCsv(const std::string & line)
  {
    std::vector<std::string_view> fields;
    std::size_t begin = 0;

    while (true) {
      const std::size_t comma = line.find(',', begin);
      if (comma == std::string::npos) {
        fields.emplace_back(line.data() + begin, line.size() - begin);
        break;
      }

      fields.emplace_back(line.data() + begin, comma - begin);
      begin = comma + 1;
    }

    return fields;
  }

  template<typename IntegerT>
  static bool parseInteger(std::string_view text, IntegerT & value)
  {
    if (text.empty()) {
      return false;
    }

    const char * first = text.data();
    const char * last = text.data() + text.size();
    const auto result = std::from_chars(first, last, value);
    return result.ec == std::errc{} && result.ptr == last;
  }

  static bool inRange(int value, int minimum, int maximum)
  {
    return value >= minimum && value <= maximum;
  }

  static bool parsePacket(
    const std::string & line,
    ControllerPacket & packet,
    std::string & error)
  {
    const auto fields = splitCsv(line);

    // CTRL + 9 data fields = 10 CSV fields.
    if (fields.size() != 10) {
      error = "expected 10 CSV fields, got " + std::to_string(fields.size());
      return false;
    }

    if (fields[0] != "CTRL") {
      error = "packet prefix is not CTRL";
      return false;
    }

    if (!parseInteger(fields[1], packet.seq) ||
      !parseInteger(fields[2], packet.throttle) ||
      !parseInteger(fields[3], packet.roll) ||
      !parseInteger(fields[4], packet.pitch) ||
      !parseInteger(fields[5], packet.yaw) ||
      !parseInteger(fields[6], packet.fire) ||
      !parseInteger(fields[7], packet.mode) ||
      !parseInteger(fields[8], packet.eland) ||
      !parseInteger(fields[9], packet.kill))
    {
      error = "one or more fields are not valid integers";
      return false;
    }

    if (!inRange(packet.throttle, 0, 1000)) {
      error = "throttle is outside 0..1000";
      return false;
    }
    if (!inRange(packet.roll, -1000, 1000) ||
      !inRange(packet.pitch, -1000, 1000) ||
      !inRange(packet.yaw, -1000, 1000))
    {
      error = "roll, pitch, or yaw is outside -1000..1000";
      return false;
    }
    if (!inRange(packet.fire, 0, 1) ||
      !inRange(packet.mode, 0, 1) ||
      !inRange(packet.eland, 0, 1) ||
      !inRange(packet.kill, 0, 1))
    {
      error = "fire, mode, eland, and kill must be 0 or 1";
      return false;
    }

    return true;
  }

  void handleValidPacket(const ControllerPacket & packet)
  {
    if (have_last_seq_) {
      const uint32_t expected = last_seq_ + 1U;
      if (packet.seq != expected) {
        // 최신값만 쓰므로 오래된 패킷 건너뜀은 정상이다. 얼마나 밀리는지 진단용으로만 출력.
        const uint32_t skipped = (packet.seq >= expected) ? (packet.seq - expected) : 0U;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "USB serial: 최신 패킷만 사용 (오래된 패킷 ~%u개 건너뜀/초당 누적)",
          skipped);
      }
    }

    last_seq_ = packet.seq;
    have_last_seq_ = true;
    last_valid_packet_time_ = SteadyClock::now();

    if (failsafe_active_) {
      RCLCPP_INFO(get_logger(), "USB packets recovered. Failsafe released.");
      failsafe_active_ = false;
    }

    publishJoy(packet);
  }

  void publishJoy(const ControllerPacket & packet)
  {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();
    msg.header.frame_id = "controller_input";

    // ESP32 패킷 → control 노드(arms_control) Joy 규약으로 변환.
    //   axes[0]=yaw, axes[1]=throttle, axes[2]=roll, axes[3]=pitch  (모두 -1..1)
    //     throttle: ESP32 0..1000 → -1..1 (바닥 0 → -1 = 최소 스로틀)
    msg.axes = {
      static_cast<float>(packet.yaw) / 1000.0F,
      static_cast<float>(packet.throttle) / 500.0F - 1.0F,
      static_cast<float>(packet.roll) / 1000.0F,
      static_cast<float>(packet.pitch) / 1000.0F
    };

    //   buttons[0]=kill, [1]=arm, [2]=mode, [3]=launch
    //     실기체 배선상 kill 스위치와 상태(arm) 스위치가 뒤바뀌어 있어 두 입력을 스왑한다.
    //     → 물리 kill 스위치는 eland 필드로, 물리 상태/arm 스위치는 kill 필드로 들어온다.
    //     fire 버튼 → launch 로 매핑.
    msg.buttons = {
      packet.eland,   // buttons[0]=kill  (물리 kill 스위치)
      packet.kill,    // buttons[1]=arm   (물리 상태/arm 스위치)
      packet.mode,
      packet.fire
    };

    joy_pub_->publish(msg);
  }

  void publishFailsafeWhenTimedOut()
  {
    const auto now_steady = SteadyClock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_steady - last_valid_packet_time_).count();

    if (elapsed_ms <= failsafe_timeout_ms_) {
      return;
    }

    if (!failsafe_active_) {
      failsafe_active_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "No valid ESP32 packet for %lld ms. Publishing USB failsafe: throttle minimum, kill=1.",
        static_cast<long long>(elapsed_ms));
    }

    const auto failsafe_period = std::chrono::duration<double>(
      1.0 / failsafe_publish_rate_hz_);

    if ((now_steady - last_failsafe_publish_time_) < failsafe_period) {
      return;
    }

    ControllerPacket safe;
    safe.throttle = 0;
    safe.roll = 0;
    safe.pitch = 0;
    safe.yaw = 0;
    safe.fire = 0;
    safe.mode = 0;
    safe.eland = 0;
    safe.kill = 1;

    publishJoy(safe);
    last_failsafe_publish_time_ = now_steady;
  }

  std::string topic_name_;
  std::string serial_device_;
  int serial_baud_ = 115200;
  double poll_rate_hz_ = 200.0;
  int max_line_length_ = 128;

  int failsafe_timeout_ms_ = 300;
  double failsafe_publish_rate_hz_ = 20.0;

  int serial_fd_ = -1;
  std::string rx_buffer_;

  uint32_t last_seq_ = 0;
  bool have_last_seq_ = false;
  bool failsafe_active_ = false;

  SteadyClock::time_point last_valid_packet_time_;
  SteadyClock::time_point last_failsafe_publish_time_;

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace arms_command

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<arms_command::ArmsCommandUartNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("arms_command_uart"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}