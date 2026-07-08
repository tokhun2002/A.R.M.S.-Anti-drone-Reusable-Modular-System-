#include "arms_control/crsf_output.hpp"

#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace arms_control {

CrsfOutput::CrsfOutput(std::string port) : port_(std::move(port)) {}

CrsfOutput::~CrsfOutput() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool CrsfOutput::try_open() {
  // O_RDWR works for both real UART and PTY (virtual serial via socat)
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tty{};
  tcgetattr(fd_, &tty);
  cfmakeraw(&tty);
  // B460800 is the closest standard baud to ELRS 420000.
  // For Jetson real UART, custom baud via ioctl can be set separately.
  cfsetospeed(&tty, B460800);
  cfsetispeed(&tty, B460800);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CREAD | CLOCAL;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
}

bool CrsfOutput::send(const Channels & ch) {
  if (fd_ < 0 && !try_open()) return false;

  auto frame = build_frame(ch);
  ssize_t n = write(fd_, frame.data(), frame.size());
  if (n < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return n == static_cast<ssize_t>(frame.size());
}

uint16_t CrsfOutput::norm_to_crsf(double v) {
  v = std::clamp(v, -1.0, 1.0);
  if (v >= 0.0) {
    return static_cast<uint16_t>(CRSF_CENTER + v * (CRSF_MAX - CRSF_CENTER));
  }
  return static_cast<uint16_t>(CRSF_CENTER + v * (CRSF_CENTER - CRSF_MIN));
}

uint16_t CrsfOutput::thr_to_crsf(double v) {
  v = std::clamp(v, 0.0, 1.0);
  return static_cast<uint16_t>(CRSF_MIN + v * (CRSF_MAX - CRSF_MIN));
}

uint8_t CrsfOutput::crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; ++i) {
    crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
  }
  return crc;
}

std::array<uint8_t, 26> CrsfOutput::build_frame(const Channels & ch) {
  std::array<uint8_t, 26> f{};
  f[0] = 0xC8;  // sync / FC address
  f[1] = 24;    // length: type(1) + payload(22) + crc(1)
  f[2] = 0x16;  // RC channels packed

  // Pack 16 × 11-bit channels into bytes f[3..24] (little-endian bit order)
  uint32_t bit = 0;
  for (int i = 0; i < 16; ++i) {
    uint32_t val = std::clamp<uint32_t>(ch[i], CRSF_MIN, CRSF_MAX);
    for (int b = 0; b < 11; ++b, ++bit) {
      if (val & (1u << b)) {
        f[3 + bit / 8] |= static_cast<uint8_t>(1u << (bit % 8));
      }
    }
  }

  // CRC over type + payload (f[2]..f[24] = 23 bytes)
  uint8_t crc = 0;
  for (int i = 2; i < 25; ++i) {
    crc = crc8_dvb_s2(crc, f[i]);
  }
  f[25] = crc;
  return f;
}

}  // namespace arms_control
