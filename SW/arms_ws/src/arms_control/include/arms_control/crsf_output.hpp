#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace arms_control {

class CrsfOutput {
public:
  static constexpr uint16_t CRSF_MIN    = 172;
  static constexpr uint16_t CRSF_CENTER = 992;
  static constexpr uint16_t CRSF_MAX    = 1811;

  using Channels = std::array<uint16_t, 16>;

  explicit CrsfOutput(std::string port);
  ~CrsfOutput();

  // Send CRSF RC frame. Opens port lazily; returns false if port unavailable.
  bool send(const Channels & ch);

  // Normalized value (-1..1) → CRSF units
  static uint16_t norm_to_crsf(double v);
  // Throttle (0..1) → CRSF units
  static uint16_t thr_to_crsf(double v);

private:
  std::string port_;
  int fd_{-1};

  bool try_open();
  static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
  static std::array<uint8_t, 26> build_frame(const Channels & ch);
};

}  // namespace arms_control
