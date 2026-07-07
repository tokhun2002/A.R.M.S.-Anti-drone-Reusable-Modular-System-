#pragma once

#include <cstdint>
#include <string>

namespace arms_command
{

class ADS1115
{
public:
  ADS1115(std::string device = "/dev/i2c-1", int address = 0x48);
  ~ADS1115();

  ADS1115(const ADS1115 &) = delete;
  ADS1115 & operator=(const ADS1115 &) = delete;

  void openDevice();
  void closeDevice();
  bool isOpen() const;

  // Read ADS1115 single-ended channel 0~3.
  // Returns the signed 16-bit ADC conversion register value.
  // With PGA ±4.096 V and 3.3 V input, expected range is roughly 0~26400.
  int16_t readSingleEnded(int channel);

private:
  void writeRegister(uint8_t reg, uint16_t value);
  uint16_t readRegister(uint8_t reg);

  std::string device_;
  int address_;
  int fd_;
};

}  // namespace arms_command
