#pragma once

#include <array>
#include <string>

namespace arms_command
{

struct SwitchState
{
  int kill   = 0;
  int land   = 0;
  int mode   = 0;
  int launch = 0;
};

class GpioInput
{
public:
  GpioInput(std::string chip_name, std::array<int, 4> line_offsets, bool active_low = true, bool request_pullup = true);
  ~GpioInput();

  GpioInput(const GpioInput &) = delete;
  GpioInput & operator=(const GpioInput &) = delete;

  void openDevice();
  void closeDevice();
  bool isOpen() const;

  SwitchState readSwitches();

private:
  int readLine(int index) const;

  std::string chip_name_;
  std::array<int, 4> line_offsets_;
  bool active_low_;
  bool request_pullup_;

  struct Impl;
  Impl * impl_;
};

}  // namespace arms_command
