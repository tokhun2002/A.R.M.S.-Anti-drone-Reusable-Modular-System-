#include "arms_controller_input/gpio_input.hpp"

#include <stdexcept>
#include <string>

#ifdef HAS_GPIOD
#include <gpiod.h>
#endif

namespace arms_controller_input
{

struct GpioInput::Impl
{
#ifdef HAS_GPIOD
  gpiod_chip * chip = nullptr;
  std::array<gpiod_line *, 3> lines = {nullptr, nullptr, nullptr};
#endif
};

GpioInput::GpioInput(std::string chip_name, std::array<int, 3> line_offsets, bool active_low, bool request_pullup)
: chip_name_(std::move(chip_name)),
  line_offsets_(line_offsets),
  active_low_(active_low),
  request_pullup_(request_pullup),
  impl_(new Impl())
{
}

GpioInput::~GpioInput()
{
  closeDevice();
  delete impl_;
  impl_ = nullptr;
}

void GpioInput::openDevice()
{
#ifdef HAS_GPIOD
  if (impl_->chip != nullptr) {
    return;
  }

  for (int offset : line_offsets_) {
    if (offset < 0) {
      throw std::runtime_error(
        "GPIO line offset is not set. Jetson 40-pin BOARD pin number is not the same as libgpiod line offset. Check with gpioinfo/gpiofind first.");
    }
  }

  impl_->chip = gpiod_chip_open_by_name(chip_name_.c_str());
  if (impl_->chip == nullptr) {
    throw std::runtime_error("Failed to open GPIO chip: " + chip_name_);
  }

  int flags = 0;
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
  if (request_pullup_) {
    flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
  }
#else
  (void)request_pullup_;
#endif

  for (int i = 0; i < 3; ++i) {
    impl_->lines[i] = gpiod_chip_get_line(impl_->chip, static_cast<unsigned int>(line_offsets_[i]));
    if (impl_->lines[i] == nullptr) {
      closeDevice();
      throw std::runtime_error("Failed to get GPIO line offset: " + std::to_string(line_offsets_[i]));
    }

    const int rc = gpiod_line_request_input_flags(impl_->lines[i], "arms_controller_input", flags);
    if (rc < 0) {
      closeDevice();
      throw std::runtime_error("Failed to request GPIO input line offset: " + std::to_string(line_offsets_[i]));
    }
  }
#else
  throw std::runtime_error("This package was built without libgpiod. Install libgpiod-dev and rebuild, or run with fake_mode=true.");
#endif
}

void GpioInput::closeDevice()
{
#ifdef HAS_GPIOD
  if (impl_ == nullptr) {
    return;
  }

  for (auto & line : impl_->lines) {
    if (line != nullptr) {
      gpiod_line_release(line);
      line = nullptr;
    }
  }

  if (impl_->chip != nullptr) {
    gpiod_chip_close(impl_->chip);
    impl_->chip = nullptr;
  }
#endif
}

bool GpioInput::isOpen() const
{
#ifdef HAS_GPIOD
  return impl_ != nullptr && impl_->chip != nullptr;
#else
  return false;
#endif
}

int GpioInput::readLine(int index) const
{
#ifdef HAS_GPIOD
  if (impl_ == nullptr || impl_->lines[index] == nullptr) {
    throw std::runtime_error("GPIO line is not open");
  }

  const int value = gpiod_line_get_value(impl_->lines[index]);
  if (value < 0) {
    throw std::runtime_error("Failed to read GPIO line");
  }

  // Wiring: switch connects GPIO to GND.
  // raw LOW(0) means pressed/ON when active_low is true.
  if (active_low_) {
    return value == 0 ? 1 : 0;
  }
  return value ? 1 : 0;
#else
  (void)index;
  throw std::runtime_error("GPIO support is not compiled in");
#endif
}

SwitchState GpioInput::readSwitches()
{
  SwitchState state;
  state.kill = readLine(0);
  state.land = readLine(1);
  state.mode = readLine(2);
  return state;
}

}  // namespace arms_controller_input
