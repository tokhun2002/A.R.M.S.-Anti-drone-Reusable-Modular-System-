#include "arms_command/ads1115.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace arms_command
{
namespace
{
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG = 0x01;

constexpr uint16_t OS_SINGLE = 0x8000;
constexpr uint16_t MUX_SINGLE_0 = 0x4000;
constexpr uint16_t MUX_SINGLE_1 = 0x5000;
constexpr uint16_t MUX_SINGLE_2 = 0x6000;
constexpr uint16_t MUX_SINGLE_3 = 0x7000;

// PGA = ±4.096 V. LSB = 4.096 / 32768 = 0.125 mV.
constexpr uint16_t PGA_4_096V = 0x0200;
constexpr uint16_t MODE_SINGLE_SHOT = 0x0100;
constexpr uint16_t DR_860SPS = 0x00E0;
constexpr uint16_t COMP_DISABLED = 0x0003;

uint16_t muxForChannel(int channel)
{
  switch (channel) {
    case 0: return MUX_SINGLE_0;
    case 1: return MUX_SINGLE_1;
    case 2: return MUX_SINGLE_2;
    case 3: return MUX_SINGLE_3;
    default:
      throw std::out_of_range("ADS1115 channel must be 0, 1, 2, or 3");
  }
}
}  // namespace

ADS1115::ADS1115(std::string device, int address)
: device_(std::move(device)), address_(address), fd_(-1)
{
}

ADS1115::~ADS1115()
{
  closeDevice();
}

void ADS1115::openDevice()
{
  if (fd_ >= 0) {
    return;
  }

  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open I2C device " + device_ + ": " + std::strerror(errno));
  }

  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    const std::string err = std::strerror(errno);
    closeDevice();
    throw std::runtime_error("Failed to select ADS1115 address 0x" +
      std::to_string(address_) + " on " + device_ + ": " + err);
  }
}

void ADS1115::closeDevice()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ADS1115::isOpen() const
{
  return fd_ >= 0;
}

void ADS1115::writeRegister(uint8_t reg, uint16_t value)
{
  if (fd_ < 0) {
    throw std::runtime_error("ADS1115 is not open");
  }

  const uint8_t buffer[3] = {
    reg,
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF)
  };

  if (::write(fd_, buffer, 3) != 3) {
    throw std::runtime_error("Failed to write ADS1115 register: " + std::string(std::strerror(errno)));
  }
}

uint16_t ADS1115::readRegister(uint8_t reg)
{
  if (fd_ < 0) {
    throw std::runtime_error("ADS1115 is not open");
  }

  if (::write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("Failed to set ADS1115 register pointer: " + std::string(std::strerror(errno)));
  }

  uint8_t buffer[2] = {0, 0};
  if (::read(fd_, buffer, 2) != 2) {
    throw std::runtime_error("Failed to read ADS1115 register: " + std::string(std::strerror(errno)));
  }

  return static_cast<uint16_t>((buffer[0] << 8) | buffer[1]);
}

int16_t ADS1115::readSingleEnded(int channel)
{
  const uint16_t config = OS_SINGLE |
    muxForChannel(channel) |
    PGA_4_096V |
    MODE_SINGLE_SHOT |
    DR_860SPS |
    COMP_DISABLED;

  writeRegister(REG_CONFIG, config);

  // 860 SPS conversion time is about 1.16 ms. 2 ms is enough and keeps the code simple.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  const uint16_t raw = readRegister(REG_CONVERSION);
  return static_cast<int16_t>(raw);
}

}  // namespace arms_command
