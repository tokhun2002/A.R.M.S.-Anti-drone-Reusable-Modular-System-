#include "arms_control/mavlink_interface.hpp"

#include <chrono>
#include <thread>

namespace arms_control {

MavlinkInterface::MavlinkInterface(const std::string & connection_url, LogFn log_fn)
: connection_url_(connection_url), log_fn_(std::move(log_fn))
{}

MavlinkInterface::~MavlinkInterface()
{
  stop_offboard_stream();
}

bool MavlinkInterface::connect()
{
  log("Connecting to " + connection_url_ + " ...");

  mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
    mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});

  auto result = mavsdk_->add_any_connection(connection_url_);
  if (result != mavsdk::ConnectionResult::Success) {
    log("Connection failed: " + std::to_string(static_cast<int>(result)));
    return false;
  }

  // Wait for a system (drone) to appear
  log("Waiting for system ...");
  while (mavsdk_->systems().empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  system_   = mavsdk_->systems().at(0);
  action_   = std::make_unique<mavsdk::Action>(system_);
  offboard_ = std::make_unique<mavsdk::Offboard>(system_);

  connected_ = true;
  log("System connected.");
  return true;
}

// ---------------------------------------------------------------------------
// Offboard stream
// ---------------------------------------------------------------------------

void MavlinkInterface::start_offboard_stream(float rate_hz)
{
  if (stream_active_) return;

  // Set an initial setpoint so MAVSDK doesn't complain
  mavsdk::Offboard::Attitude initial{};
  initial.roll_deg  = 0.f;
  initial.pitch_deg = 0.f;
  initial.yaw_deg   = 0.f;
  initial.thrust_value = 0.f;
  offboard_->set_attitude(initial);

  stream_active_ = true;
  stream_thread_ = std::thread(&MavlinkInterface::stream_loop, this, rate_hz);
  log("Offboard stream started at " + std::to_string(rate_hz) + " Hz.");
}

void MavlinkInterface::stop_offboard_stream()
{
  stream_active_ = false;
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }
}

void MavlinkInterface::stream_loop(float rate_hz)
{
  auto interval = std::chrono::duration<double>(1.0 / rate_hz);

  while (stream_active_) {
    mavsdk::Offboard::Attitude cmd{};
    cmd.roll_deg     = cmd_roll_.load();
    cmd.pitch_deg    = cmd_pitch_.load();
    cmd.yaw_deg      = cmd_yaw_.load();
    cmd.thrust_value = cmd_thrust_.load();

    offboard_->set_attitude(cmd);

    std::this_thread::sleep_for(interval);
  }
}

void MavlinkInterface::set_attitude_command(const AttitudeCmd & cmd)
{
  cmd_roll_.store(cmd.roll_deg);
  cmd_pitch_.store(cmd.pitch_deg);
  cmd_yaw_.store(cmd.yaw_deg);
  cmd_thrust_.store(std::clamp(cmd.thrust, 0.f, 1.f));
}

// ---------------------------------------------------------------------------
// Mode & arming
// ---------------------------------------------------------------------------

void MavlinkInterface::arm()
{
  if (!connected_) return;
  auto result = action_->arm();
  if (result != mavsdk::Action::Result::Success) {
    log("Arm failed: " + std::to_string(static_cast<int>(result)));
  } else {
    log("Armed.");
  }
}

void MavlinkInterface::disarm()
{
  if (!connected_) return;
  action_->disarm();
  log("Disarmed.");
}

void MavlinkInterface::set_offboard_mode()
{
  if (!connected_) return;
  auto result = offboard_->start();
  if (result != mavsdk::Offboard::Result::Success) {
    log("OFFBOARD start failed: " + std::to_string(static_cast<int>(result)));
  } else {
    log("OFFBOARD mode started.");
  }
}

void MavlinkInterface::send_rtl()
{
  if (!connected_) return;
  auto result = action_->return_to_launch();
  if (result != mavsdk::Action::Result::Success) {
    log("RTL failed: " + std::to_string(static_cast<int>(result)));
  } else {
    log("RTL command sent.");
  }
}

void MavlinkInterface::trigger_payload()
{
  if (!connected_) return;
  // Actuate servo on AUX1 (index 0 in MAVSDK actuator control group 0, channel 4+)
  // Adjust channel index per hardware wiring.
  mavsdk::Action::ActuatorControlGroup group{};
  group.controls[4] = 1.f;   // AUX1 → max (fire)
  action_->set_actuator_control(0, group);
  log("Payload trigger sent.");
}

void MavlinkInterface::log(const std::string & msg) const
{
  if (log_fn_) log_fn_("[MAVLink] " + msg);
}

}  // namespace arms_control
