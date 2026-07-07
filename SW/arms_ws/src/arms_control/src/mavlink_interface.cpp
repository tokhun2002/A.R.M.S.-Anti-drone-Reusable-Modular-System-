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
    mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation});

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

  // [옵션B] Telemetry 시작 → PX4 현재 local NED 위치 구독
  telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
  telemetry_->subscribe_position_velocity_ned(
    [this](mavsdk::Telemetry::PositionVelocityNed pv) {
      cur_north_.store(pv.position.north_m);
      cur_east_.store(pv.position.east_m);
      cur_down_.store(pv.position.down_m);
      have_pos_.store(true);
    });

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
    if (pos_mode_.load()) {
      // [옵션B] 위치명령 모드 (NED) — PX4 위치제어기가 목표점에 수렴
      mavsdk::Offboard::PositionNedYaw p{};
      p.north_m = pos_north_.load();
      p.east_m  = pos_east_.load();
      p.down_m  = pos_down_.load();
      p.yaw_deg = pos_yaw_.load();
      offboard_->set_position_ned(p);
    } else if (vel_mode_.load()) {
      // [옵션B] 속도명령 모드 (NED)
      mavsdk::Offboard::VelocityNedYaw v{};
      v.north_m_s = vel_north_.load();
      v.east_m_s  = vel_east_.load();
      v.down_m_s  = vel_down_.load();
      v.yaw_deg   = vel_yaw_.load();
      offboard_->set_velocity_ned(v);
    } else {
      mavsdk::Offboard::Attitude cmd{};
      cmd.roll_deg     = cmd_roll_.load();
      cmd.pitch_deg    = cmd_pitch_.load();
      cmd.yaw_deg      = cmd_yaw_.load();
      cmd.thrust_value = cmd_thrust_.load();
      offboard_->set_attitude(cmd);
    }
    std::this_thread::sleep_for(interval);
  }
}

void MavlinkInterface::set_attitude_command(const AttitudeCmd & cmd)
{
  vel_mode_.store(false);   // [옵션B] attitude 명령 들어오면 자세모드로 복귀
  pos_mode_.store(false);
  cmd_roll_.store(cmd.roll_deg);
  cmd_pitch_.store(cmd.pitch_deg);
  cmd_yaw_.store(cmd.yaw_deg);
  cmd_thrust_.store(std::clamp(cmd.thrust, 0.f, 1.f));
}

void MavlinkInterface::set_velocity_ned(float north_m_s, float east_m_s,
                                        float down_m_s, float yaw_deg)
{
  vel_north_.store(north_m_s);
  vel_east_.store(east_m_s);
  vel_down_.store(down_m_s);
  vel_yaw_.store(yaw_deg);
  vel_mode_.store(true);   // velocity 모드 진입
}

void MavlinkInterface::set_position_ned(float north_m, float east_m,
                                        float down_m, float yaw_deg)
{
  pos_north_.store(north_m);
  pos_east_.store(east_m);
  pos_down_.store(down_m);
  pos_yaw_.store(yaw_deg);
  pos_mode_.store(true);
  vel_mode_.store(false);
}

bool MavlinkInterface::get_position_ned(float & north_m, float & east_m, float & down_m)
{
  if (!have_pos_.load()) return false;
  north_m = cur_north_.load();
  east_m  = cur_east_.load();
  down_m  = cur_down_.load();
  return true;
}

void MavlinkInterface::use_attitude_mode()
{
  vel_mode_.store(false);
  pos_mode_.store(false);
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

bool MavlinkInterface::set_offboard_mode()
{
  if (!connected_) return false;
  // EKF 수렴/스트림 타이밍에 따라 1회 start() 는 랜덤 실패(result 7).
  // 성공할 때까지 최대 20회 재시도(사이 0.5s 대기) → 항상 OFFBOARD 진입 보장.
  for (int attempt = 1; attempt <= 20; ++attempt) {
    auto result = offboard_->start();
    if (result == mavsdk::Offboard::Result::Success) {
      log("OFFBOARD mode started. (attempt " + std::to_string(attempt) + ")");
      return true;
    }
    log("OFFBOARD start failed: " + std::to_string(static_cast<int>(result))
        + " (attempt " + std::to_string(attempt) + "/20, 재시도)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  log("OFFBOARD 진입 최종 실패 — EKF/연결 확인 필요");
  return false;
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
  // set_actuator: index starts at 1, value range -1.0 ~ 1.0
  // Adjust index per hardware wiring (e.g. AUX1 = 1).
  auto result = action_->set_actuator(1, 1.f);
  if (result != mavsdk::Action::Result::Success) {
    log("Payload trigger failed: " + std::to_string(static_cast<int>(result)));
  } else {
    log("Payload trigger sent.");
  }
}

void MavlinkInterface::log(const std::string & msg) const
{
  if (log_fn_) log_fn_("[MAVLink] " + msg);
}

}  // namespace arms_control
