#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace arms_control {

struct AttitudeCmd {
  float roll_deg{0.f};
  float pitch_deg{0.f};
  float yaw_deg{0.f};
  float thrust{0.f};   // [0.0, 1.0]
};

class MavlinkInterface {
public:
  using LogFn = std::function<void(const std::string &)>;

  MavlinkInterface(const std::string & connection_url, LogFn log_fn = nullptr);
  ~MavlinkInterface();

  /** Connect and wait for heartbeat. Returns false on timeout. */
  bool connect();
  bool is_connected() const { return connected_; }

  /**
   * Start streaming attitude setpoints in a background thread.
   * Must be called BEFORE set_offboard_mode() for PX4 to accept OFFBOARD.
   */
  void start_offboard_stream(float rate_hz = 30.f);
  void stop_offboard_stream();

  /** Update the attitude command sent by the stream thread. Thread-safe. */
  void set_attitude_command(const AttitudeCmd & cmd);

  void arm();
  void disarm();
  void set_offboard_mode();
  void send_rtl();
  void trigger_payload();

private:
  void stream_loop(float rate_hz);
  void log(const std::string & msg) const;

  std::string connection_url_;
  LogFn       log_fn_;

  std::unique_ptr<mavsdk::Mavsdk>    mavsdk_;
  std::shared_ptr<mavsdk::System>    system_;
  std::unique_ptr<mavsdk::Action>    action_;
  std::unique_ptr<mavsdk::Offboard>  offboard_;

  bool connected_{false};

  // Stream thread
  std::thread         stream_thread_;
  std::atomic<bool>   stream_active_{false};

  // Shared attitude command (lock-free via atomic struct copy trick is not
  // reliable for non-trivial types, so use a simple mutex-free approach with
  // double-buffering by keeping pod floats as atomics)
  std::atomic<float>  cmd_roll_{0.f};
  std::atomic<float>  cmd_pitch_{0.f};
  std::atomic<float>  cmd_yaw_{0.f};
  std::atomic<float>  cmd_thrust_{0.f};
};

}  // namespace arms_control
