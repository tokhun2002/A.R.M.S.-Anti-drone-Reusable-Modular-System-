#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "arms_msgs/msg/bounding_box.hpp"

namespace arms_control {

enum class State {
  IDLE,
  SEARCH,
  LOCK,
  BOOST,
  TRACK,
  FIRE,
  RTL,
};

std::string to_string(State s);

struct SMParams {
  double confidence_threshold{0.65};
  double lock_duration_sec{2.0};
  int    lost_frames_threshold{10};
  double lock_box_tolerance{0.15};
  double fire_distance_m{5.0};
};

class StateMachine {
public:
  using LogFn = std::function<void(const std::string &)>;

  explicit StateMachine(const SMParams & params, LogFn log_fn = nullptr);

  // ---------- external events ----------
  void on_detection(const std::vector<arms_msgs::msg::BoundingBox> & detections);
  void on_launch_button();
  void on_boost_complete();
  void on_distance(double distance_m);
  void on_fire_complete();
  void arm();
  void disarm();
  void on_landed();
  void force_search();   // 외부 RESET: 어떤 상태든 SEARCH 로 강제 복귀

  // ---------- accessors ----------
  State       state()             const { return state_; }
  double      lock_elapsed_sec()  const { return lock_elapsed_sec_; }
  double      current_error_x()   const { return error_x_; }
  double      current_error_y()   const { return error_y_; }
  bool        target_locked()     const { return target_locked_; }

private:
  void transition(State next);
  void on_target_lost();
  const arms_msgs::msg::BoundingBox * best_detection(
    const std::vector<arms_msgs::msg::BoundingBox> & dets) const;

  SMParams params_;
  LogFn    log_fn_;

  State  state_{State::IDLE};
  double error_x_{0.0};
  double error_y_{0.0};
  double lock_elapsed_sec_{0.0};
  bool   target_locked_{false};
  int    lost_frame_count_{0};

  using Clock     = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  bool       lock_timer_running_{false};
  TimePoint  lock_start_time_;
};

}  // namespace arms_control
