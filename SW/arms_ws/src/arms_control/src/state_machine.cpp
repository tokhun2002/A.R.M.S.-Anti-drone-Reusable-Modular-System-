#include "arms_control/state_machine.hpp"

#include <algorithm>
#include <chrono>

namespace arms_control {

std::string to_string(State s)
{
  switch (s) {
    case State::IDLE:   return "IDLE";
    case State::SEARCH: return "SEARCH";
    case State::LOCK:   return "LOCK";
    case State::BOOST:  return "BOOST";
    case State::TRACK:  return "TRACK";
    case State::FIRE:   return "FIRE";
    case State::RTL:    return "RTL";
  }
  return "UNKNOWN";
}

StateMachine::StateMachine(const SMParams & params, LogFn log_fn)
: params_(params), log_fn_(std::move(log_fn))
{}

// ---------------------------------------------------------------------------
// External events
// ---------------------------------------------------------------------------

void StateMachine::on_detection(
  const std::vector<arms_msgs::msg::BoundingBox> & detections)
{
  const auto * best = best_detection(detections);

  if (!best || best->confidence < static_cast<float>(params_.confidence_threshold)) {
    on_target_lost();
    target_locked_ = false;
    error_x_ = 0.0;
    error_y_ = 0.0;
    return;
  }

  // Compute normalized pixel error from frame center
  error_x_ = static_cast<double>(best->x_center) - 0.5;
  error_y_ = static_cast<double>(best->y_center) - 0.5;
  lost_frame_count_ = 0;

  auto now = Clock::now();

  if (state_ == State::SEARCH) {
    if (!lock_timer_running_) {
      lock_start_time_    = now;
      lock_timer_running_ = true;
    }
    lock_elapsed_sec_ =
      std::chrono::duration<double>(now - lock_start_time_).count();

    if (lock_elapsed_sec_ >= params_.lock_duration_sec) {
      transition(State::LOCK);
      target_locked_ = true;
    }
  } else if (state_ == State::LOCK ||
             state_ == State::BOOST ||
             state_ == State::TRACK ||
             state_ == State::FIRE) {
    if (lock_timer_running_) {
      lock_elapsed_sec_ =
        std::chrono::duration<double>(now - lock_start_time_).count();
    }
    target_locked_ = true;
  }
}

void StateMachine::on_launch_button()
{
  if (state_ == State::LOCK) {
    transition(State::TRACK);   // BOOST 없이 바로 위치보정 추적
  } else if (log_fn_) {
    log_fn_("[StateMachine] Launch button pressed in state " +
            to_string(state_) + " — ignored.");
  }
}

void StateMachine::on_boost_complete()
{
  if (state_ == State::BOOST) {
    transition(State::TRACK);   // 부스트 끝(또는 빗나감) → 위치보정
  }
}

void StateMachine::on_distance(double distance_m)
{
  if ((state_ == State::TRACK || state_ == State::BOOST) &&
      distance_m < params_.fire_distance_m) {
    transition(State::FIRE);
  }
}

void StateMachine::on_fire_complete()
{
  if (state_ == State::FIRE) {
    transition(State::RTL);
  }
}

void StateMachine::arm()
{
  if (state_ == State::IDLE) {
    lock_timer_running_ = false;
    lost_frame_count_   = 0;
    transition(State::SEARCH);
  }
}

void StateMachine::disarm()
{
  transition(State::IDLE);
}

void StateMachine::on_landed()
{
  if (state_ == State::RTL) {
    transition(State::IDLE);
  }
}

void StateMachine::force_search()
{
  // RESET: RTL/TRACK/FIRE 등 어떤 상태든 즉시 SEARCH 로.
  // 모든 락온/타이머/소실카운트 초기화해서 깨끗한 탐색 상태로 만든다.
  lock_timer_running_ = false;
  lock_elapsed_sec_   = 0.0;
  lost_frame_count_   = 0;
  target_locked_      = false;
  error_x_            = 0.0;
  error_y_            = 0.0;
  transition(State::SEARCH);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void StateMachine::on_target_lost()
{
  // BOOST 중에는 타겟 소실해도 발사 유지(committed). SEARCH/LOCK/TRACK 만 복귀 처리.
  if (state_ == State::SEARCH ||
      state_ == State::LOCK   ||
      state_ == State::TRACK)
  {
    ++lost_frame_count_;
    if (lost_frame_count_ >= params_.lost_frames_threshold) {
      if (state_ != State::SEARCH) {
        transition(State::SEARCH);
      }
      lock_timer_running_ = false;
      lock_elapsed_sec_   = 0.0;
      lost_frame_count_   = 0;
    }
  }
}

const arms_msgs::msg::BoundingBox * StateMachine::best_detection(
  const std::vector<arms_msgs::msg::BoundingBox> & dets) const
{
  if (dets.empty()) return nullptr;

  const arms_msgs::msg::BoundingBox * best = nullptr;
  for (const auto & d : dets) {
    if (!best || d.confidence > best->confidence) {
      best = &d;
    }
  }
  return best;
}

void StateMachine::transition(State next)
{
  if (log_fn_) {
    log_fn_("[StateMachine] " + to_string(state_) + " -> " + to_string(next));
  }
  state_ = next;
}

}  // namespace arms_control
