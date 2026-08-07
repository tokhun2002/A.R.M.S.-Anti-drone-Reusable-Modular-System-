#include "arms_control/state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace arms_control {

std::string to_string(State s)
{
  switch (s) {
    case State::IDLE:   return "IDLE";
    case State::SEARCH: return "SEARCH";
    case State::LOCK:   return "LOCK";
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
    auto now = Clock::now();
    if (last_detection_valid_) {
      double elapsed =
        std::chrono::duration<double>(now - last_detection_time_).count();
      if (elapsed < params_.detection_timeout_sec) {
        // 홀드: error_x_/error_y_ 유지, I 동결 플래그만 세움
        detection_held_ = true;
        return;
      }
    }
    // 타임아웃 또는 한 번도 검출 없음 → 실패 처리
    detection_held_ = false;
    target_locked_ = false;
    error_x_ = 0.0;
    error_y_ = 0.0;
    on_detection_timeout();
    return;
  }

  // Valid detection
  detection_held_ = false;
  auto now = Clock::now();
  last_detection_time_  = now;
  last_detection_valid_ = true;

  // Compute normalized pixel error from frame center
  error_x_ = static_cast<double>(best->x_center) - 0.5;
  error_y_ = static_cast<double>(best->y_center) - 0.5;

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
    transition(State::TRACK);
  } else if (log_fn_) {
    log_fn_("[StateMachine] Launch button pressed in state " +
            to_string(state_) + " — ignored.");
  }
}

void StateMachine::on_distance(double distance_m)
{
  if (state_ != State::TRACK) return;
  // 무효(-1/0) 라이다 값으로는 절대 발사 안 함. (-1 < fire_distance 라 예전엔 즉시 가짜명중)
  if (!(distance_m > 0.0)) return;

  // FIRE 는 "요격거리 이내" + "공이 화면 중앙 근처(정렬)" 둘 다 만족해야 발생.
  double emag = std::hypot(error_x_, error_y_);
  if (distance_m < params_.fire_distance_m &&
      emag < params_.fire_align_tol) {
    transition(State::FIRE);
  }
}

void StateMachine::on_external_hit()
{
  // 심판이 드론-풍선 실제 충돌을 감지 → 라이다와 무관하게 명중 처리(FIRE).
  if (state_ == State::TRACK || state_ == State::LOCK) {
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
    lock_timer_running_   = false;
    detection_held_       = false;
    last_detection_valid_ = false;
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
  lock_timer_running_   = false;
  lock_elapsed_sec_     = 0.0;
  target_locked_        = false;
  detection_held_       = false;
  last_detection_valid_ = false;
  error_x_              = 0.0;
  error_y_              = 0.0;
  transition(State::SEARCH);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void StateMachine::on_detection_timeout()
{
  if (state_ == State::SEARCH ||
      state_ == State::LOCK   ||
      state_ == State::TRACK)
  {
    if (state_ != State::SEARCH) {
      transition(State::SEARCH);
    }
    lock_timer_running_ = false;
    lock_elapsed_sec_   = 0.0;
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
