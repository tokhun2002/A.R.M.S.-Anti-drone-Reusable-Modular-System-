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
  TRACK,
  FIRE,
  RTL,
};

std::string to_string(State s);

struct SMParams {
  double confidence_threshold{0.65};
  double lock_duration_sec{2.0};
  double detection_timeout_sec{1.0};
  double lock_box_tolerance{0.15};
  double fire_align_tol{0.2};   // FIRE 조건: 이 오차 이내로 '중앙 정렬'돼야 (가짜명중 방지)
  double tau_fire_sec{0.3};     // 비전 looming: 충돌까지 시간(τ) 이 값 미만이면 발사
  double loom_s_min{0.1};       // FIRE 최소 bbox 크기(정규화). 원거리/노이즈 오탐 차단
};

class StateMachine {
public:
  using LogFn = std::function<void(const std::string &)>;

  explicit StateMachine(const SMParams & params, LogFn log_fn = nullptr);

  // ---------- external events ----------
  void on_detection(const std::vector<arms_msgs::msg::BoundingBox> & detections);
  void on_launch_button();
  // 비전 looming(τ) 기반 FIRE 판정. τ=충돌까지 시간, bbox_size=현재 표적 크기(정규화).
  // τ 는 스케일 불변 → 표적 실제크기·카메라 스펙 무관(작은 x500도 큰 풍선도 동일).
  void on_looming(double tau_sec, double bbox_size);
  void on_fire_complete();
  void on_hit();   // 심판 직격 명중(/arms/hit) → RTL (SITL: 영상 FIRE 대신)
  void arm();
  void disarm();
  void on_landed();
  void force_search();   // 외부 RESET: 어떤 상태든 SEARCH 로 강제 복귀
  void set_tau_fire(double t)      { params_.tau_fire_sec = t; }     // 런타임 τ 임계 조정
  void set_loom_s_min(double s)    { params_.loom_s_min = s; }       // 런타임 최소크기 게이트

  // ---------- accessors ----------
  State       state()               const { return state_; }
  double      lock_elapsed_sec()    const { return lock_elapsed_sec_; }
  double      current_error_x()     const { return error_x_; }
  double      current_error_y()     const { return error_y_; }
  bool        target_locked()       const { return target_locked_; }
  /** 직전 유효 검출로부터 timeout 이내 홀드 중이면 true. PID I-freeze 판단에 사용. */
  bool        is_detection_held()   const { return detection_held_; }

private:
  void transition(State next);
  void on_detection_timeout();
  const arms_msgs::msg::BoundingBox * best_detection(
    const std::vector<arms_msgs::msg::BoundingBox> & dets) const;

  SMParams params_;
  LogFn    log_fn_;

  State  state_{State::IDLE};
  double error_x_{0.0};
  double error_y_{0.0};
  double lock_elapsed_sec_{0.0};
  bool   target_locked_{false};
  bool   detection_held_{false};

  using Clock     = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  bool       lock_timer_running_{false};
  TimePoint  lock_start_time_;

  bool       last_detection_valid_{false};
  TimePoint  last_detection_time_;
};

}  // namespace arms_control
