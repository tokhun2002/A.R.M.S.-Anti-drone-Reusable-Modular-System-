#include "arms_control/servo_lock.hpp"

#include <fstream>
#include <string>
#include <sys/stat.h>
#include <thread>

namespace arms_control {

ServoLock::ServoLock(const Params & params, LogFn log_fn)
: params_(params), log_fn_(std::move(log_fn)) {}

void ServoLock::log(const std::string & msg) const {
  if (log_fn_) log_fn_(msg);
}

bool ServoLock::write_file(const std::string & path, const std::string & value) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << value;
  ofs.flush();
  return ofs.good();
}

static bool path_exists(const std::string & path) {
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0;
}

void ServoLock::init() {
  if (!params_.enabled) {
    log("[ServoLock] disabled (servo.enabled=false) — 서보 제어 비활성.");
    available_ = false;
    return;
  }

  pwm_dir_ = params_.chip_path + "/pwm" + std::to_string(params_.channel);

  // 1) export (이미 export 됐으면 write 가 실패해도 무시하고 dir 존재로 판단).
  if (!path_exists(pwm_dir_)) {
    write_file(params_.chip_path + "/export", std::to_string(params_.channel));
    // udev 가 pwmN 디렉터리/권한을 만들 때까지 잠깐 대기 (최대 ~500ms).
    for (int i = 0; i < 50 && !path_exists(pwm_dir_ + "/enable"); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (!path_exists(pwm_dir_ + "/enable")) {
    log("[ServoLock] sysfs PWM 사용 불가 (" + pwm_dir_ +
        ") — pinmux/권한 확인. 서보 명령 무시.");
    available_ = false;
    return;
  }

  // 2) duty=0 → period → 초기 duty(open) → enable.
  //   (기존 duty>새 period 면 period 설정이 EINVAL 나므로 duty 를 먼저 0 으로.)
  bool ok = true;
  ok &= write_file(pwm_dir_ + "/duty_cycle", "0");
  ok &= write_file(pwm_dir_ + "/period", std::to_string(params_.period_ns));
  ok &= write_file(pwm_dir_ + "/duty_cycle", std::to_string(params_.open_duty_ns));
  ok &= write_file(pwm_dir_ + "/enable", "1");

  if (!ok) {
    log("[ServoLock] sysfs 쓰기 실패 (" + pwm_dir_ + ") — 권한 확인. 서보 명령 무시.");
    available_ = false;
    return;
  }

  current_duty_ns_ = params_.open_duty_ns;  // 시작 시 OPEN (IDLE 기본)
  available_ = true;
  log("[ServoLock] 준비 완료: " + pwm_dir_ +
      " period=" + std::to_string(params_.period_ns) +
      "ns lock=" + std::to_string(params_.lock_duty_ns) +
      "ns open=" + std::to_string(params_.open_duty_ns) + "ns (초기 OPEN)");
}

void ServoLock::set_duty(long duty_ns) {
  if (!available_) return;
  if (duty_ns == current_duty_ns_) return;  // 값 변화 시에만 기록
  if (write_file(pwm_dir_ + "/duty_cycle", std::to_string(duty_ns))) {
    current_duty_ns_ = duty_ns;
  } else {
    log("[ServoLock] duty_cycle 쓰기 실패.");
  }
}

void ServoLock::lock() { set_duty(params_.lock_duty_ns); }
void ServoLock::open() { set_duty(params_.open_duty_ns); }

}  // namespace arms_control
