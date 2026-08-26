#include "arms_control/servo_lock.hpp"

#include <fstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>   // access(), W_OK

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
  }

  // export 직후 pwmN 파일들은 잠깐 root:root(쓰기 불가)이고, udev(99-gpio.rules)가
  // gpio 그룹 권한을 적용하는 데 시간이 걸린다. **존재(enable)만이 아니라 실제
  // 쓰기 가능(W_OK)까지 기다린다** → 부팅 시 arms_control 이 udev 보다 먼저 떠도
  // 권한 적용을 기다렸다가 쓰므로 "sysfs 쓰기 실패"로 서보가 죽는 레이스를 없앤다.
  const std::string duty_path = pwm_dir_ + "/duty_cycle";
  bool writable = false;
  for (int i = 0; i < 300; ++i) {   // 최대 ~3s
    if (path_exists(pwm_dir_ + "/enable") &&
        ::access(duty_path.c_str(), W_OK) == 0) {
      writable = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!writable) {
    log("[ServoLock] sysfs PWM 쓰기 권한 대기 시간초과 (" + pwm_dir_ +
        ") — udev(99-gpio.rules)/pinmux/gpio 그룹 확인. 서보 명령 무시.");
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
    const char* what = (duty_ns == params_.lock_duty_ns) ? " (LOCK)"
                     : (duty_ns == params_.open_duty_ns) ? " (OPEN)" : "";
    log("[ServoLock] duty=" + std::to_string(duty_ns) + "ns" + what);  // 실제 PWM 쓰기마다 로그
  } else {
    log("[ServoLock] duty_cycle 쓰기 실패.");
  }
}

void ServoLock::lock() { set_duty(params_.lock_duty_ns); }
void ServoLock::open() { set_duty(params_.open_duty_ns); }

}  // namespace arms_control
