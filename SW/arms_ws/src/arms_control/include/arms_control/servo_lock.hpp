#pragma once

#include <functional>
#include <string>

namespace arms_control {

// ---------------------------------------------------------------------------
// ServoLock
// ---------------------------------------------------------------------------
// 발사 잠금장치(클램프) 서보를 Jetson 하드웨어 PWM(sysfs)으로 구동한다.
// 원본 파이썬 드라이버(arms_command/servo/servo_motor.py, Jetson.GPIO)와 동일한
// 신호를 C++ 에서 sysfs 로 직접 써서 arms_control_node 에 통합했다.
//
// 매핑(JETSON_ORIN_NANO, gpio_pin_data.py 기준):
//   물리 핀 15 = GPIO12 / GP88_PWM1 → 3280000.pwm → /sys/class/pwm/pwmchip0, 채널 0.
//   50Hz(20ms) 주기, 90°=1.5ms(LOCK 기본), 180°=2.5ms(OPEN 기본).
//
// 전제: 핀15 PWM1 pinmux(jetson-io) + /sys/class/pwm 쓰기 권한(99-gpio.rules udev).
//       (README_SERVO_TEST.md 의 파이썬 경로와 동일 요건)
//
// sysfs 접근 실패(비-Jetson, pinmux 미설정, 권한 없음, enabled=false)에는
// 경고 1회만 남기고 이후 모든 명령을 무시(no-op)한다. → SITL/개발 PC 에서도
// 노드가 정상 기동한다.
// ---------------------------------------------------------------------------
class ServoLock {
 public:
  using LogFn = std::function<void(const std::string &)>;

  struct Params {
    bool        enabled{true};
    std::string chip_path{"/sys/class/pwm/pwmchip0"};
    int         channel{0};
    long        period_ns{20000000};    // 50Hz = 20ms
    long        lock_duty_ns{1500000};  // 90°  = 1.5ms (LOCK 기본)
    long        open_duty_ns{2500000};  // 180° = 2.5ms (OPEN 기본)
  };

  explicit ServoLock(const Params & params, LogFn log_fn = nullptr);

  // sysfs export → period → 초기 duty(open) → enable. 실패 시 available_=false.
  void init();

  void lock();   // duty = lock_duty_ns (클램프 잠금)
  void open();   // duty = open_duty_ns (클램프 해제)

  bool available() const { return available_; }

 private:
  // sysfs 파일에 문자열을 쓴다. 성공 여부 반환.
  bool write_file(const std::string & path, const std::string & value) const;
  // duty_cycle 갱신(값이 바뀔 때만 기록).
  void set_duty(long duty_ns);
  void log(const std::string & msg) const;

  Params      params_;
  LogFn       log_fn_;
  bool        available_{false};   // sysfs 구동 가능 여부
  std::string pwm_dir_;            // /sys/class/pwm/pwmchipN/pwmM
  long        current_duty_ns_{-1};  // 마지막으로 기록한 duty (-1=미기록)

  // 노드 종료 시 unexport/disable 하지 않는다:
  //   종료(정상/크래시) 시 클램프가 풀리면 안 되므로 마지막 위치를 유지한다.
};

}  // namespace arms_control
