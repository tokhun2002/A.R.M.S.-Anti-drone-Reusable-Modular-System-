#include <Arduino.h>

/*
  ============================================================
  ESP32-S3 N16R8 USB Controller Transmitter
  ============================================================

  - 짐벌 아날로그 축 4개 읽기
  - 스위치 4개 읽기
  - 1차 LPF와 중앙 데드밴드 적용
  - USB CDC를 통해 Jetson으로 100 Hz 전송

  Jetson 장치:
  /dev/ttyACM0

  패킷 형식:
  CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill\n

  Arduino IDE:
  Tools -> USB CDC On Boot -> Enabled
*/

// ============================================================
// USB CDC 설정
// ============================================================

static constexpr uint32_t SERIAL_BAUD = 115200;

// true이면 CTRL 패킷 대신 RAW ADC 값을 출력한다.
static constexpr bool RAW_DEBUG_MODE = false;

// ============================================================
// 짐벌 핀
// ============================================================

static constexpr int PIN_ROLL     = 1;
static constexpr int PIN_PITCH    = 2;
static constexpr int PIN_THROTTLE = 4;
static constexpr int PIN_YAW      = 5;

// ============================================================
// 스위치 핀
// ============================================================

static constexpr int PIN_KILL  = 15;
static constexpr int PIN_ELAND = 16;
static constexpr int PIN_MODE  = 17;
static constexpr int PIN_FIRE  = 18;

// ============================================================
// 샘플링 및 필터
// ============================================================

static constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // 100 Hz
static constexpr float LPF_ALPHA = 0.82f;
static constexpr int CENTER_DEADBAND = 30;
static constexpr uint32_t DEBOUNCE_MS = 30;

// ============================================================
// 축 설정
// ============================================================

struct AxisConfig {
  int pin;
  int raw_min;
  int raw_mid;
  int raw_max;
  bool invert;
  float filtered_raw;
};

AxisConfig axis_roll = {
  PIN_ROLL,
  525,
  2048,
  3800,
  false,
  2048.0f
};

AxisConfig axis_pitch = {
  PIN_PITCH,
  300,
  1931,
  3758,
  true,
  1931.0f
};

AxisConfig axis_throttle = {
  PIN_THROTTLE,
  307,
  1931,
  3562,
  false,
  307.0f
};

AxisConfig axis_yaw = {
  PIN_YAW,
  347,
  1917,
  3800,
  false,
  1917.0f
};

// ============================================================
// 버튼 설정
// ============================================================

struct ButtonState {
  int pin;
  bool stable_pressed;
  bool last_reading_pressed;
  uint32_t last_change_ms;
};

ButtonState btn_kill  = {PIN_KILL,  false, false, 0};
ButtonState btn_eland = {PIN_ELAND, false, false, 0};
ButtonState btn_mode  = {PIN_MODE,  false, false, 0};
ButtonState btn_fire  = {PIN_FIRE,  false, false, 0};

// ============================================================
// 상태 변수
// ============================================================

uint32_t seq_count = 0;

bool kill_state = false;
bool eland_state = false;
bool mode_state = false;
bool fire_state = false;

char tx_packet[128];

// ============================================================
// 값 변환
// ============================================================

long mapConstrainLong(
  long x,
  long in_min,
  long in_max,
  long out_min,
  long out_max
) {
  if (in_min == in_max) {
    return out_min;
  }

  if (in_min < in_max) {
    x = constrain(x, in_min, in_max);
  } else {
    x = constrain(x, in_max, in_min);
  }

  return
    (x - in_min) * (out_max - out_min) /
    (in_max - in_min) + out_min;
}

// ============================================================
// 축 읽기
// ============================================================

void updateAxisLPF(AxisConfig &axis) {
  const int raw = analogRead(axis.pin);

  axis.filtered_raw =
    LPF_ALPHA * axis.filtered_raw +
    (1.0f - LPF_ALPHA) * static_cast<float>(raw);
}

int readCenterAxis(AxisConfig &axis) {
  updateAxisLPF(axis);

  const int raw = static_cast<int>(axis.filtered_raw + 0.5f);
  int value = 0;

  if (raw >= axis.raw_mid) {
    value = static_cast<int>(
      mapConstrainLong(
        raw,
        axis.raw_mid,
        axis.raw_max,
        0,
        1000
      )
    );
  } else {
    value = static_cast<int>(
      mapConstrainLong(
        raw,
        axis.raw_min,
        axis.raw_mid,
        -1000,
        0
      )
    );
  }

  if (axis.invert) {
    value = -value;
  }

  if (abs(value) < CENTER_DEADBAND) {
    value = 0;
  }

  return constrain(value, -1000, 1000);
}

int readThrottleAxis(AxisConfig &axis) {
  updateAxisLPF(axis);

  const int raw = static_cast<int>(axis.filtered_raw + 0.5f);

  int value = static_cast<int>(
    mapConstrainLong(
      raw,
      axis.raw_min,
      axis.raw_max,
      0,
      1000
    )
  );

  if (axis.invert) {
    value = 1000 - value;
  }

  return constrain(value, 0, 1000);
}

// ============================================================
// 버튼 처리
// ============================================================

void updateButton(ButtonState &button) {
  const bool reading_pressed = (digitalRead(button.pin) == LOW);
  const uint32_t now_ms = millis();

  if (reading_pressed != button.last_reading_pressed) {
    button.last_reading_pressed = reading_pressed;
    button.last_change_ms = now_ms;
  }

  if ((uint32_t)(now_ms - button.last_change_ms) < DEBOUNCE_MS) {
    return;
  }

  button.stable_pressed = reading_pressed;
}

void updateSwitchLogic() {
  updateButton(btn_kill);
  updateButton(btn_eland);
  updateButton(btn_mode);
  updateButton(btn_fire);

  kill_state = btn_kill.stable_pressed;
  eland_state = btn_eland.stable_pressed;
  mode_state = btn_mode.stable_pressed;
  fire_state = btn_fire.stable_pressed;
}

// ============================================================
// 디버그 출력
// ============================================================

void printRawDebug() {
  Serial.printf(
    "RAW,%d,%d,%d,%d,%d,%d,%d,%d\n",
    analogRead(PIN_ROLL),
    analogRead(PIN_PITCH),
    analogRead(PIN_THROTTLE),
    analogRead(PIN_YAW),
    digitalRead(PIN_KILL),
    digitalRead(PIN_ELAND),
    digitalRead(PIN_MODE),
    digitalRead(PIN_FIRE)
  );
}

// ============================================================
// USB 패킷 전송
// ============================================================

void sendControllerPacket(
  int roll,
  int pitch,
  int throttle,
  int yaw,
  bool kill,
  bool eland,
  bool mode,
  bool fire
) {
  const int length = snprintf(
    tx_packet,
    sizeof(tx_packet),
    "CTRL,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
    static_cast<unsigned long>(seq_count),
    throttle,
    roll,
    pitch,
    yaw,
    fire ? 1 : 0,
    mode ? 1 : 0,
    eland ? 1 : 0,
    kill ? 1 : 0
  );

  if (length <= 0 || length >= static_cast<int>(sizeof(tx_packet))) {
    return;
  }

  Serial.write(
    reinterpret_cast<const uint8_t *>(tx_packet),
    static_cast<size_t>(length)
  );

  seq_count++;
}

// ============================================================
// setup
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  analogReadResolution(12);

  analogSetPinAttenuation(PIN_ROLL, ADC_11db);
  analogSetPinAttenuation(PIN_PITCH, ADC_11db);
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
  analogSetPinAttenuation(PIN_YAW, ADC_11db);

  pinMode(PIN_KILL, INPUT_PULLUP);
  pinMode(PIN_ELAND, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_FIRE, INPUT_PULLUP);

  axis_roll.filtered_raw = analogRead(PIN_ROLL);
  axis_pitch.filtered_raw = analogRead(PIN_PITCH);
  axis_throttle.filtered_raw = analogRead(PIN_THROTTLE);
  axis_yaw.filtered_raw = analogRead(PIN_YAW);
}

// ============================================================
// loop
// ============================================================

void loop() {
  static uint32_t last_sample_us = 0;

  const uint32_t now_us = micros();

  if ((uint32_t)(now_us - last_sample_us) < SAMPLE_PERIOD_US) {
    return;
  }

  last_sample_us += SAMPLE_PERIOD_US;

  if (RAW_DEBUG_MODE) {
    printRawDebug();
    return;
  }

  const int roll = readCenterAxis(axis_roll);
  const int pitch = readCenterAxis(axis_pitch);
  const int throttle = readThrottleAxis(axis_throttle);
  const int yaw = readCenterAxis(axis_yaw);

  updateSwitchLogic();

  sendControllerPacket(
    roll,
    pitch,
    throttle,
    yaw,
    kill_state,
    eland_state,
    mode_state,
    fire_state
  );
}