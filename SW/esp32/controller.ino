#include <Arduino.h>

/*
  ============================================================
  ESP32-S3 N16R8 Controller Transmitter
  ============================================================

  [역할]
  - 짐벌 2개에서 아날로그 축 4개 읽기
  - 스위치 4개 읽기
    1. Kill switch: 현재 ON/OFF 상태 전송
    2. Emergency Landing switch: 현재 ON/OFF 상태 전송
    3. Auto / Manual Mode switch: 현재 ON/OFF 상태 전송
    4. Fire switch: 한 번 누르면 1로 고정
  - 짐벌 값에 1차 LPF 적용
  - ESP32-S3 핀 UART로 Jetson에 조종기 신호 전송

  [Jetson 전송 패킷 형식]
  CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill

  예시:
  CTRL,1523,642,-35,120,8,1,1,0,0

  [값 범위]
  roll      : -1000 ~ 1000
  pitch     : -1000 ~ 1000
  throttle  : 0 ~ 1000
  yaw       : -1000 ~ 1000
  kill      : 0 or 1
  eland     : 0 or 1
  mode      : 0 = manual, 1 = auto
  fire      : 0 or 1

  [샘플링 / 전송 속도]
  SAMPLE_PERIOD_US = 10000 us
  즉, 10 ms마다 1번 전송
  전송 주파수 = 100 Hz


  ============================================================
  핀 연결 정리
  ============================================================

  [짐벌 전원]
  짐벌1 VCC -> ESP32-S3 3V3
  짐벌2 VCC -> ESP32-S3 3V3

  짐벌1 GND -> ESP32-S3 GND
  짐벌2 GND -> ESP32-S3 GND

  주의:
  ESP32-S3 ADC 핀에는 5V를 넣으면 안 됨.
  짐벌 출력 신호는 반드시 0 ~ 3.3V 범위여야 함.


  [짐벌 신호선 4개]
  Self-centering 짐벌 X축 -> GPIO1  : Roll
  Self-centering 짐벌 Y축 -> GPIO2  : Pitch

  Throttle 모드 짐벌 Y축 -> GPIO4  : Throttle
  Throttle 모드 짐벌 X축 -> GPIO5  : Yaw


  [스위치 4개]
  Kill switch:
    GPIO15 ---- 스위치 ---- GND

  Emergency Landing switch:
    GPIO16 ---- 스위치 ---- GND

  Auto / Manual Mode switch:
    GPIO17 ---- 스위치 ---- GND

  Fire switch:
    GPIO18 ---- 스위치 ---- GND

  코드에서 INPUT_PULLUP 사용.
  따라서:
    안 누름 = HIGH
    누름   = LOW


  [Jetson UART 통신]
  ESP32-S3 GPIO7 TX  -> Jetson RX, J12 pin 10
  ESP32-S3 GPIO6 RX  <- Jetson TX, J12 pin 8
  ESP32-S3 GND       <-> Jetson GND, J12 pin 6

  ESP32가 Jetson으로 보내기만 할 거면 최소 배선은:
  ESP32-S3 GPIO7 TX -> Jetson RX, J12 pin 10
  ESP32-S3 GND      -> Jetson GND

  하지만 나중에 Jetson에서 ESP32로 명령을 보낼 수도 있으므로
  GPIO6 RX도 같이 연결하는 것을 추천함.
*/


// ============================================================
// UART 설정
// ============================================================

// Jetson으로 보내는 UART.
// USB Serial은 디버그용으로 따로 사용함.
HardwareSerial JetsonSerial(1);

// ESP32-S3 UART 핀
const int JETSON_RX_PIN = 6;  // ESP32-S3 RX <- Jetson TX, Jetson J12 pin 8
const int JETSON_TX_PIN = 7;  // ESP32-S3 TX -> Jetson RX, Jetson J12 pin 10

const uint32_t SERIAL_BAUD = 115200;


// ============================================================
// 짐벌 핀 설정
// ============================================================

// ESP32-S3 ADC1 핀 사용
const int PIN_ROLL     = 1;  // Roll 짐벌 신호선
const int PIN_PITCH    = 2;  // Pitch 짐벌 신호선
const int PIN_THROTTLE = 4;  // Throttle 짐벌 신호선
const int PIN_YAW      = 5;  // Yaw 짐벌 신호선


// ============================================================
// 스위치 핀 설정
// ============================================================

const int PIN_KILL  = 15;  // Kill switch
const int PIN_ELAND = 16;  // Emergency Landing switch
const int PIN_MODE  = 17;  // Auto / Manual Mode switch
const int PIN_FIRE  = 18;  // Fire switch


// ============================================================
// 샘플링 설정
// ============================================================

// Jetson으로 조종기 신호를 보내는 속도
// 10000 us = 10 ms
// 1초에 100번 전송 = 100 Hz
const uint32_t SAMPLE_PERIOD_US = 10000;


// ============================================================
// LPF 설정
// ============================================================

// 1차 LPF:
// filtered[k] = alpha * filtered[k-1] + (1 - alpha) * raw[k]
//
// alpha가 클수록 부드러움.
// alpha가 작을수록 반응이 빠름.
// 100Hz 조종기 입력에서는 0.75 ~ 0.90 정도 추천.
const float LPF_ALPHA = 0.82f;

// 중앙 근처 노이즈 제거
// 정규화 후 -30 ~ 30이면 0으로 처리
const int CENTER_DEADBAND = 30;


// ============================================================
// 스위치 동작
// ============================================================
// Kill, E-Land, Mode:
//   실제 스위치 상태를 그대로 전송
//   OFF = 0, ON = 1
//
// Fire:
//   한 번 눌리면 1로 고정
//   ESP32를 재부팅하기 전까지 1 유지


// ============================================================
// 디버그 설정
// ============================================================

// true로 바꾸면 Jetson으로 CTRL 패킷을 보내지 않고
// USB Serial Monitor로 RAW ADC 값과 스위치 raw 상태를 출력함.
// 보정할 때만 true로 사용.
const bool RAW_DEBUG_MODE = false;

// true로 바꾸면 Jetson으로 보내는 CTRL 패킷을
// USB Serial Monitor에도 같이 출력함.
// 평상시에는 false 추천.
const bool ECHO_PACKET_TO_USB = false;


// ============================================================
// 축 보정값 설정
// ============================================================
// 아래 값은 확인된 조종기 출력 범위를 기준으로 1차 보정한 값.
// RAW_DEBUG_MODE = true로 실제 raw 값을 측정하면 더 정확하게 보정 가능.
//
// ESP32-S3 ADC 12bit 기준: 0 ~ 4095
// 실제 짐벌은 보통 0~4095 전체 범위를 다 쓰지 않음.
// ============================================================

struct AxisConfig {
  int pin;

  int raw_min;
  int raw_mid;
  int raw_max;

  bool invert;

  float filtered_raw;
};


// 아래 보정값은 사용자가 확인한 정규화 출력값을 기준으로 역산한 값임.
// 더 정확한 보정이 필요하면 RAW_DEBUG_MODE = true로 실제 ADC 값을 측정해서 교체.

// Roll 축
// 관측값: 중앙 약 0, 최소 약 -871, 최대 1000
AxisConfig axis_roll = {
  PIN_ROLL,
  525,       // 실제 왼쪽 끝 추정 raw
  2048,      // 중앙
  3800,      // 오른쪽 끝은 기존 범위에서 이미 1000 도달
  false,
  2048.0f
};


// Pitch 축
// 관측값: 중앙 약 +67, 최소 약 -976, 최대 1000
// 기존 invert=true를 유지하면서 실제 중앙을 약 1931로 보정
AxisConfig axis_pitch = {
  PIN_PITCH,
  300,       // 한쪽 끝은 기존 범위에서 이미 1000 도달
  1931,      // 실제 중앙 추정 raw
  3758,      // 반대쪽 끝 추정 raw
  true,
  1931.0f
};


// Throttle 축
// 관측값: 최저 약 0~3, 최고 약 932
AxisConfig axis_throttle = {
  PIN_THROTTLE,
  307,       // 스로틀 최저 추정 raw
  1931,      // throttle에서는 직접 사용하지 않음
  3562,      // 스로틀 최고 추정 raw
  false,
  307.0f
};


// Yaw 축
// 관측값: 중앙 약 -75, 최소 약 -973, 최대 1000
AxisConfig axis_yaw = {
  PIN_YAW,
  347,       // 왼쪽 끝 추정 raw
  1917,      // 실제 중앙 추정 raw
  3800,      // 오른쪽 끝은 기존 범위에서 이미 1000 도달
  false,
  1917.0f
};


// ============================================================
// 버튼 디바운싱 설정
// ============================================================

const uint32_t DEBOUNCE_MS = 30;

struct ButtonState {
  int pin;

  bool stable_pressed;
  bool last_reading_pressed;

  bool pressed_event;
  bool released_event;

  uint32_t last_change_ms;
};


ButtonState btn_kill = {
  PIN_KILL,
  false,
  false,
  false,
  false,
  0
};

ButtonState btn_eland = {
  PIN_ELAND,
  false,
  false,
  false,
  false,
  0
};

ButtonState btn_mode = {
  PIN_MODE,
  false,
  false,
  false,
  false,
  0
};

ButtonState btn_fire = {
  PIN_FIRE,
  false,
  false,
  false,
  false,
  0
};


// ============================================================
// 상태 변수
// ============================================================

uint32_t seq_count = 0;

// Kill, E-Land, Mode는 실제 스위치 상태를 그대로 사용
// OFF = 0, ON = 1
bool kill_state = false;
bool eland_state = false;
bool mode_state = false;

// Fire는 한 번 누르면 1로 고정
// 재부팅하기 전까지 1 유지
bool fire_latched = false;


// ============================================================
// 유틸 함수
// ============================================================

long mapConstrainLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_min == in_max) {
    return out_min;
  }

  if (in_min < in_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
  } else {
    if (x > in_min) x = in_min;
    if (x < in_max) x = in_max;
  }

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// ============================================================
// 짐벌 축 읽기 함수
// ============================================================

void updateAxisLPF(AxisConfig &axis) {
  int raw = analogRead(axis.pin);

  axis.filtered_raw =
    LPF_ALPHA * axis.filtered_raw +
    (1.0f - LPF_ALPHA) * raw;
}


// 중앙복귀 축용
// Roll, Pitch, Yaw에 사용
// 결과: -1000 ~ 1000
int readCenterAxis(AxisConfig &axis) {
  updateAxisLPF(axis);

  int raw = (int)(axis.filtered_raw + 0.5f);
  int value = 0;

  if (raw >= axis.raw_mid) {
    value = mapConstrainLong(raw, axis.raw_mid, axis.raw_max, 0, 1000);
  } else {
    value = mapConstrainLong(raw, axis.raw_min, axis.raw_mid, -1000, 0);
  }

  if (axis.invert) {
    value = -value;
  }

  if (abs(value) < CENTER_DEADBAND) {
    value = 0;
  }

  value = constrain(value, -1000, 1000);

  return value;
}


// 스로틀 축용
// 결과: 0 ~ 1000
int readThrottleAxis(AxisConfig &axis) {
  updateAxisLPF(axis);

  int raw = (int)(axis.filtered_raw + 0.5f);

  int value = mapConstrainLong(raw, axis.raw_min, axis.raw_max, 0, 1000);

  if (axis.invert) {
    value = 1000 - value;
  }

  value = constrain(value, 0, 1000);

  return value;
}


// ============================================================
// 버튼 읽기 함수
// ============================================================

void updateButton(ButtonState &btn) {
  btn.pressed_event = false;
  btn.released_event = false;

  // INPUT_PULLUP 기준:
  // 안 누름 = HIGH
  // 누름   = LOW
  bool reading_pressed = (digitalRead(btn.pin) == LOW);

  uint32_t now_ms = millis();

  if (reading_pressed != btn.last_reading_pressed) {
    btn.last_change_ms = now_ms;
    btn.last_reading_pressed = reading_pressed;
  }

  if ((now_ms - btn.last_change_ms) >= DEBOUNCE_MS) {
    if (reading_pressed != btn.stable_pressed) {
      bool prev_state = btn.stable_pressed;
      btn.stable_pressed = reading_pressed;

      if (!prev_state && btn.stable_pressed) {
        btn.pressed_event = true;
      }

      if (prev_state && !btn.stable_pressed) {
        btn.released_event = true;
      }
    }
  }
}


void updateSwitchLogic() {
  updateButton(btn_kill);
  updateButton(btn_eland);
  updateButton(btn_mode);
  updateButton(btn_fire);

  // Kill, E-Land, Mode:
  // 실제 스위치의 ON/OFF 상태를 그대로 사용
  kill_state  = btn_kill.stable_pressed;
  eland_state = btn_eland.stable_pressed;
  mode_state  = btn_mode.stable_pressed;

  // Fire:
  // 한 번 눌린 순간부터 계속 1 유지
  // 스위치를 다시 OFF로 돌려도 0으로 돌아가지 않음
  if (btn_fire.pressed_event) {
    fire_latched = true;
  }
}



// ============================================================
// 출력 함수
// ============================================================

void printRawDebug() {
  int raw_roll     = analogRead(PIN_ROLL);
  int raw_pitch    = analogRead(PIN_PITCH);
  int raw_throttle = analogRead(PIN_THROTTLE);
  int raw_yaw      = analogRead(PIN_YAW);

  Serial.print("RAW,");
  Serial.print(raw_roll);
  Serial.print(",");
  Serial.print(raw_pitch);
  Serial.print(",");
  Serial.print(raw_throttle);
  Serial.print(",");
  Serial.print(raw_yaw);
  Serial.print(",");

  // INPUT_PULLUP 기준:
  // 1 = 안 누름
  // 0 = 누름
  Serial.print(digitalRead(PIN_KILL));
  Serial.print(",");
  Serial.print(digitalRead(PIN_ELAND));
  Serial.print(",");
  Serial.print(digitalRead(PIN_MODE));
  Serial.print(",");
  Serial.println(digitalRead(PIN_FIRE));
}


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
  char packet[128];

  int len = snprintf(
    packet,
    sizeof(packet),
    "CTRL,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
    (unsigned long)seq_count,
    throttle,
    roll,
    pitch,
    yaw,
    fire ? 1 : 0,
    mode ? 1 : 0,
    eland ? 1 : 0,
    kill ? 1 : 0
  );

  if (len > 0) {
    // 실제 Jetson으로 나가는 UART
    JetsonSerial.write((uint8_t *)packet, len);

    // 디버깅용으로 USB Serial Monitor에도 같은 패킷 출력
    if (ECHO_PACKET_TO_USB) {
      Serial.write((uint8_t *)packet, len);
    }
  }

  seq_count++;
}


// ============================================================
// setup
// ============================================================

void setup() {
  // USB Serial: Arduino IDE Serial Monitor 디버그용
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Jetson UART: 실제 조종기 신호 전송용
  JetsonSerial.begin(
    SERIAL_BAUD,
    SERIAL_8N1,
    JETSON_RX_PIN,
    JETSON_TX_PIN
  );

  // ADC 해상도 설정
  // 12bit: 0 ~ 4095
  analogReadResolution(12);

  // ADC 입력 범위 설정
  // 3.3V 근처까지 읽기 위한 설정
  analogSetPinAttenuation(PIN_ROLL, ADC_11db);
  analogSetPinAttenuation(PIN_PITCH, ADC_11db);
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
  analogSetPinAttenuation(PIN_YAW, ADC_11db);

  // 스위치 입력 설정
  pinMode(PIN_KILL, INPUT_PULLUP);
  pinMode(PIN_ELAND, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);
  pinMode(PIN_FIRE, INPUT_PULLUP);

  // LPF 초기값 설정
  axis_roll.filtered_raw     = analogRead(PIN_ROLL);
  axis_pitch.filtered_raw    = analogRead(PIN_PITCH);
  axis_throttle.filtered_raw = analogRead(PIN_THROTTLE);
  axis_yaw.filtered_raw      = analogRead(PIN_YAW);

  Serial.println("ESP32_S3_CONTROLLER_START");
  Serial.println("Sampling rate: 100 Hz");
  Serial.println("Packet format: CTRL,seq,throttle,roll,pitch,yaw,fire,mode,eland,kill");
}


// ============================================================
// loop
// ============================================================

void loop() {
  static uint32_t last_sample_us = 0;

  uint32_t now_us = micros();

  // 100Hz 주기 유지
  if ((uint32_t)(now_us - last_sample_us) >= SAMPLE_PERIOD_US) {
    last_sample_us = now_us;

    // 보정 모드
    if (RAW_DEBUG_MODE) {
      printRawDebug();
      return;
    }

    // 짐벌 값 읽기
    int roll     = readCenterAxis(axis_roll);
    int pitch    = readCenterAxis(axis_pitch);
    int throttle = readThrottleAxis(axis_throttle);
    int yaw      = readCenterAxis(axis_yaw);

    // 스위치 상태 갱신
    updateSwitchLogic();

    // Jetson으로 조종기 패킷 전송
    sendControllerPacket(
      roll,
      pitch,
      throttle,
      yaw,
      kill_state,
      eland_state,
      mode_state,
      fire_latched
    );
  }
}