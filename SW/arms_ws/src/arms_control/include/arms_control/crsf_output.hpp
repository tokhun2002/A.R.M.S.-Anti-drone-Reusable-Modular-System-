#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace arms_control {

class CrsfOutput {
public:
  static constexpr uint16_t CRSF_MIN    = 172;
  static constexpr uint16_t CRSF_CENTER = 992;
  static constexpr uint16_t CRSF_MAX    = 1811;

  using Channels = std::array<uint16_t, 16>;

  struct Frame {
    uint8_t address{0};              // 프레임 목적지 주소를 보관한다.
    uint8_t type{0};                 // 텔레메트리 종류를 보관한다.
    std::vector<uint8_t> payload;    // 타입별 원본 데이터를 보관한다.
  };

  struct ReceiveResult {
    std::vector<Frame> frames;       // CRC 검증을 통과한 응답을 모은다.
    std::size_t bytes_read{0};       // 이번 호출에서 읽은 바이트 수를 센다.
    std::size_t echo_frames{0};      // 자체 송신 에코 수를 센다.
    std::size_t crc_errors{0};       // CRC 불일치 수를 센다.
    std::size_t framing_errors{0};   // 길이 오류 수를 센다.
  };

  // ELRS CRSF에서 주로 쓰는 사용자 지정 전송 속도를 받는다.
  explicit CrsfOutput(std::string port, int baud = 400000);
  ~CrsfOutput();

  /* CRSF RC 채널 프레임을 전송한다. */
  bool send(const Channels & ch);

  /* UART 버퍼에 도착한 CRSF 응답을 논블로킹으로 읽는다. */
  ReceiveResult receive();

  // 정규화 조종값(-1~1)을 CRSF 단위로 바꾼다.
  static uint16_t norm_to_crsf(double v);
  // 정규화 스로틀(0~1)을 CRSF 단위로 바꾼다.
  static uint16_t thr_to_crsf(double v);

private:
  static constexpr std::size_t CRSF_MAX_FRAME_SIZE = 64;   // 사양상 최대 프레임 크기를 제한한다.

  std::string port_;                                       // UART 장치 경로를 보관한다.
  int baud_{400000};                                       // UART 전송 속도를 보관한다.
  int fd_{-1};                                             // 열린 UART 파일 디스크립터를 보관한다.
  std::vector<uint8_t> rx_buffer_;                         // 호출 사이의 미완성 프레임을 유지한다.
  std::array<uint8_t, 26> last_tx_frame_{};                // 자체 송신 에코 판별에 사용한다.
  bool has_last_tx_frame_{false};                          // 비교 가능한 송신 프레임 유무를 나타낸다.

  /* UART 포트를 필요한 시점에 연다. */
  bool try_open();

  /* UART 포트와 남은 수신 데이터를 정리한다. */
  void close_port();

  /* 마지막 송신 프레임과 동일한 수신 에코인지 확인한다. */
  bool is_last_tx_echo(const uint8_t * data, std::size_t size) const;

  static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
  static std::array<uint8_t, 26> build_frame(const Channels & ch);
};

}  // namespace arms_control
