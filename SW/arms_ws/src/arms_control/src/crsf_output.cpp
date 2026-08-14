#include "arms_control/crsf_output.hpp"

#include <algorithm>
#include <cerrno>
#include <utility>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <unistd.h>

namespace arms_control {

/* CRSF UART 경로와 전송 속도를 초기화한다. */
CrsfOutput::CrsfOutput(std::string port, int baud)
    : port_(std::move(port)), baud_(baud) {}

/* 열린 CRSF UART를 정리한다. */
CrsfOutput::~CrsfOutput() {
  close_port();
}

/* CRSF UART를 논블로킹 양방향 모드로 연다. */
bool CrsfOutput::try_open() {
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);   // 실기체 UART와 SITL PTY를 함께 지원한다.
  if (fd_ < 0) return false;

  struct termios2 tty{};                                        // 400000 같은 사용자 속도를 설정한다.
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    close_port();
    return false;
  }

  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                   IXON);                                       // 입력 바이트를 가공하지 않는다.
  tty.c_oflag &= ~OPOST;                                        // 출력 바이트를 가공하지 않는다.
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);     // 에코와 줄 단위 처리를 끈다.

  tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);           // 8N1과 무흐름제어를 준비한다.
  tty.c_cflag |= CS8 | CREAD | CLOCAL;                           // 8비트 수신기를 활성화한다.

  tty.c_cflag &= ~CBAUD;                                        // 표준 속도 선택 비트를 지운다.
  tty.c_cflag |= BOTHER;                                        // 사용자 지정 속도를 선택한다.
  tty.c_ispeed = baud_;                                         // 수신 속도를 설정한다.
  tty.c_ospeed = baud_;                                         // 송신 속도를 설정한다.

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    close_port();
    return false;
  }

  // USB-CRSF(CP2102) 연결에서 DTR/RTS 가 assert 되면 모듈을 흔들거나 리셋/부트로더로
  // 오인될 수 있어 내려둔다. (성공 사례 scripts/crsf/test.py 의 dtr=False, rts=False 와 동일)
  // 40핀 UART(ttyTHS*) 에는 모뎀 제어선이 없어 무해하므로 항상 적용한다.
  int modem_bits = TIOCM_DTR | TIOCM_RTS;
  ioctl(fd_, TIOCMBIC, &modem_bits);   // 실패해도 치명적이지 않으므로 반환값은 무시한다.
  return true;
}

/* CRSF RC 채널 프레임 전체를 UART에 전송한다. */
bool CrsfOutput::send(const Channels & ch) {
  if (fd_ < 0 && !try_open()) return false;

  const auto frame = build_frame(ch);                            // 16채널을 단일 CRSF 프레임으로 만든다.
  std::size_t written = 0;                                      // 부분 쓰기를 이어갈 위치를 기록한다.

  while (written < frame.size()) {
    const ssize_t count = ::write(
      fd_, frame.data() + written, frame.size() - written);     // 남은 바이트만 UART에 기록한다.

    if (count > 0) {
      written += static_cast<std::size_t>(count);                // 기록된 만큼 다음 위치로 이동한다.
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }

    close_port();
    return false;
  }

  last_tx_frame_ = frame;                                       // 다음 수신에서 자체 에코를 걸러낸다.
  has_last_tx_frame_ = true;                                    // 에코 비교가 가능함을 표시한다.
  return true;
}

/* UART에서 CRSF 프레임을 읽고 길이와 CRC를 검증한다. */
CrsfOutput::ReceiveResult CrsfOutput::receive() {
  ReceiveResult result;                                         // 이번 호출의 수신 결과를 모은다.
  if (fd_ < 0 && !try_open()) return result;

  std::array<uint8_t, 256> chunk{};                             // 커널 UART 버퍼를 나눠서 비운다.
  while (true) {
    const ssize_t count = ::read(fd_, chunk.data(), chunk.size());   // 도착한 바이트만 논블로킹으로 읽는다.
    if (count > 0) {
      const auto size = static_cast<std::size_t>(count);         // 벡터 삽입에 맞는 크기로 변환한다.
      rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.begin() + size);   // 미완성 데이터 뒤에 이어 붙인다.
      result.bytes_read += size;                                 // 진단용 수신량을 누적한다.
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }

    close_port();
    return result;
  }

  while (rx_buffer_.size() >= 2) {
    const std::size_t body_size = rx_buffer_[1];                 // type부터 CRC까지의 길이를 읽는다.
    if (body_size < 2 || body_size > CRSF_MAX_FRAME_SIZE - 2) {
      rx_buffer_.erase(rx_buffer_.begin());                      // 잘못된 시작 바이트만 버리고 재동기화한다.
      ++result.framing_errors;                                  // 길이 오류를 진단 정보에 남긴다.
      continue;
    }

    const std::size_t frame_size = body_size + 2;                // 주소와 길이 바이트를 포함한 크기를 계산한다.
    if (rx_buffer_.size() < frame_size) {
      break;
    }

    uint8_t crc = 0;                                             // type과 payload의 CRC를 계산한다.
    for (std::size_t i = 2; i + 1 < frame_size; ++i) {
      crc = crc8_dvb_s2(crc, rx_buffer_[i]);                     // 사양의 CRC8-DVB-S2를 누적한다.
    }

    if (crc != rx_buffer_[frame_size - 1]) {
      rx_buffer_.erase(rx_buffer_.begin());                      // CRC 실패 시 한 바이트씩 이동해 재동기화한다.
      ++result.crc_errors;                                      // 파형 또는 속도 문제를 확인할 수 있게 센다.
      continue;
    }

    if (is_last_tx_echo(rx_buffer_.data(), frame_size)) {
      ++result.echo_frames;                                     // 자체 송신 프레임은 텔레메트리에서 제외한다.
    } else {
      Frame frame;                                               // 검증된 텔레메트리 프레임을 구성한다.
      frame.address = rx_buffer_[0];                             // 수신 목적지 주소를 보관한다.
      frame.type = rx_buffer_[2];                                // 텔레메트리 타입을 보관한다.
      frame.payload.assign(
        rx_buffer_.begin() + 3,
        rx_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size - 1));   // CRC를 제외한 payload를 복사한다.
      result.frames.push_back(std::move(frame));                 // 호출자가 처리할 목록에 추가한다.
    }

    rx_buffer_.erase(
      rx_buffer_.begin(),
      rx_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));   // 처리한 프레임을 버퍼에서 제거한다.
  }

  return result;
}

/* UART 포트와 남은 수신 데이터를 정리한다. */
void CrsfOutput::close_port() {
  if (fd_ >= 0) {
    ::close(fd_);                                                // 열린 UART 파일 디스크립터를 닫는다.
    fd_ = -1;                                                    // 다음 호출에서 다시 열도록 표시한다.
  }
  rx_buffer_.clear();                                            // 끊어진 연결의 미완성 데이터를 버린다.
}

/* 마지막으로 보낸 RC 프레임의 자체 수신 에코인지 확인한다. */
bool CrsfOutput::is_last_tx_echo(const uint8_t * data, std::size_t size) const {
  return has_last_tx_frame_ && size == last_tx_frame_.size() &&
         std::equal(last_tx_frame_.begin(), last_tx_frame_.end(), data);   // 전체 바이트가 같을 때만 에코로 본다.
}

/* 정규화 조종값을 CRSF 채널 단위로 변환한다. */
uint16_t CrsfOutput::norm_to_crsf(double v) {
  v = std::clamp(v, -1.0, 1.0);                                 // 입력 범위를 유효한 조종 범위로 제한한다.
  if (v >= 0.0) {
    return static_cast<uint16_t>(CRSF_CENTER + v * (CRSF_MAX - CRSF_CENTER));
  }
  return static_cast<uint16_t>(CRSF_CENTER + v * (CRSF_CENTER - CRSF_MIN));
}

/* 정규화 스로틀을 CRSF 채널 단위로 변환한다. */
uint16_t CrsfOutput::thr_to_crsf(double v) {
  v = std::clamp(v, 0.0, 1.0);                                  // 스로틀 범위를 0에서 1 사이로 제한한다.
  return static_cast<uint16_t>(CRSF_MIN + v * (CRSF_MAX - CRSF_MIN));
}

/* CRSF용 CRC8-DVB-S2 값을 한 바이트 갱신한다. */
uint8_t CrsfOutput::crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;                                                      // 새 바이트를 현재 CRC에 반영한다.
  for (int i = 0; i < 8; ++i) {
    crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);       // 다항식 0xD5로 각 비트를 진행한다.
  }
  return crc;
}

/* 16개 채널을 CRSF RC_CHANNELS_PACKED 프레임으로 만든다. */
std::array<uint8_t, 26> CrsfOutput::build_frame(const Channels & ch) {
  std::array<uint8_t, 26> f{};                                  // 고정 길이 RC 프레임을 초기화한다.
  f[0] = 0xC8;                                                   // 호환되는 CRSF 동기 주소를 기록한다.
  f[1] = 24;                                                     // type과 payload와 CRC 길이를 기록한다.
  f[2] = 0x16;                                                   // RC_CHANNELS_PACKED 타입을 기록한다.

  uint32_t bit = 0;                                              // 22바이트 payload의 현재 비트 위치를 센다.
  for (int i = 0; i < 16; ++i) {
    uint32_t val = std::clamp<uint32_t>(ch[i], CRSF_MIN, CRSF_MAX);   // 채널을 11비트 유효 범위로 제한한다.
    for (int b = 0; b < 11; ++b, ++bit) {
      if (val & (1u << b)) {
        f[3 + bit / 8] |= static_cast<uint8_t>(1u << (bit % 8));     // LSB 우선으로 payload에 채운다.
      }
    }
  }

  uint8_t crc = 0;                                               // type과 payload의 CRC를 계산한다.
  for (int i = 2; i < 25; ++i) {
    crc = crc8_dvb_s2(crc, f[i]);                               // 프레임 내용을 순서대로 반영한다.
  }
  f[25] = crc;                                                   // 마지막 바이트에 CRC를 기록한다.
  return f;
}

}  // namespace arms_control
