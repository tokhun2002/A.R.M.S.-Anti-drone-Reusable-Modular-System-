# CRSF 텔레메트리 디코딩 통합 (FC → ELRS TX → Jetson).
# 각 필드는 해당 프레임의 '마지막 수신값'을 담는다(프레임마다 도착 주기가 다름).
std_msgs/Header header

# ── 배터리 (CRSF 0x08) ───────────────────────────────
float32 voltage_v               # 전압 [V]
float32 current_a               # 전류 [A]
int32   capacity_used_mah       # 사용 용량 [mAh]
int32   battery_remaining_pct   # 잔량 [%] (control 계산: cell_count 설정 시 전압기반, 아니면 FC 원값)

# ── 자세 Attitude (CRSF 0x1E) [deg] ──────────────────
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

# ── 수직속도 Vario (CRSF 0x07) ───────────────────────
float32 vertical_speed_mps      # 수직 속도 [m/s] (+ 상승)

# ── 비행 모드 (CRSF 0x21) ────────────────────────────
string  flight_mode             # 예: "ANGL", "ACRO", "MANU", "!FS!" 등

# ── 링크 통계 uplink (CRSF 0x14) ─────────────────────
int32   uplink_rssi_dbm         # 상향 RSSI [dBm] (안테나1)
int32   uplink_lq               # 상향 링크 품질 [%]
int32   uplink_snr_db           # 상향 SNR [dB]
