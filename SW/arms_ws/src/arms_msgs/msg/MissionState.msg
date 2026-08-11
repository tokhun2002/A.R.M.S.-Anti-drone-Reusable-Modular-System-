std_msgs/Header header
string  state               # IDLE | SEARCH | LOCK | TRACK | FIRE | RTL
float32 lock_elapsed_sec    # continuous detection duration so far [s]
float32 error_x             # normalized pixel error from frame center [-0.5, 0.5]
float32 error_y
bool    target_locked
float32 kp_now              # 현재 적용 중인 P 게인 (시간 램프 값)
bool    armed               # 유효 ARM (effective_arm: 재토글 래치 반영, CH5 실제 arm)
bool    manual_mode         # true=manual(손제어/Altitude), false=auto(영상유도/Manual)
bool    prearm_blocked      # 수동: arm 스위치 올렸지만 스틱 미(未)idle 로 arm 차단됨 (UI 경고용)
