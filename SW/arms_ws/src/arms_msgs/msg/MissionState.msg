std_msgs/Header header
string  state               # IDLE | SEARCH | LOCK | TRACK | FIRE | RTL
float32 lock_elapsed_sec    # continuous detection duration so far [s]
float32 error_x             # normalized pixel error from frame center [-0.5, 0.5]
float32 error_y
bool    target_locked
