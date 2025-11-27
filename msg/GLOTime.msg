# GLOTime block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        freq_nr
uint8        n_4
uint8        kp
# unit 1 day
uint16       capital_n
# unit of tau_gps, tau_c: 10^9 nanoseconds
float32      tau_gps
float64      tau_c
# unit 1s
float32      b1
#uint 1s/msd
float32      b2