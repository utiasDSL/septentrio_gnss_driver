# BDSAlm block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        wn_a
# 1 s
uint32       t_oa
float32      sqrt_a
float32      ecc
# omega, m_0, omega_0, delta_i: 1 semi-circle
float32      omega
float32      m_0
float32      omega_0
float32      delta_i
# 1 semi-circle/s
float32      omega_dot
# 1 s
float32      a_f0
# 1 s/s
float32      a_f1
uint16       health
uint8[2]     reserved