# GPSAlm block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
float32      ecc
uint32       t_oa
float32      delta_i
float32      omega_dot
float32      sqrt_a
float32      omega_0
float32      omega
float32      m_0
float32      a_f1
float32      a_f0
uint8        wn_a
uint8        config
uint8        health8
uint8        health6