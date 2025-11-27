# GALAlm block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        source
float32      ecc
# 1s
uint32       t_oa
# 1 semi-circle
float32      delta_i
# 1 semi-circle/s
float32      omega_dot
float32      sqrt_a
# 1 semi-circle
float32      omega_0
float32      omega
float32      m_0
# 1 s/s
float32      a_f1
# 1s
float32      a_f0
uint8        wn_a
uint8        sv_id_a
uint16       health
uint8        iod_a