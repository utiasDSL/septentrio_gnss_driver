# BDSIon block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
# 1 s
float32      alpha_0
float32      beta_0
# 1 s/semi-circle
float32      alpha_1
float32      beta_1
# 1 s/semi-circle²
float32      alpha_2
float32      beta_2
# 1 s/semi-circle³
float32      alpha_3
float32      beta_3
