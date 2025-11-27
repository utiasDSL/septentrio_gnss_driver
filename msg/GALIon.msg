# GALION block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        source
# 1 10^(-22) W/(m²Hz)
float32      a_i0
# 1 10^(-22) W/(m²Hz) / deg
float32      a_i1
# 1 10^(-22) W/(m²Hz) / deg²
float32      a_i2
uint8        storm_flags
