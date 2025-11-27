# BDSUtc block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
# 1 s/s
float32      a_1
# 1 s
float32      a_0
int8         del_t_ls
# 1 week
uint8        wn_lsf
# 1 day
uint8        dn
int8         del_t_lsf