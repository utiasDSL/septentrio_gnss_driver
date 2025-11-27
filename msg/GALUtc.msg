# GALUtc block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        source
# 1 s/s
float32      a_1      -2e10
# 1s
float64      a_0      -2e10
uint32       t_ot
uint8        wn_ot
int8         del_t_ls
uint8        wn_lsf
uint8        dn
int8         del_t_lsf