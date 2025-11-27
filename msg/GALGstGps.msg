# GALGstGps block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        source
# scale*unit  1e9 ns/s
float32      a_1g
# scale*unit  1e9 ns
float32      a_0g
uint32       t_og
uint8        wn_og