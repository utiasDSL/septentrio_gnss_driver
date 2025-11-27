# GEOLongTermCorr block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        n
uint8        sb_length
uint8[3]     reserved

LTCorr[]     lt_corr