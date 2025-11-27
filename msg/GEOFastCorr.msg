# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        mt
uint8        iod_prn
uint8        iod_fast_corr
uint8        n
uint8        sb_length

FastCorr[]   fast_corr