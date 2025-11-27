# GEOLongTermCorr block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        n
uint8        sb_length
# unit 1Hz²/cycle²
float32      doppler_var_factor

MeasExtraChannel[]  channels