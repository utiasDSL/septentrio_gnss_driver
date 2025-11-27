# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint16                base_station_id

uint8 FIXED=0
uint8 MOVING=1
uint8 UNKNOWN=255
uint8                  base_type

uint8 RTCM2X3=0
uint8 RTCM2X24=2
uint8 CMR2X1=4
uint8 RTCM3X10051006=8
uint8 RTCMV=9
uint8 CMRPLUS=10
uint8                   source

uint8                   datum    255
uint8                   reserved
float64                 x
float64                 y
float64                 z