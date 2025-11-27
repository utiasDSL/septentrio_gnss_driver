# GEOAlm block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved0
uint8        data_id
uint16       health
uint32       t_oa
float64      x_g
float64      y_g
float64      z_g
float64      dx_g
float64      dy_g
float64      dz_g