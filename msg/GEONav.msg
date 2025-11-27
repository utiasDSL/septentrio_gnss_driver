# GEONav block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
uint16       iod_n
uint16       ura
uint32       t0
float64      x_g
float64      y_g
float64      z_g
float64      dx_g
float64      dy_g
float64      dz_g
float64      ddx_g
float64      ddy_g
float64      ddz_g
float32      ag_f0
float32      ag_f1