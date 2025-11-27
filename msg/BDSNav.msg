# BDSNav block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
uint16       wn
uint8        ura
uint8        sat_h1
uint8        iod_c
uint8        iod_e
uint16       reserved2
float32      t_gd1
float32      t_gd2      -2e10
uint32       t_oc
# 1 s/s²
float32      a_f2
# 1 s/s
float32      a_f1
# 1 s
float32      a_f0
# 1 meter
float32      c_rs
# 1 semi-circle/s
float32      del_n
# 1 semi-circle
float64      m_0
# 1 rad
float32      c_uc
float64      ecc
# 1 rad
float32      c_us
float64      sqrt_a
uint32       t_oe
float32      c_ic
# 1 semi-circle
float64      omega_0
# 1 rad
float32      c_is
# 1 semi-sircle
float64      i_0
# 1 m
float32      c_rc
# 1 semi-circle
float64      omega
# 1 semi-circle/s
float32      omega_dot
# 1 semi-circle/s
float32      i_dot
uint16       wnt_oc
uint16       wnt_oe



















