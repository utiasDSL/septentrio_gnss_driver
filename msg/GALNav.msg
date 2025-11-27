# GALNav block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        source
# 1 m^(1/2)
float64      sqrt_a
# 1 semi-circle
float64      m_0
float64      ecc
# 1 semi-circle
float64      i_0
# 1 semi-circle
float64      omega
# 1 semi-circle
float64      omega_0
# 1 semi-circle/s
float32      omega_dot
# 1 semi-circle/s
float32      i_dot
# 1 semi-circle/s
float32      del_n
# 1 rad
float32      c_uc
# 1 rad
float32      c_us
# 1 rad
float32      c_rc
# 1 rad
float32      c_rs
# 1 rad
float32      c_ic
# 1 rad
float32      c_is
# 1s
uint32       t_oe
uint32       t_oc
# 1 s/s²
float32      a_f2
float32      a_f1
# 1 s
float32      a_f0
# 1 week
uint16       wnt_oe
uint16       wnt_oc
uint16       iod_nav
uint16       health_ossol
uint8        healteh_prs
uint8        sisa_l1_e5a   255
uint8        sisa_l1_e5b   255
uint8        sisa_l1a_e6a  255
# bgd_x 1s
float32      bgd_l1_e5a    -2e10
float32      bgd_l1_e5b    -2e10
float32      bgd_l1a_e6a   -2e10
uint8        c_nav_enc     255


