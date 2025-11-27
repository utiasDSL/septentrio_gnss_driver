# GPSNav block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved1
uint16       wn
uint8        ca_or_p_on_l2
uint8        ura
uint8        health
uint8        l2_data_flag
uint16       iodc
uint8        iode2
uint8        iode3
uint8        fit_int_flg
uint8        reserved2
float32      t_gd
uint32       t_oc
float32      a_f2
float32      a_f1
float32      a_f0
float32      c_rs
float32      del_n
float64      m_0
float32      c_uc
float64      ecc
float32      c_us
float64      sqrt_a
uint32       t_oe
float32      c_ic
float64      omega_0
float32      c_is
float64      i_0
float32      c_rc
float64      omega
float32      omega_dot
float32      i_dot
uint16       wnt_oc
uint16       wnt_oe
