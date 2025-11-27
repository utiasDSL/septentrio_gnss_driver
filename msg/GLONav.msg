# GLONav block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        freq_nr
# unit of x y z: 1000 meter
float64      x
float64      y
float64      z
# unit of dx dy dz: 1000 meter/second
float32      dx
float32      dy
float32      dz
# unit of ddx ddy ddz: 1000 meter/second^2
float32      ddx
float32      ddy
float32      ddz
# unit of gamma 1Hz/Hz
float32      gamma
# unit of tau t_oe dtau: 1s
float32      tau
float32      dtau
uint32       t_oe
uint16       wn_toe
# unit of p1 and tb: 1 minute
uint8        p1
uint8        p2
# unit of E and n_t: 1 day
uint8        capital_e
uint8        capital_b
uint16       tb
uint8        capital_m
uint8        capital_p
uint8        l_health_flag
uint8        p4
uint16       n_t
# unit of f_t: 0.01 meter
uint16       f_t
uint8        capital_c      255