# GLOAlm block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        sv_id
uint8        freq_nr
float32      epsilon
# unit of t_oa, t_ln, tau: 1s
uint32       t_oa
# unit of delta_i, glo_lambda, omega: 1 semi-circle
float32      delta_i
float32      glo_lambda
float32      t_ln
float32      omega
# unit 1 s/orbit-period
float32      delta_t
# unit 1 s/orbit-period^2
float32      d_delta_t
float32      tau
uint8        wn_a
uint8        capital_c
uint16       capital_n
uint8        capital_m
uint8        capital_n_4
