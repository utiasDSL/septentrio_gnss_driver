# ReceiverStatus block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        cpu_load 255
uint8        ext_error
uint32       up_time
uint32       rx_state
uint32       rx_error
uint8        n
uint8        sb_length
uint8        cmd_count
uint8        temperature
AGCState[]   agc_states