# ReceiverSetup block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8[2]     reserved
string       marker_name
string       marker_nbr
string       observer
string       agency
string       rx_serial_nbr
string       rx_name
string       rx_version
string       ant_serial_nbr
string       ant_type
float32      delta_h
float32      delta_e
float32      delta_n
string       marker_type
string       gnss_fw_version
string       product_name
float64      latitude           -2e10
float64      longitude          -2e10
float32      height             -2e10
string       station_code
uint8        monument_idx
uint8        receiver_idx
string       country_code