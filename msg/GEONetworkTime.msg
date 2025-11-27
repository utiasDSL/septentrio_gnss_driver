# GEONetworkTime block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8        prn
uint8        reserved
float32      a_1
float64      a_0
uint32       t_ot
uint8        wn_t
int8         del_t_ls
uint8        wn_lsf
uint8        dn
int8         del_t_lsf
uint8        utc_std
uint16       gps_wn
uint32       gps_tow
uint8        glonass_id