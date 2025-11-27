# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8 RTCMV2=0
uint8 CMRV2=1
uint8 RTCMV3=2
uint8 RTCMV=3
uint8 SPARTN=4
uint8 RESERVED=5
uint8                    mode

uint8 COM1=0
uint8 COM2=1
uint8 COM3=2
uint8 COM4=3
uint8 USB1=4
uint8 USB2=5
uint8 IPCONNECTION=6
uint8 SBFFILE=7
uint8 LBAND=8
uint8 NTRIP=9
uint8 OTG1=10
uint8 OTG2=11
uint8 BLUETOOTH=12
uint8 UHFMODEM=15
uint8 IPRCONNECTION=16
uint8 DIRECTCALLPORT=17
uint8 IPSCONNECTION=18
uint8 SSR2OBS=19
uint8                     source 255

uint32[]                  rtcm2_words
uint8[]                   cmr_message
uint8[]                   rtcm3_message
uint8[]                   rtcmv_message



