# NTRIPClientConnection block
# subblock of the NTRIPClientStatus block

uint8         cd_index

uint8 DISABLED=0
uint8 INITIALIZING=1
uint8 RUNNING=2
uint8 ERROR=3
uint8 RETRYING=4
uint8 DUPLICATED=5
uint8         status

uint8 NO_ERROR=0
uint8 INIT_ERROR=1
uint8 AUTH_ERROR=2
uint8 CONN_ERROR=3
uint8 MOUNTPOINT_INVALID=4
uint8 MOUNTPOINT_UNAVAILABLE=5
uint8 WAITING_FOR_GGA=6
uint8 GGA_DISABLED=7
uint8 RESOLVING_HOST_FAILED=8
uint8 OUT_OF_REGION=9
uint8 TLS_SETUP_ERROR=10
uint8 TLS_HANDSHAKE_ERROR=11
uint8 TLS_FINGERPRINT_ERROR=12
uint8 TLS_TIME_NOT_KNOWN=13
uint8 UNKNOWN_ERROR=254
uint8           error_code

uint8           info