# SatInfo block
# subblock of the SatVisibility block

uint8     sv_id
uint8     freq_nr
# scale*unit: 0.01 degrees
uint16    azimuth  65535
# scale*unit: 0.01 degrees
int16     elevation -32768

uint8     rise_set

uint8 ALMANAC=1
uint8 EPHEMERIS=2
uint8 UNKNOWN=255
uint8     satellite_info