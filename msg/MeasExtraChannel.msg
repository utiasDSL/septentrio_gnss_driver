# MessExtraChannelSub block, Long Time Correction
# subblock of the MeasExtra block

uint8   rx_channel
uint8   type
# scale*unit 0.001m
int16   mp_correction
# scale*unit 0.001m
int16   smoothing_correction
# scale*unit 0.0001m²
uint16  code_var               65535
# scale*unit 1 mcycle^2
uint16  carrier_var            65535
# 1s
uint16  lock_time              65535

uint8   cum_loss_cont
# scale*unit 1.953125 mcycle
int8    carrier_mp_correction

uint8   info

# scale*unit 0.03125 db-Hz
uint8   misc