# Enum: encoder type
uint8 ENCODER_TYPE_ABSOLUTE = 0
uint8 ENCODER_TYPE_INCREMENTAL = 1

std_msgs/Header header
int64 data
int64 resolution
uint8 type