# FlipDotFrame.msg
uint32 width
uint32 height

# Data is packed as a bitmask. 
# Each byte contains 8 pixels.
# Bit 0 of data[0] is pixel (0,0).
uint8[] data