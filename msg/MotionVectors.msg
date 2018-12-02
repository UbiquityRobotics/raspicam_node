# Message header
std_msgs/Header header

# Number of macroblocks in horizontal dimension
uint8 mbx

# Number of macroblocks in vertical dimension
uint8 mby

# Horizontal component of motion vectors
int8[] x

# Vertical component of motion vectors
int8[] y

# Sum of Absolute Difference metric of motion vectors
uint16[] sad
