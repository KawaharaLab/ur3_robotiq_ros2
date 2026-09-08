# Time immediately after the Modbus status response was received.
std_msgs/Header header

# Raw Robotiq input-register values from one status read.
# g_obj retains the complete two-bit object-detection value (0..3).
uint8 g_obj
uint8 g_flt
uint8 g_pr
uint8 g_po
uint8 g_cu

# Wall-clock duration of the synchronous Modbus read that produced this sample.
float32 read_duration_ms
