Header header # Standard ROS message header

# Doppler Velocity Log (DVL) Bottom Track (Ground Speed)
int32 dvl_validity_time  # Data validity time Time tag in steps of 100 µs
uint8 dvl_id  # 0 : dvl1       - 1 : dvl2
float32 xv1_groundspeed_ms  # Longitudinal ground speed
float32 xv2_groundspeed_ms  # Transverse ground speed
float32 xv3_groundspeed_ms  # Vertical ground speed
float32 dvl_speedofsound_ms
float32 dvl_altitude_m  # Bottom range 
float32 xv1_stddev_ms
float32 xv2_stddev_ms
float32 xv3_stddev_ms
