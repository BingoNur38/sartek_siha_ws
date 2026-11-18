# Genel sistem sağlığı raporu (health_monitor -> coordinator + watchdog)
std_msgs/Header header

float32 battery_voltage
float32 battery_percent         # 0.0 - 100.0
uint8   gps_fix_type            # 0=NoFix 1=2D 2=3D 3=DGPS
uint8   gps_satellites
float32 imu_vibration_x
float32 imu_vibration_y
float32 imu_vibration_z
bool    is_armed
string  flight_mode
bool    rc_connected
bool    is_emergency
string  emergency_reason
