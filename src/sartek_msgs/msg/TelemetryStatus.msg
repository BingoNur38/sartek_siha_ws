# Genel uçuş telemetrisi (px4_telemetry_bridge -> tüm görev/karar/GCS düğümleri)
# Standart koordinat sistemi: Local ENU (X=Doğu, Y=Kuzey, Z=Yukarı pozitif)
std_msgs/Header header

# Hız bilgileri (m/s)
float32 airspeed
float32 groundspeed
float32 vel_x
float32 vel_y
float32 vel_z

# Konum ve irtifa (Local ENU, metre)
float32 altitude                # Kalkış / Home referansına göre bağıl irtifa (m)
float32 altitude_amsl           # Deniz seviyesinden yükseklik (AMSL, m)
float32 pos_x
float32 pos_y
float32 pos_z

# Yönelim açıları (derece)
float32 roll_deg
float32 pitch_deg
float32 yaw_deg                 # 0..360 veya ENU/pusula yönelimi (derece)

# Küresel GPS ve Home koordinatları (WGS84)
float64 latitude
float64 longitude
float64 home_latitude
float64 home_longitude
float32 home_altitude
bool    home_set

# Batarya, GPS kalitesi ve IMU titreşimi
float32 battery_voltage
float32 battery_remaining       # 0.0 - 1.0
uint8   gps_fix_type            # 0=NoFix 2=2D 3=3D 4=DGPS 5=RTK_Float 6=RTK_Fixed
int32   gps_satellites
float32 imu_vibration_x
float32 imu_vibration_y
float32 imu_vibration_z

# Özet durum
bool    is_critical
string  warning_message
bool    is_connected
