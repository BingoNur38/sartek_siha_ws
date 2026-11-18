# Acil durum güvenlik override komutu (health_monitor/watchdog -> control_bridge)
std_msgs/Header header

bool    override_active
string  override_mode           # "LOITER", "RTL", "FULL_THROTTLE", "CLIMB"
string  reason
float32 override_alt            # Zorla çıkılacak güvenli irtifa (m, 0 = değiştirme)
