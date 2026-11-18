# Görev işçilerinden otopilot kontrol köprüsüne giden yörünge/tutum emri
std_msgs/Header header

string  command_type            # "GOTO", "ORBIT", "ATTITUDE", "ATTITUDE_BYPASS", "RTL", "LOITER", "LAND"
float32 target_x                # Hedef Local ENU X (m)
float32 target_y                # Hedef Local ENU Y (m)
float32 target_z                # Hedef Local ENU Z (m, yukarı pozitif)
float32 speed                   # İstenilen seyir hızı (m/s)
string  state_name              # Mevcut görev durumu (loglama/izleme için)
bool    is_dynamic_target       # Dinamik hedef mi?
string  commander_id            # Yetkili düğüm kimliği (örn: "mission_patrol", "mission_qr")

# Doğrudan tutum / dalış / dogfight kontrolü için (command_type == "ATTITUDE" veya "ATTITUDE_BYPASS")
float32 roll_deg                # İstenilen yatış açısı (derece)
float32 pitch_deg               # İstenilen yunuslama açısı (derece)
float32 yaw_deg                 # İstenilen yönelim açısı (derece)
float32 thrust                  # İstenilen gaz oranı (0.0 - 1.0)
