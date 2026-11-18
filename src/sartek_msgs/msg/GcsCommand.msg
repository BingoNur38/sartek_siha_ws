# Yer Kontrol İstasyonu (GCS) ve Hakem Trafik Komut Paketi
std_msgs/Header header

uint32  msg_id                  # Mesaj sıra numarası (takip için)
float64 timestamp               # Zaman damgası (Unix Epoch)
string  command                 # "START", "ABORT", "PAUSE", "RESUME", "LAND", "RTL", "SWITCH_MISSION", "UPDATE_NFZ"
string  target_mission          # "PATROL", "QR", "DOGFIGHT", "EVASION"

# Koordinat parametreleri (GOTO veya SWITCH_MISSION için)
float64 target_latitude
float64 target_longitude
float64 target_altitude

# Yasaklı Alan (NFZ / HSS) Listesi
NfzInfo[] nfz_list

# Diğer İHA/Hava Araçlarının Trafik Listesi
TargetInfo[] traffic_list

# Manuel rota çizmek/enjekte etmek için Waypoint listesi
float64[] route_latitudes
float64[] route_longitudes
float64[] route_altitudes
