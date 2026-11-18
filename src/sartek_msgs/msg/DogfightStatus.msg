# Sabit kanatlı İHA takip görev durumu (dogfight_detector/tracker -> mission_dogfight)
std_msgs/Header header

bool    target_visible          # YOLO hedef görüntüde var mı?
bool    target_locked           # Takip kilitlendi mi?
string  target_class            # YOLO sınıf adı (örn: "airplane", "fixed_wing_uav")
float32 confidence              # Tespit güveni (0.0 - 1.0)
float32 bbox_cx                 # Hedef bbox merkezi X (0.0 - 1.0 normalize)
float32 bbox_cy                 # Hedef bbox merkezi Y (0.0 - 1.0 normalize)
float32 bbox_w                  # Hedef bbox genişliği (0.0 - 1.0 normalize)
float32 bbox_h                  # Hedef bbox yüksekliği (0.0 - 1.0 normalize)
float32 error_x                 # Merkez yatay hata (normalize: -1.0 sol .. +1.0 sağ)
float32 error_y                 # Merkez dikey hata (normalize: -1.0 üst .. +1.0 alt)
float32 area_ratio              # Hedefin görüntü alanı oranı (tahmini mesafe)
float32 pid_roll_output         # PID çıktı gözlemi (attitude/rate kontrolü veya log için)
float32 pid_pitch_output
float32 pid_throttle_output
