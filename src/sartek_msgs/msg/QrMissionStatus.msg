# QR okuma görev durumu (qr_vision -> mission_qr)
std_msgs/Header header

bool    qr_visible              # Kamera görüş alanında QR / hedef panosu var mı?
bool    qr_decoded              # QR başarıyla okundu ve decode edildi mi?
string  qr_content              # Okunan içerik
float32 error_x                 # Kamera merkezi yatay hata (normalize: -1.0 sol .. +1.0 sağ)
float32 error_y                 # Kamera merkezi dikey hata (normalize: -1.0 üst .. +1.0 alt)
float32 bbox_cx                 # QR bounding box merkezi X (normalize: 0.0 - 1.0)
float32 bbox_cy                 # QR bounding box merkezi Y (normalize: 0.0 - 1.0)
float32 area_ratio              # QR'nin ekran alanı oranı (0.0 - 1.0): yakınlık tahmini
float32 confidence              # Tespit güveni (0.0 - 1.0)
