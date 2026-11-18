# Genel hedef/rakip bilgisi (algı veya hakem trafik verisi -> görevler / target_selector)
std_msgs/Header header

bool    detected
float32 target_x                # Hedef Local ENU X (m)
float32 target_y                # Hedef Local ENU Y (m)
float32 target_z                # Hedef Local ENU Z (m)
float32 confidence
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
string  target_class
int32   target_id
float32 error_x                 # normalize [-1.0, +1.0]
float32 error_y                 # normalize [-1.0, +1.0]
