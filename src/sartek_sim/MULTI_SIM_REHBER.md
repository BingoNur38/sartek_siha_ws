# 🛩️ SARTEK Çoklu İHA Simülasyon Rehberi (`AV-15`)

Bu doküman, **Gazebo Harmonic + PX4 SITL (v1.18+) + ROS 2 Humble (Micro XRCE-DDS)** üzerinde 3B yarışma dünyasında **5 sanal sabit kanatlı İHA + 1 kameralı kendi uçağımızın** aynı anda çalıştırılmasını ve test edilmesini açıklar.

> **Mimari Notu:** Eski `multi_ws` yapısından farklı olarak **MAVROS kullanılmaz**. 6 İHA'nın tamamı tek bir `MicroXRCEAgent` (`UDP:8888`) üzerinden izole DDS isim uzaylarıyla haberleşir.

---

## 1. Eklenen ve Güncellenen Dosyalar

| Dosya | Açıklama |
|---|---|
| `src/sartek_sim/worlds/sartek_yarisma.sdf` | Pist işaretleri, hangarlar, kontrol kulesi, sınır pylonları, 3B tepeler, ağaç kümeleri ve yerdeki QR kamikaze hedefini içeren özel 3B yarışma dünyası |
| `src/sartek_sim/launch/multi_sim.launch.py` | 5 sanal uçak (`gz_advanced_plane_1..5`) + 1 kameralı uçak (`gz_advanced_plane_cam`) + `MicroXRCEAgent` + kamera köprüsü + 5 otonom waypoint düğümünü tek komutla başlatır |
| `src/sartek_sim/sartek_sim/target_offboard.py` | 5 sanal İHA'nın otonom `TAKEOFF → ARM → OFFBOARD → Rastgele Waypoint` döngüsünü yürüten PX4 v1.18 uyumlu ROS 2 Python düğümü |
| `src/sartek_sim/setup.py` & `package.xml` | `target_offboard` konsol betiği kaydı ve `rclpy` / `px4_msgs` bağımlılıkları |

---

## 2. 6 İHA Yerleşim ve İsim Uzayı Tablosu

| Uçak | Gazebo Model Adı | PX4 Instance | `MAV_SYS_ID` | ROS 2 DDS İsim Uzayı | Başlangıç Konumu `(x, y, z)` | Görevi / Seyir İrtifası |
|---|---|---|---|---|---|---|
| **6. Uçak (Kendi İHA'mız)** | `gz_advanced_plane_cam` | `0` | `1` | `/fmu/...` *(boş prefix)* | `(0, 0, 0.30)` | Bağımsız kontrol (`sartek_px4_bridge` / QGC) + `/camera/image_raw` |
| **1. Sanal İHA** | `gz_advanced_plane_1` | `1` | `2` | `/gz_advanced_plane_1/fmu/...` | `(0, 25, 0.30)` | Otonom rastgele waypoint (`~55 m`) |
| **2. Sanal İHA** | `gz_advanced_plane_2` | `2` | `3` | `/gz_advanced_plane_2/fmu/...` | `(0, 50, 0.30)` | Otonom rastgele waypoint (`~70 m`) |
| **3. Sanal İHA** | `gz_advanced_plane_3` | `3` | `4` | `/gz_advanced_plane_3/fmu/...` | `(0, 75, 0.30)` | Otonom rastgele waypoint (`~85 m`) |
| **4. Sanal İHA** | `gz_advanced_plane_4` | `4` | `5` | `/gz_advanced_plane_4/fmu/...` | `(0, -25, 0.30)` | Otonom rastgele waypoint (`~100 m`) |
| **5. Sanal İHA** | `gz_advanced_plane_5` | `5` | `6` | `/gz_advanced_plane_5/fmu/...` | `(0, -50, 0.30)` | Otonom rastgele waypoint (`~115 m`) |

---

## 3. Derleme ve Çalıştırma

### Adım 1: Paketi Derleme
```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sartek_sim --symlink-install
source ~/sartek_siha_ws/install/setup.bash
```

### Adım 2: 5 + 1 Çoklu Simülasyonu Başlatma (Terminal 1)
```bash
pkill -9 -f target_offboard 2>/dev/null || true
pkill -9 -f multi_sim.launch.py 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 launch sartek_sim multi_sim.launch.py
```
- `~0–20 sn`: `sartek_yarisma.sdf` dünyası ve 6 uçak yüklenir.
- `~28–45 sn`: 5 sanal uçak (`gz_advanced_plane_1..5`) otonom olarak pistten havalanır ve farklı irtifa katmanlarında rastgele waypoint takibine başlar.
- `gz_advanced_plane_cam` (6. uçak) pistin ortasında bağımsız kontrol için bekler.

### Adım 3: 6. Kameralı Uçağı Bağımsız Kontrol Etme (Terminal 2 & 3)
**Terminal 2 — `sartek_px4_bridge` başlatma:**
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
ros2 launch sartek_px4_bridge px4_bridge.launch.py
```

**Terminal 3 — 6. uçağa kalkış / waypoint komutu gönderme:**
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash

# Kalkış ve ARM
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'TAKEOFF'}"
sleep 2
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'ARM'}"

# (İsteğe bağlı) Kamera görüntüsünü izleme:
ros2 run rqt_image_view rqt_image_view
```

### Adım 4: QGroundControl ile İzleme *(Opsiyonel)*
```bash
qgroundcontrol &
```
> **Not (Ubuntu 22.04):** QGroundControl'ün en güncel AppImage sürümleri `GLIBC_2.38` (Ubuntu 24.04) istediği için Ubuntu 22.04 üzerinde `v4.4.2` AppImage sürümü doğrulanmıştır.
