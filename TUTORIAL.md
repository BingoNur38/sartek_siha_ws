# 🚀 SARTEK Çoklu İHA Simülasyon Tutorial (`5+1 İHA`)

Bu rehber, **`sartek_siha_ws`** reposunu bilgisayarına indiren bir takım arkadaşının **sıfırdan 3B yarışma sahasında 5 otonom sanal İHA + 1 kameralı kendi İHA'mızı** çalıştırabilmesi için gereken tüm adımları sırayla anlatır.

> **Kısa Özet:** Sistem **Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic + PX4 SITL (v1.18+)** ve **Micro XRCE-DDS** ile çalışır (**MAVROS kullanılmaz**).

---

## 0. Başlamadan Önce Sistem Kontrolü

Bilgisayarında aşağıdaki 4 bileşenin kurulu olması gerekir. Terminale şu komutu yapıştırıp hepsinin hazır olduğunu doğrula:

```bash
source /opt/ros/humble/setup.bash
which ros2 gz MicroXRCEAgent
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
echo "RMW: $RMW_IMPLEMENTATION"
```

**Beklenen sonuç:**
- `ros2`, `gz` (Gazebo **Harmonic**), `MicroXRCEAgent` yolları görünmeli.
- `~/PX4-Autopilot/build/px4_sitl_default/bin/px4` dosyası mevcut olmalı.
- `RMW` çıktısı `rmw_cyclonedds_cpp` olmalı *(boşsa her terminalde veya `~/.bashrc` içinde `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ekle)*.

> Bunlardan biri eksikse önce [INSTALLATION.md](INSTALLATION.md) (ROS 2 + MicroXRCEAgent) ve [PX4_SIM_KURULUM.md](PX4_SIM_KURULUM.md) (Gazebo Harmonic + PX4-Autopilot) rehberlerini tamamla.

---

## 1. Repoyu Bilgisayarına İndir

`src/px4_msgs` alt modül (submodule) olarak eklidir; bu yüzden **`--recurse-submodules`** ile indirmen gerekir:

```bash
cd ~
git clone --recurse-submodules https://github.com/BingoNur38/sartek_siha_ws.git
cd ~/sartek_siha_ws
```

> Eğer `--recurse-submodules` olmadan indirdiysen `src/px4_msgs` boş gelir. Hemen şu komutla doldurabilirsin:
> ```bash
> cd ~/sartek_siha_ws
> git submodule update --init --recursive
> ```

---

## 2. Workspace'i Derle (`colcon build`)

```bash
cd ~/sartek_siha_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Eksik ROS bağımlılıklarını yükle
rosdep install --from-paths src --ignore-src -r -y

# Tüm paketleri derle (ilk derleme px4_msgs yüzünden birkaç dakika sürebilir)
colcon build --symlink-install

# Derlenen ortamı yükle
source ~/sartek_siha_ws/install/setup.bash
```

*(İleride sadece `sartek_sim` içinde değişiklik yaparsan hız için `colcon build --packages-select sartek_sim --symlink-install` kullanabilirsin.)*

---

## 3. 5+1 Çoklu İHA Simülasyonunu Başlat (Terminal 1)

Önce arkada kalmış eski simülasyon süreçlerini temizle, sonra tek komutla başlat:

```bash
pkill -9 -f target_offboard 2>/dev/null || true
pkill -9 -f multi_sim.launch.py 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch sartek_sim multi_sim.launch.py
```

### Başlattıktan Sonra Neler Olacak?
- **0 – 20. saniye:** Gazebo Harmonic içinde 3B yarışma sahası (`sartek_yarisma.sdf`), `MicroXRCEAgent`, kamera köprüsü ve **6 uçak** sırayla sahneye yerleşir.
- **28 – 45. saniye:** **5 sanal İHA** (`gz_advanced_plane_1` .. `gz_advanced_plane_5`) otonom olarak pistten kalkar ve `55 m – 115 m` arasındaki farklı irtifa katmanlarında rastgele waypoint'lerde uçmaya başlar.
- **6. uçak (`gz_advanced_plane_cam`):** Pistin ortasında `(0, 0)` konumunda senin kontrolün için bekler.

### Kullanışlı Başlatma Seçenekleri
İhtiyacına göre launch komutunun sonuna şu parametreleri ekleyebilirsin:

| Parametre | Varsayılan | Ne İşe Yarar? |
|---|---|---|
| `px4_dir:=/yol/PX4-Autopilot` | `~/PX4-Autopilot` | PX4 farklı bir klasörde kuruluysa yolunu belirtir |
| `headless:=true` | `false` | Bilgisayar kasıyorsa Gazebo penceresi olmadan arka planda çalıştırır |
| `start_targets:=false` | `true` | 5 sanal uçağın otonom kalkışını kapatır (yerde beklerler) |
| `world_name:=yunuseli` | `sartek_yarisma` | Yarışma dünyası yerine Yunuseli sahasını açar |
| `start_bridge:=false` | `true` | Kamera görüntüsü köprüsünü (`/camera/image_raw`) kapatır |

Örnek (PX4 özel dizindeyse ve Gazebo arayüzü olmadan):
```bash
ros2 launch sartek_sim multi_sim.launch.py px4_dir:=/home/kullanici/PX4-Autopilot headless:=true
```

---

## 4. Kendi Kameralı Uçağımızı (6. Uçak) Uçurma ve İzleme

Simülasyon çalışırken yeni terminaller açıp 6. uçağı (`gz_advanced_plane_cam`) kontrol edebilirsin:

### Terminal 2 — Uçuş Köprüsünü Başlat (`sartek_px4_bridge`)
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch sartek_px4_bridge px4_bridge.launch.py
```

### Terminal 3 — Kalkış Komutu Gönder ve Kontrol Et
```bash
source /opt/ros/humble/setup.bash
source ~/sartek_siha_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 6. uçağa kalkış ve ARM komutu gönder
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'TAKEOFF'}"
sleep 2
ros2 topic pub --once /gorev/vehicle_command std_msgs/msg/String "{data: 'ARM'}"
```

### Telemetri ve Kamerayı Doğrulama
```bash
# 6. uçağın telemetri verisi (ENU)
ros2 topic echo /gorev/telemetry_status --once

# Kamera görüntüsünü pencerede aç
ros2 run rqt_image_view rqt_image_view
# (Açılan pencerede üstten /camera/image_raw seç)

# 6 uçağın DDS topic'lerini listele
ros2 topic list | grep fmu
```

### Uçak İsim Uzayları (Topic Prefix)
| Uçak | Model Adı | ROS 2 Topic Ön Eki | İrtifa / Görev |
|---|---|---|---|
| **Kendi İHA'mız** | `gz_advanced_plane_cam` | `/fmu/...` + `/camera/image_raw` | Bağımsız kontrol (`sartek_px4_bridge` / QGC) |
| **1–5. Sanal İHA** | `gz_advanced_plane_1..5` | `/gz_advanced_plane_1..5/fmu/...` | Otonom uçuş (`55 m, 70 m, 85 m, 100 m, 115 m`) |

---

## 5. QGroundControl ile İzleme *(Opsiyonel)*

Harita üzerinden tüm uçakları görmek istersen:
```bash
qgroundcontrol &
```
> **Ubuntu 22.04 Notu:** Yeni QGC AppImage sürümleri `GLIBC_2.38` (Ubuntu 24.04) istediği için Ubuntu 22.04'te **`v4.4.2`** AppImage sürümünü kullan.

---

## 6. Simülasyonu Kapatma

Terminal 1'de `Ctrl + C` yap. Arkada kalan süreç olursa tek seferde temizle:

```bash
pkill -9 -f target_offboard 2>/dev/null || true
pkill -9 -f multi_sim.launch.py 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
```

---

## 7. Sık Karşılaşılan Hatalar ve Çözümleri

| Hata / Sorun | Sebep | Çözüm |
|---|---|---|
| `PX4 SITL binary bulunamadı` | `~/PX4-Autopilot` derlenmemiş veya başka dizinde | `cd ~/PX4-Autopilot && make px4_sitl gz_advanced_plane` çalıştır (ilk açılınca kapat) veya `px4_dir:=...` ver |
| `px4_msgs` bulunamadı / derlenmiyor | Submodule indirilmemiş | `git submodule update --init --recursive` yapıp tekrar `colcon build --symlink-install` çalıştır |
| Uçaklar kalkmıyor veya `/fmu/...` boş | `RMW_IMPLEMENTATION` farklı veya `~/.bashrc`'de eski `px4_ros_ws` var | Tüm terminallerde `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` olsun; `~/.bashrc`'den eski workspace `source` satırlarını kaldır |
| Gazebo çok kasıyor | 6 adet PX4 + Gazebo GUI ağır geliyor | `ros2 launch sartek_sim multi_sim.launch.py headless:=true` ile GUI olmadan çalıştır |
| `cv_bridge` / `numpy` import hatası | `numpy>=2.0` yüklü | `pip3 install --user "numpy>=1.23.0,<2.0.0" "opencv-python>=4.8.0,<4.9.0"` çalıştır |

Daha detaylı teknik mimari ve geliştirici notları için [README.md](README.md), [ws_rehber.md](ws_rehber.md) ve [src/sartek_sim/MULTI_SIM_REHBER.md](src/sartek_sim/MULTI_SIM_REHBER.md) dosyalarına bakabilirsin.
